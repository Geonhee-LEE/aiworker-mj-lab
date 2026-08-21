"""Train ACT and persist reproducible checkpoints, metrics and split metadata."""

import json
import random
import time

import numpy as np
import torch
import yaml
from torch.utils.data import DataLoader

from ..visualization.rerun_training import TrainingRerunLogger
from ..visualization.wandb_training import TrainingWandbLogger
from .dataset_loader import (
    ACTEpisodeDataset,
    compute_stats,
    episode_paths,
    save_stats,
    split_episodes,
)
from .policy import ACTPolicy
from .training_config import ACTTrainingConfig
from .training_output import plot_metric, write_metrics


def _device(name):
    if name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(name)


def _device_description(device):
    if device.type != "cuda":
        return str(device)
    return f"{device} ({torch.cuda.get_device_name(device)})"


def _seed_everything(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _average_metrics(rows):
    if not rows:
        raise ValueError("cannot average an empty data loader")
    return {
        key: float(np.mean([row[key] for row in rows]))
        for key in rows[0]
    }


def _run_epoch(
        policy, loader, device, *, optimizer=None, kl_weight=10.0,
        global_step=0):
    training = optimizer is not None
    policy.train(training)
    rows = []
    for batch in loader:
        batch = {
            name: value.to(device, non_blocking=True)
            for name, value in batch.items()
        }
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            losses = policy.loss(batch, kl_weight=kl_weight)
            if training:
                losses["loss"].backward()
                optimizer.step()
                global_step += 1
        rows.append({
            name: float(value.detach().cpu())
            for name, value in losses.items()
        })
    return _average_metrics(rows), global_step


def _optimizer(policy, config):
    backbone_parameters = []
    policy_parameters = []
    for name, parameter in policy.named_parameters():
        if not parameter.requires_grad:
            continue
        target = (backbone_parameters
                  if name.startswith("image_backbone.")
                  else policy_parameters)
        target.append(parameter)
    return torch.optim.AdamW([
        {"params": policy_parameters},
        {"params": backbone_parameters,
         "lr": config.backbone_learning_rate},
    ], lr=config.learning_rate, weight_decay=config.weight_decay)


def _write_run_metadata(config, splits):
    run_dir = config.run_dir
    for child in ("checkpoints", "metrics", "plots", "rerun"):
        (run_dir / child).mkdir(parents=True, exist_ok=True)
    with (run_dir / "config.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config.as_dict(), stream, sort_keys=False)
    with (run_dir / "episode_splits.json").open(
            "w", encoding="utf-8") as stream:
        json.dump({
            name: [str(path) for path in paths]
            for name, paths in splits.items()
        }, stream, indent=2)


def _checkpoint(policy, optimizer, policy_config, config, epoch, global_step,
                validation_loss):
    return {
        "model": policy.state_dict(),
        "optimizer": optimizer.state_dict(),
        "epoch": epoch,
        "global_step": global_step,
        "policy_config": policy_config.as_dict(),
        "camera_names": list(config.camera_names),
        "policy_indices": config.policy_indices,
        "validation_loss": validation_loss,
        "training_seed": config.training_seed,
    }


def train(config_path):
    """Train the YAML-configured ACT run and return its output directory."""
    config = ACTTrainingConfig.load(config_path)
    _seed_everything(config.training_seed)

    paths = episode_paths(config.dataset_dir)
    train_paths, validation_paths, test_paths = split_episodes(
        paths, config.validation_fraction, config.test_fraction,
        config.split_seed)
    splits = {
        "train": train_paths,
        "validation": validation_paths,
        "test": test_paths,
    }
    _write_run_metadata(config, splits)

    stats = compute_stats(
        train_paths, qpos_indices=config.policy_indices,
        action_indices=config.policy_indices)
    save_stats(stats, config.run_dir / "dataset_stats.pkl")
    dataset_options = {
        "stats": stats,
        "camera_names": config.camera_names,
        "chunk_size": config.chunk_size,
        "qpos_indices": config.policy_indices,
        "action_indices": config.policy_indices,
    }
    train_dataset = ACTEpisodeDataset(train_paths, **dataset_options)
    validation_dataset = ACTEpisodeDataset(
        validation_paths, **dataset_options)
    loader_options = {
        "batch_size": config.batch_size,
        "num_workers": config.num_workers,
        "pin_memory": torch.cuda.is_available(),
    }
    train_loader = DataLoader(
        train_dataset, shuffle=True, **loader_options)
    validation_loader = DataLoader(
        validation_dataset, shuffle=False, **loader_options)

    policy_config = config.policy_config()
    device = _device(config.device)
    policy = ACTPolicy(policy_config).to(device)
    optimizer = _optimizer(policy, config)
    history = []
    global_step = 0
    best_loss = float("inf")
    started = time.perf_counter()
    print(
        f"training device={_device_description(device)} "
        f"epochs={config.epochs} train_batches={len(train_loader)} "
        f"validation_batches={len(validation_loader)}",
        flush=True)

    rerun_path = config.run_dir / "rerun/training.rrd"
    with (
            TrainingRerunLogger(
                rerun_path, enabled=config.rerun) as rerun_logger,
            TrainingWandbLogger(
                enabled=config.wandb.enabled,
                project=config.wandb.project,
                entity=config.wandb.entity,
                run_name=config.run_name,
                config=config.as_dict()) as wandb_logger):
        for epoch in range(config.epochs):
            train_metrics, global_step = _run_epoch(
                policy, train_loader, device, optimizer=optimizer,
                kl_weight=config.kl_weight, global_step=global_step)
            validation_metrics, _ = _run_epoch(
                policy, validation_loader, device,
                kl_weight=config.kl_weight)
            row = {"epoch": epoch, "global_step": global_step}
            row.update({
                f"train/{name}": value
                for name, value in train_metrics.items()
            })
            row.update({
                f"val/{name}": value
                for name, value in validation_metrics.items()
            })
            row["learning_rate"] = optimizer.param_groups[0]["lr"]
            row["elapsed_time"] = time.perf_counter() - started
            history.append(row)
            rerun_logger.log_epoch(row)
            wandb_logger.log_epoch(row)

            checkpoint = _checkpoint(
                policy, optimizer, policy_config, config, epoch, global_step,
                row["val/loss"])
            torch.save(
                checkpoint,
                config.run_dir / "checkpoints/policy_last.ckpt")
            if row["val/loss"] < best_loss:
                best_loss = row["val/loss"]
                torch.save(
                    checkpoint,
                    config.run_dir / "checkpoints/policy_best.ckpt")
            write_metrics(history, config.run_dir / "metrics")
            for name in ("loss", "l1", "kl", "learning_rate"):
                plot_metric(
                    history, name, config.run_dir / f"plots/{name}.png")

            completed_epochs = epoch + 1
            elapsed = row["elapsed_time"]
            eta = elapsed / completed_epochs * (
                config.epochs - completed_epochs)
            print(
                f"epoch={completed_epochs:04d}/{config.epochs:04d} "
                f"train={row['train/loss']:.6f} "
                f"val={row['val/loss']:.6f} best={best_loss:.6f} "
                f"elapsed={elapsed:.1f}s eta={eta:.1f}s",
                flush=True)
    return config.run_dir


__all__ = ["train"]
