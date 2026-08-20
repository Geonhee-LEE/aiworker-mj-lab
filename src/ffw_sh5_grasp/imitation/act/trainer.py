"""ACT training loop with checkpoints, JSONL/CSV, PNG and Rerun outputs."""

import csv
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw
import torch
from torch.utils.data import DataLoader
import yaml

from .dataset_loader import (
    ACTEpisodeDataset, compute_stats, episode_paths, save_stats, split_episodes)
from .policy import ACTPolicy, ACTPolicyConfig
from ..visualization.rerun_training import TrainingRerunLogger


def _device(name):
    if name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(name)


def _average_metrics(items):
    keys = items[0]
    return {key: float(np.mean([item[key] for item in items])) for key in keys}


def _plot_metric(history, name, path):
    width, height, margin = 960, 540, 60
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    draw.line((margin, margin, margin, height - margin), fill="black", width=2)
    draw.line((margin, height - margin, width - margin, height - margin),
              fill="black", width=2)
    series = []
    for key, color in ((f"train/{name}", "#1874d1"),
                       (f"val/{name}", "#dc3c3c"),
                       (name, "#1874d1")):
        values = [(row["epoch"], row[key]) for row in history if key in row]
        if values and not any(item[0] == key for item in series):
            series.append((key, values, color))
    if not series:
        return
    all_values = [value for _, values, _ in series for _, value in values]
    low, high = min(all_values), max(all_values)
    if high <= low:
        high = low + 1.0
    last_epoch = max(epoch for _, values, _ in series for epoch, _ in values)
    xscale = (width - 2 * margin) / max(1, last_epoch)
    yscale = (height - 2 * margin) / (high - low)
    for index, (label, values, color) in enumerate(series):
        points = [
            (margin + epoch * xscale,
             height - margin - (value - low) * yscale)
            for epoch, value in values
        ]
        if len(points) > 1:
            draw.line(points, fill=color, width=3)
        for point in points:
            draw.ellipse((point[0] - 2, point[1] - 2,
                          point[0] + 2, point[1] + 2), fill=color)
        draw.text((margin + index * 180, 18), label, fill=color)
    if name == "loss" and any(label == "val/loss" for label, _, _ in series):
        values = next(values for label, values, _ in series if label == "val/loss")
        epoch, value = min(values, key=lambda item: item[1])
        point = (margin + epoch * xscale,
                 height - margin - (value - low) * yscale)
        draw.ellipse((point[0] - 6, point[1] - 6,
                      point[0] + 6, point[1] + 6), outline="#008800", width=3)
        draw.text((margin, height - 35),
                  f"best epoch={epoch}, min val={value:.6g}", fill="#008800")
    draw.text((width // 2 - 40, height - 30), "epoch", fill="black")
    draw.text((8, 8), name, fill="black")
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def _write_metrics(history, metrics_dir):
    metrics_dir.mkdir(parents=True, exist_ok=True)
    with (metrics_dir / "metrics.jsonl").open("w", encoding="utf-8") as stream:
        for row in history:
            stream.write(json.dumps(row) + "\n")
    keys = list(history[0])
    with (metrics_dir / "metrics.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(history)


def _run_epoch(policy, loader, device, *, optimizer=None, kl_weight=10.0,
               global_step=0):
    training = optimizer is not None
    policy.train(training)
    rows = []
    for batch in loader:
        batch = {name: value.to(device, non_blocking=True)
                 for name, value in batch.items()}
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            losses = policy.loss(batch, kl_weight=kl_weight)
            if training:
                losses["loss"].backward()
                torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
                optimizer.step()
                global_step += 1
        rows.append({name: float(value.detach().cpu())
                     for name, value in losses.items()})
    return _average_metrics(rows), global_step


def train(config_path):
    config_path = Path(config_path)
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    run_dir = Path(config["output_dir"]) / config["run_name"]
    for child in ("checkpoints", "metrics", "plots", "rerun"):
        (run_dir / child).mkdir(parents=True, exist_ok=True)
    with (run_dir / "config.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)

    paths = episode_paths(config["dataset_dir"])
    train_paths, validation_paths, test_paths = split_episodes(
        paths, config["validation_fraction"], config.get("test_fraction", 0.1),
        config["split_seed"])
    with (run_dir / "episode_splits.json").open("w", encoding="utf-8") as stream:
        json.dump({
            "train": [str(path) for path in train_paths],
            "validation": [str(path) for path in validation_paths],
            "test": [str(path) for path in test_paths],
        }, stream, indent=2)
    stats = compute_stats(train_paths)
    save_stats(stats, run_dir / "dataset_stats.pkl")
    dataset_args = dict(
        stats=stats, camera_names=config["camera_names"],
        chunk_size=config["chunk_size"])
    train_dataset = ACTEpisodeDataset(train_paths, **dataset_args)
    validation_dataset = ACTEpisodeDataset(validation_paths, **dataset_args)
    loader_args = dict(
        batch_size=config["batch_size"], num_workers=config["num_workers"],
        pin_memory=torch.cuda.is_available())
    train_loader = DataLoader(train_dataset, shuffle=True, **loader_args)
    validation_loader = DataLoader(validation_dataset, shuffle=False, **loader_args)

    policy_config = ACTPolicyConfig(
        state_dim=config["state_dim"], action_dim=config["action_dim"],
        chunk_size=config["chunk_size"],
        camera_count=len(config["camera_names"]),
        hidden_dim=config["hidden_dim"], latent_dim=config["latent_dim"],
        transformer_layers=config["transformer_layers"],
        attention_heads=config["attention_heads"], dropout=config["dropout"])
    device = _device(config["device"])
    policy = ACTPolicy(policy_config).to(device)
    optimizer = torch.optim.AdamW(
        policy.parameters(), lr=config["learning_rate"],
        weight_decay=config["weight_decay"])
    history, global_step, best_loss = [], 0, float("inf")
    started = time.perf_counter()
    rerun_path = run_dir / "rerun/training.rrd"
    with TrainingRerunLogger(rerun_path, enabled=config.get("rerun", True)) as logger:
        for epoch in range(config["epochs"]):
            train_metrics, global_step = _run_epoch(
                policy, train_loader, device, optimizer=optimizer,
                kl_weight=config["kl_weight"], global_step=global_step)
            validation_metrics, _ = _run_epoch(
                policy, validation_loader, device,
                kl_weight=config["kl_weight"])
            row = {"epoch": epoch, "global_step": global_step}
            row.update({f"train/{name}": value
                        for name, value in train_metrics.items()})
            row.update({f"val/{name}": value
                        for name, value in validation_metrics.items()})
            row["learning_rate"] = optimizer.param_groups[0]["lr"]
            row["elapsed_time"] = time.perf_counter() - started
            history.append(row)
            logger.log_epoch(row)
            checkpoint = {
                "model": policy.state_dict(), "optimizer": optimizer.state_dict(),
                "epoch": epoch, "global_step": global_step,
                "policy_config": policy_config.as_dict(),
                "camera_names": config["camera_names"],
                "validation_loss": row["val/loss"],
            }
            torch.save(checkpoint, run_dir / "checkpoints/policy_last.ckpt")
            if row["val/loss"] < best_loss:
                best_loss = row["val/loss"]
                torch.save(checkpoint, run_dir / "checkpoints/policy_best.ckpt")
            _write_metrics(history, run_dir / "metrics")
            for name in ("loss", "l1", "kl", "learning_rate"):
                _plot_metric(history, name, run_dir / f"plots/{name}.png")
            print(
                f"epoch={epoch:04d} train={row['train/loss']:.6f} "
                f"val={row['val/loss']:.6f} best={best_loss:.6f}")
    return run_dir


__all__ = ["train"]
