"""Configuration dedicated to modular joint/task ACT experiments."""

from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

from .modular_representations import REPRESENTATION_NAMES
from .policy import ACTPolicyConfig
from .training_config import WandbConfig


@dataclass(frozen=True)
class ModularACTTrainingConfig:
    run_name: str
    dataset_dir: Path
    output_dir: Path
    camera_names: tuple[str, ...]
    representation: str
    policy_side: str = "right"
    state_dim: int = 8
    action_dim: int = 8
    chunk_size: int = 90
    hidden_dim: int = 512
    latent_dim: int = 32
    encoder_layers: int = 4
    decoder_layers: int = 7
    feedforward_dim: int = 3200
    attention_heads: int = 8
    dropout: float = 0.1
    backbone: str = "resnet18"
    pretrained_backbone: bool = True
    batch_size: int = 8
    epochs: int = 2000
    learning_rate: float = 1e-5
    backbone_learning_rate: float = 1e-5
    weight_decay: float = 1e-4
    kl_weight: float = 10.0
    validation_fraction: float = 0.1
    test_fraction: float = 0.1
    split_seed: int = 42
    training_seed: int = 1
    num_workers: int = 1
    prefetch_factor: int = 1
    device: str = "auto"
    checkpoint_interval: int = 100
    plot_interval: int = 100
    rerun: bool = True
    wandb: WandbConfig = field(default_factory=WandbConfig)

    @classmethod
    def load(cls, path):
        path = Path(path)
        with path.open("r", encoding="utf-8") as stream:
            values = yaml.safe_load(stream)
        if not isinstance(values, dict):
            raise TypeError(f"modular ACT config must be a mapping: {path}")
        values = dict(values)
        values["dataset_dir"] = Path(values["dataset_dir"])
        values["output_dir"] = Path(values["output_dir"])
        values["camera_names"] = tuple(values["camera_names"])
        wandb_values = values.pop("wandb", {})
        if not isinstance(wandb_values, dict):
            raise TypeError("wandb must be a mapping")
        values["wandb"] = WandbConfig(**wandb_values)
        result = cls(**values)
        result.validate()
        return result

    @property
    def run_dir(self):
        return self.output_dir / self.run_name

    def policy_config(self):
        return ACTPolicyConfig(
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            chunk_size=self.chunk_size,
            camera_count=len(self.camera_names),
            hidden_dim=self.hidden_dim,
            latent_dim=self.latent_dim,
            encoder_layers=self.encoder_layers,
            decoder_layers=self.decoder_layers,
            feedforward_dim=self.feedforward_dim,
            attention_heads=self.attention_heads,
            dropout=self.dropout,
            backbone=self.backbone,
            pretrained_backbone=self.pretrained_backbone,
        )

    def as_dict(self):
        values = asdict(self)
        values["dataset_dir"] = str(self.dataset_dir)
        values["output_dir"] = str(self.output_dir)
        values["camera_names"] = list(self.camera_names)
        return values

    def validate(self):
        if not self.run_name.strip():
            raise ValueError("run_name must be non-empty")
        if self.representation not in REPRESENTATION_NAMES:
            raise ValueError(
                f"representation must be one of {REPRESENTATION_NAMES}")
        if self.policy_side != "right":
            raise ValueError("modular joint/task comparison supports right only")
        if self.state_dim != 8 or self.action_dim != 8:
            raise ValueError(
                "right joint/task comparison requires 8D state and action")
        if not self.camera_names or len(set(self.camera_names)) != len(
                self.camera_names):
            raise ValueError("camera_names must be non-empty and unique")
        positive_integers = {
            "batch_size": self.batch_size,
            "epochs": self.epochs,
            "chunk_size": self.chunk_size,
            "num_workers+1": self.num_workers + 1,
            "prefetch_factor": self.prefetch_factor,
            "checkpoint_interval": self.checkpoint_interval,
            "plot_interval": self.plot_interval,
        }
        invalid = [
            name for name, value in positive_integers.items() if value <= 0]
        if invalid:
            raise ValueError(f"invalid modular training values: {invalid}")
        fractions = self.validation_fraction + self.test_fraction
        if (self.validation_fraction <= 0.0 or self.test_fraction < 0.0
                or fractions >= 1.0):
            raise ValueError("invalid validation/test fractions")
        if self.learning_rate <= 0.0 or self.backbone_learning_rate <= 0.0:
            raise ValueError("learning rates must be positive")
        if self.weight_decay < 0.0 or self.kl_weight < 0.0:
            raise ValueError("weight decay and KL weight must be non-negative")
        self.policy_config()


__all__ = ["ModularACTTrainingConfig"]
