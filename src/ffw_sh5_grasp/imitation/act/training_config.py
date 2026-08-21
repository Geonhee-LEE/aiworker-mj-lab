"""Validated YAML configuration for ACT training runs."""

from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

from ..data.schema import ACTION_DIM, RIGHT_POLICY_INDICES
from .policy import ACTPolicyConfig


@dataclass(frozen=True)
class WandbConfig:
    enabled: bool = False
    project: str = "aiworker-act"
    entity: str | None = None


@dataclass(frozen=True)
class ACTTrainingConfig:
    """Training, split and output settings loaded from one YAML file."""

    run_name: str
    dataset_dir: Path
    output_dir: Path
    camera_names: tuple[str, ...]
    policy_side: str = "both"
    state_dim: int = ACTION_DIM
    action_dim: int = ACTION_DIM
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
    epochs: int = 200
    learning_rate: float = 1e-5
    backbone_learning_rate: float = 1e-5
    weight_decay: float = 1e-4
    kl_weight: float = 10.0
    validation_fraction: float = 0.1
    test_fraction: float = 0.1
    split_seed: int = 42
    training_seed: int = 1
    num_workers: int = 0
    device: str = "auto"
    rerun: bool = True
    wandb: WandbConfig = field(default_factory=WandbConfig)

    @classmethod
    def load(cls, path):
        path = Path(path)
        with path.open("r", encoding="utf-8") as stream:
            values = yaml.safe_load(stream)
        if not isinstance(values, dict):
            raise TypeError(f"ACT config must be a YAML mapping: {path}")
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
    def policy_indices(self):
        if self.policy_side == "both":
            return tuple(range(ACTION_DIM))
        return RIGHT_POLICY_INDICES

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
        if self.policy_side not in ("both", "right"):
            raise ValueError("policy_side must be 'both' or 'right'")
        if not self.camera_names:
            raise ValueError("camera_names must be non-empty")
        if len(set(self.camera_names)) != len(self.camera_names):
            raise ValueError("camera_names must be unique")
        if self.policy_side == "right":
            left_cameras = [
                name for name in self.camera_names if "left" in name]
            if left_cameras:
                raise ValueError(
                    "right-arm policy cannot use left cameras: "
                    f"{left_cameras}")
        policy_dim = len(self.policy_indices)
        if self.state_dim != policy_dim or self.action_dim != policy_dim:
            raise ValueError(
                "state_dim and action_dim must match policy_side "
                f"({policy_dim})")
        positive_integers = {
            "batch_size": self.batch_size,
            "epochs": self.epochs,
            "chunk_size": self.chunk_size,
            "num_workers+1": self.num_workers + 1,
        }
        invalid = [
            name for name, value in positive_integers.items() if value <= 0]
        if invalid:
            raise ValueError(f"invalid training dimensions: {invalid}")
        fractions = self.validation_fraction + self.test_fraction
        if (self.validation_fraction <= 0 or self.test_fraction < 0
                or fractions >= 1):
            raise ValueError(
                "validation_fraction must be positive; test_fraction must be "
                "non-negative; their sum must be below 1")
        if self.learning_rate <= 0 or self.backbone_learning_rate <= 0:
            raise ValueError("learning rates must be positive")
        if self.weight_decay < 0 or self.kl_weight < 0:
            raise ValueError("weight_decay and kl_weight must be non-negative")
        # Also validates architecture-only constraints in one place.
        self.policy_config()


__all__ = ["ACTTrainingConfig", "WandbConfig"]
