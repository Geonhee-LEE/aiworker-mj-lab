"""Validated YAML configuration for joint- and task-space ACT training."""

from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

from ..data.schema import ACTION_DIM, RIGHT_POLICY_INDICES
from .policy import ACTPolicyConfig
from .representations import REPRESENTATION_NAMES


@dataclass(frozen=True)
class WandbConfig:
    enabled: bool = False
    project: str = "aiworker-act"
    entity: str | None = None


@dataclass(frozen=True)
class ACTTrainingConfig:
    """One training contract for legacy joint and right-arm Joint/Task runs."""

    run_name: str
    dataset_dir: Path
    output_dir: Path
    camera_names: tuple[str, ...]
    episode_count: int | None = None
    representation: str = "joint"
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
            raise TypeError(f"ACT config must be a YAML mapping: {path}")
        values = dict(values)
        # Legacy 16D configs predate ``policy_side`` and meant both arms.
        if (
            "policy_side" not in values
            and values.get("state_dim", 8) == ACTION_DIM
            and values.get("action_dim", 8) == ACTION_DIM
        ):
            values["policy_side"] = "both"
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
        if self.representation != "joint":
            return None
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
        if self.representation not in REPRESENTATION_NAMES:
            raise ValueError(f"representation must be one of {REPRESENTATION_NAMES}")
        if self.policy_side not in ("both", "right"):
            raise ValueError("policy_side must be 'both' or 'right'")
        if self.representation == "task" and self.policy_side != "right":
            raise ValueError("task representation supports the right arm only")
        expected_dim = (
            ACTION_DIM
            if self.representation == "joint" and self.policy_side == "both"
            else len(RIGHT_POLICY_INDICES)
        )
        if self.state_dim != expected_dim or self.action_dim != expected_dim:
            raise ValueError(
                "state_dim and action_dim must match representation/policy_side "
                f"({expected_dim})"
            )
        if not self.camera_names:
            raise ValueError("camera_names must be non-empty")
        if len(set(self.camera_names)) != len(self.camera_names):
            raise ValueError("camera_names must be unique")
        positive_integers = {
            "batch_size": self.batch_size,
            "epochs": self.epochs,
            "chunk_size": self.chunk_size,
            "num_workers+1": self.num_workers + 1,
            "prefetch_factor": self.prefetch_factor,
            "checkpoint_interval": self.checkpoint_interval,
            "plot_interval": self.plot_interval,
        }
        invalid = [name for name, value in positive_integers.items() if value <= 0]
        if invalid:
            raise ValueError(f"invalid training values: {invalid}")
        if self.episode_count is not None and self.episode_count <= 0:
            raise ValueError("episode_count must be positive when provided")
        fractions = self.validation_fraction + self.test_fraction
        if (
            self.validation_fraction <= 0.0
            or self.test_fraction < 0.0
            or fractions >= 1.0
        ):
            raise ValueError("invalid validation/test fractions")
        if self.learning_rate <= 0.0 or self.backbone_learning_rate <= 0.0:
            raise ValueError("learning rates must be positive")
        if self.weight_decay < 0.0 or self.kl_weight < 0.0:
            raise ValueError("weight decay and KL weight must be non-negative")
        self.policy_config()


__all__ = ["ACTTrainingConfig", "WandbConfig"]
