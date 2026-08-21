"""Paper-faithful Action Chunking Transformer adapted to FFW-SH5 dimensions."""

from dataclasses import asdict, dataclass

import torch
from torch import nn

from .backbone import (
    IMAGENET_MEAN,
    IMAGENET_STD,
    PositionEmbeddingSine2D,
    ResNet18Backbone,
)
from .transformer import (
    PositionalDecoder,
    PositionalDecoderLayer,
    PositionalEncoder,
    PositionalEncoderLayer,
)

ARCHITECTURE_VERSION = 2


@dataclass(frozen=True)
class ACTPolicyConfig:
    """Serializable ACT architecture configuration.

    Defaults mirror Table III of the ALOHA paper. ``state_dim``,
    ``action_dim`` and ``camera_count`` remain configurable because FFW-SH5's
    can-to-box policy uses one arm, a grasp synergy and two cameras.
    """

    state_dim: int = 16
    action_dim: int = 16
    chunk_size: int = 90
    camera_count: int = 2
    hidden_dim: int = 512
    latent_dim: int = 32
    encoder_layers: int = 4
    decoder_layers: int = 7
    feedforward_dim: int = 3200
    attention_heads: int = 8
    dropout: float = 0.1
    backbone: str = "resnet18"
    pretrained_backbone: bool = True
    architecture_version: int = ARCHITECTURE_VERSION

    def __post_init__(self):
        positive = {
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "chunk_size": self.chunk_size,
            "camera_count": self.camera_count,
            "hidden_dim": self.hidden_dim,
            "latent_dim": self.latent_dim,
            "encoder_layers": self.encoder_layers,
            "decoder_layers": self.decoder_layers,
            "feedforward_dim": self.feedforward_dim,
            "attention_heads": self.attention_heads,
        }
        invalid = [name for name, value in positive.items() if int(value) <= 0]
        if invalid:
            raise ValueError(f"ACT dimensions must be positive: {invalid}")
        if self.hidden_dim % self.attention_heads:
            raise ValueError("hidden_dim must be divisible by attention_heads")
        if self.hidden_dim % 4:
            raise ValueError("hidden_dim must be divisible by 4 for 2D positions")
        if not 0.0 <= self.dropout < 1.0:
            raise ValueError("dropout must be within [0,1)")
        if self.backbone != "resnet18":
            raise ValueError("the paper-faithful ACT backbone must be resnet18")
        if self.architecture_version != ARCHITECTURE_VERSION:
            raise ValueError(
                "checkpoint uses an incompatible ACT architecture version: "
                f"{self.architecture_version}")

    def as_dict(self):
        return asdict(self)


def _sinusoidal_positions(length, hidden_dim):
    positions = torch.arange(length, dtype=torch.float32)[:, None]
    dimensions = torch.arange(hidden_dim, dtype=torch.float32)[None, :]
    angles = positions / torch.pow(
        10_000.0, 2 * torch.div(dimensions, 2, rounding_mode="floor")
        / hidden_dim)
    table = torch.empty_like(angles)
    table[:, 0::2] = angles[:, 0::2].sin()
    table[:, 1::2] = angles[:, 1::2].cos()
    return table


class ACTPolicy(nn.Module):
    """CVAE policy that predicts a chunk of absolute joint targets.

    Training uses the demonstration action chunk to infer a latent style. At
    inference the posterior encoder is bypassed and style is fixed to zero, the
    mean of the unit Gaussian prior. Images retain their spatial feature maps;
    they are never globally pooled before transformer attention.
    """

    def __init__(self, config, *, load_backbone_weights=None):
        super().__init__()
        self.config = config
        hidden = config.hidden_dim
        if load_backbone_weights is None:
            load_backbone_weights = config.pretrained_backbone
        self.image_backbone = ResNet18Backbone(
            pretrained=bool(load_backbone_weights))
        self.image_projection = nn.Conv2d(
            self.image_backbone.output_channels, hidden, kernel_size=1)
        self.image_position = PositionEmbeddingSine2D(hidden)
        self.register_buffer(
            "image_mean", torch.tensor(IMAGENET_MEAN).reshape(1, 1, 3, 1, 1))
        self.register_buffer(
            "image_std", torch.tensor(IMAGENET_STD).reshape(1, 1, 3, 1, 1))

        encoder_layer = PositionalEncoderLayer(
            hidden, config.attention_heads, config.feedforward_dim,
            config.dropout)
        self.observation_encoder = PositionalEncoder(
            encoder_layer, config.encoder_layers)
        self.qpos_projection = nn.Linear(config.state_dim, hidden)
        self.latent_projection = nn.Linear(config.latent_dim, hidden)
        self.additional_position = nn.Embedding(2, hidden)

        posterior_layer = PositionalEncoderLayer(
            hidden, config.attention_heads, config.feedforward_dim,
            config.dropout)
        self.posterior_encoder = PositionalEncoder(
            posterior_layer, config.encoder_layers)
        self.posterior_cls = nn.Embedding(1, hidden)
        self.posterior_qpos_projection = nn.Linear(config.state_dim, hidden)
        self.posterior_action_projection = nn.Linear(config.action_dim, hidden)
        self.latent_stats = nn.Linear(hidden, config.latent_dim * 2)
        self.register_buffer(
            "posterior_position",
            _sinusoidal_positions(config.chunk_size + 2, hidden))

        decoder_layer = PositionalDecoderLayer(
            hidden, config.attention_heads, config.feedforward_dim,
            config.dropout)
        self.action_decoder = PositionalDecoder(
            decoder_layer, config.decoder_layers, hidden)
        self.action_queries = nn.Embedding(config.chunk_size, hidden)
        self.action_head = nn.Linear(hidden, config.action_dim)
        # The released model contains this head, but its loss/inference path does
        # not consume it. Keep it so the architecture remains directly comparable.
        self.pad_head = nn.Linear(hidden, 1)

    def _encode_posterior(self, qpos, actions, is_pad):
        batch_size = actions.shape[0]
        cls = self.posterior_cls.weight.expand(batch_size, -1, -1)
        qpos_token = self.posterior_qpos_projection(qpos)[:, None]
        action_tokens = self.posterior_action_projection(actions)
        source = torch.cat((cls, qpos_token, action_tokens), dim=1)
        position = self.posterior_position[None].expand(batch_size, -1, -1)
        prefix_mask = torch.zeros(
            (batch_size, 2), dtype=torch.bool, device=actions.device)
        padding_mask = torch.cat((prefix_mask, is_pad), dim=1)
        encoded = self.posterior_encoder(
            source, position, padding_mask=padding_mask)
        mean, log_variance = self.latent_stats(encoded[:, 0]).chunk(2, dim=-1)
        standard_deviation = torch.exp(0.5 * log_variance)
        latent = mean + standard_deviation * torch.randn_like(mean)
        return latent, mean, log_variance

    def _encode_observation(self, qpos, images, latent):
        batch_size, camera_count = images.shape[:2]
        normalized = (images - self.image_mean) / self.image_std
        features = self.image_backbone(normalized.reshape(
            batch_size * camera_count, *images.shape[2:]))
        features = self.image_projection(features)
        positions = self.image_position(features)
        _, _, height, width = features.shape
        features = features.reshape(
            batch_size, camera_count, self.config.hidden_dim,
            height, width).permute(0, 1, 3, 4, 2)
        positions = positions.reshape(
            batch_size, camera_count, height, width,
            self.config.hidden_dim)
        image_tokens = features.flatten(1, 3)
        image_positions = positions.flatten(1, 3)

        prefix = torch.stack((
            self.latent_projection(latent), self.qpos_projection(qpos)), dim=1)
        prefix_position = self.additional_position.weight[None].expand(
            batch_size, -1, -1)
        source = torch.cat((prefix, image_tokens), dim=1)
        position = torch.cat((prefix_position, image_positions), dim=1)
        return self.observation_encoder(source, position), position

    def forward(self, qpos, images, actions=None, is_pad=None):
        self._validate_inputs(qpos, images, actions, is_pad)
        batch_size = qpos.shape[0]
        if actions is None:
            latent = torch.zeros(
                (batch_size, self.config.latent_dim), device=qpos.device,
                dtype=qpos.dtype)
            mean = log_variance = None
        else:
            if is_pad is None:
                is_pad = torch.zeros(
                    actions.shape[:2], dtype=torch.bool,
                    device=actions.device)
            latent, mean, log_variance = self._encode_posterior(
                qpos, actions, is_pad)

        memory, memory_position = self._encode_observation(
            qpos, images, latent)
        query_position = self.action_queries.weight[None].expand(
            batch_size, -1, -1)
        target = torch.zeros_like(query_position)
        decoded = self.action_decoder(
            target, memory, query_position, memory_position)
        return {
            "actions": self.action_head(decoded),
            "is_pad": self.pad_head(decoded).squeeze(-1),
            "mean": mean,
            "log_variance": log_variance,
        }

    def _validate_inputs(self, qpos, images, actions, is_pad):
        batch_size = qpos.shape[0]
        if qpos.shape != (batch_size, self.config.state_dim):
            raise ValueError(
                "qpos must have shape "
                f"[B,{self.config.state_dim}], got {tuple(qpos.shape)}")
        expected_image_prefix = (batch_size, self.config.camera_count, 3)
        if images.ndim != 5 or tuple(images.shape[:3]) != expected_image_prefix:
            raise ValueError(
                "images must have shape "
                f"[B,{self.config.camera_count},3,H,W], "
                f"got {tuple(images.shape)}")
        if actions is not None:
            expected_actions = (
                batch_size, self.config.chunk_size, self.config.action_dim)
            if tuple(actions.shape) != expected_actions:
                raise ValueError(
                    f"actions must have shape {expected_actions}, "
                    f"got {tuple(actions.shape)}")
            if is_pad is not None and tuple(is_pad.shape) != expected_actions[:2]:
                raise ValueError(
                    f"is_pad must have shape {expected_actions[:2]}, "
                    f"got {tuple(is_pad.shape)}")

    def loss(self, batch, *, kl_weight=10.0):
        """Return the released ACT objective: masked L1 plus beta-weighted KL."""
        output = self(
            batch["qpos"], batch["images"],
            batch["actions"], batch["is_pad"])
        valid = (~batch["is_pad"]).unsqueeze(-1)
        l1 = (torch.abs(
            output["actions"] - batch["actions"]) * valid).mean()
        mean = output["mean"]
        log_variance = output["log_variance"]
        kl = (-0.5 * (
            1.0 + log_variance - mean.square() - log_variance.exp()
        ).sum(dim=-1)).mean()
        total = l1 + float(kl_weight) * kl
        return {"loss": total, "l1": l1, "kl": kl}


__all__ = ["ARCHITECTURE_VERSION", "ACTPolicy", "ACTPolicyConfig"]
