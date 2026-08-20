"""Compact CVAE transformer that predicts ACT-style future action chunks."""

from dataclasses import asdict, dataclass
import math

import torch
from torch import nn
import torch.nn.functional as functional


@dataclass(frozen=True)
class ACTPolicyConfig:
    state_dim: int = 16
    action_dim: int = 16
    chunk_size: int = 32
    camera_count: int = 3
    hidden_dim: int = 256
    latent_dim: int = 32
    transformer_layers: int = 4
    attention_heads: int = 8
    dropout: float = 0.1

    def as_dict(self):
        return asdict(self)


class _ImageEncoder(nn.Module):
    def __init__(self, hidden_dim):
        super().__init__()
        self.network = nn.Sequential(
            nn.Conv2d(3, 32, 5, stride=2, padding=2), nn.ReLU(),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(64, 128, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(128, hidden_dim, 3, stride=2, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d(1),
        )

    def forward(self, image):
        return self.network(image).flatten(1)


class ACTPolicy(nn.Module):
    """Condition a transformer decoder on qpos, RGB cameras and CVAE latent."""

    def __init__(self, config):
        super().__init__()
        self.config = config
        if config.hidden_dim % config.attention_heads:
            raise ValueError("hidden_dim must be divisible by attention_heads")
        hidden = config.hidden_dim
        self.image_encoder = _ImageEncoder(hidden)
        self.qpos_projection = nn.Linear(config.state_dim, hidden)
        self.camera_embedding = nn.Parameter(
            torch.randn(config.camera_count, hidden) / math.sqrt(hidden))
        self.source_position = nn.Parameter(
            torch.randn(config.camera_count + 1, hidden) / math.sqrt(hidden))
        encoder_layer = nn.TransformerEncoderLayer(
            hidden, config.attention_heads, hidden * 4, config.dropout,
            batch_first=True, norm_first=True)
        self.observation_encoder = nn.TransformerEncoder(
            encoder_layer, config.transformer_layers, enable_nested_tensor=False)

        self.action_projection = nn.Linear(config.action_dim, hidden)
        self.posterior_cls = nn.Parameter(torch.zeros(1, 1, hidden))
        self.posterior_position = nn.Parameter(
            torch.randn(config.chunk_size + 2, hidden) / math.sqrt(hidden))
        posterior_layer = nn.TransformerEncoderLayer(
            hidden, config.attention_heads, hidden * 4, config.dropout,
            batch_first=True, norm_first=True)
        self.posterior = nn.TransformerEncoder(
            posterior_layer, 2, enable_nested_tensor=False)
        self.latent_stats = nn.Linear(hidden, config.latent_dim * 2)
        self.latent_projection = nn.Linear(config.latent_dim, hidden)

        decoder_layer = nn.TransformerDecoderLayer(
            hidden, config.attention_heads, hidden * 4, config.dropout,
            batch_first=True, norm_first=True)
        self.decoder = nn.TransformerDecoder(
            decoder_layer, config.transformer_layers)
        self.action_queries = nn.Parameter(
            torch.randn(config.chunk_size, hidden) / math.sqrt(hidden))
        self.action_head = nn.Linear(hidden, config.action_dim)
        self.pad_head = nn.Linear(hidden, 1)

    def _posterior(self, qpos_token, actions, is_pad):
        batch = actions.shape[0]
        cls = self.posterior_cls.expand(batch, -1, -1)
        tokens = torch.cat((
            cls, qpos_token[:, None, :], self.action_projection(actions)), dim=1)
        tokens = tokens + self.posterior_position[None, :tokens.shape[1]]
        padding = torch.cat((
            torch.zeros((batch, 2), dtype=torch.bool, device=actions.device),
            is_pad), dim=1)
        encoded = self.posterior(tokens, src_key_padding_mask=padding)
        mean, log_variance = self.latent_stats(encoded[:, 0]).chunk(2, dim=-1)
        latent = mean + torch.exp(0.5 * log_variance) * torch.randn_like(mean)
        return latent, mean, log_variance

    def forward(self, qpos, images, actions=None, is_pad=None):
        if images.ndim != 5:
            raise ValueError("images must have shape [B,Cameras,3,H,W]")
        batch, cameras = images.shape[:2]
        if cameras != self.config.camera_count:
            raise ValueError(f"expected {self.config.camera_count} cameras")
        image_tokens = self.image_encoder(
            images.reshape(batch * cameras, *images.shape[2:]))
        image_tokens = image_tokens.reshape(batch, cameras, -1)
        image_tokens = image_tokens + self.camera_embedding[None]
        qpos_token = self.qpos_projection(qpos)
        source = torch.cat((qpos_token[:, None], image_tokens), dim=1)
        source = source + self.source_position[None]
        memory = self.observation_encoder(source)

        if actions is None:
            latent = torch.zeros(
                (batch, self.config.latent_dim), device=qpos.device,
                dtype=qpos.dtype)
            mean = log_variance = None
        else:
            if is_pad is None:
                is_pad = torch.zeros(
                    actions.shape[:2], dtype=torch.bool, device=actions.device)
            latent, mean, log_variance = self._posterior(
                qpos_token, actions, is_pad)
        query = self.action_queries[None].expand(batch, -1, -1)
        query = query + self.latent_projection(latent)[:, None]
        decoded = self.decoder(query, memory)
        return {
            "actions": self.action_head(decoded),
            "is_pad": self.pad_head(decoded).squeeze(-1),
            "mean": mean,
            "log_variance": log_variance,
        }

    def loss(self, batch, *, kl_weight=10.0):
        output = self(
            batch["qpos"], batch["images"],
            batch["actions"], batch["is_pad"])
        valid = ~batch["is_pad"]
        l1_per_step = torch.abs(output["actions"] - batch["actions"]).mean(-1)
        l1 = l1_per_step[valid].mean()
        pad = functional.binary_cross_entropy_with_logits(
            output["is_pad"], batch["is_pad"].float())
        mean, log_variance = output["mean"], output["log_variance"]
        kl = -0.5 * torch.mean(
            1.0 + log_variance - mean.square() - log_variance.exp())
        total = l1 + pad + float(kl_weight) * kl
        return {"loss": total, "l1": l1, "kl": kl, "pad": pad}


__all__ = ["ACTPolicy", "ACTPolicyConfig"]
