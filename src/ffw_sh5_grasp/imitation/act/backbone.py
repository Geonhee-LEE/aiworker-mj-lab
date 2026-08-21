"""ResNet18 image features and sinusoidal 2D positions for ACT."""

import math

import torch
from torch import nn

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


class FrozenBatchNorm2d(nn.Module):
    """BatchNorm with fixed ImageNet statistics, as used by DETR/ACT."""

    def __init__(self, feature_count):
        super().__init__()
        self.register_buffer("weight", torch.ones(feature_count))
        self.register_buffer("bias", torch.zeros(feature_count))
        self.register_buffer("running_mean", torch.zeros(feature_count))
        self.register_buffer("running_var", torch.ones(feature_count))

    def _load_from_state_dict(
            self, state_dict, prefix, local_metadata, strict, missing_keys,
            unexpected_keys, error_messages):
        state_dict.pop(prefix + "num_batches_tracked", None)
        super()._load_from_state_dict(
            state_dict, prefix, local_metadata, strict, missing_keys,
            unexpected_keys, error_messages)

    def forward(self, values):
        weight = self.weight.reshape(1, -1, 1, 1)
        bias = self.bias.reshape(1, -1, 1, 1)
        scale = weight * (self.running_var.reshape(
            1, -1, 1, 1) + 1e-5).rsqrt()
        offset = bias - self.running_mean.reshape(1, -1, 1, 1) * scale
        return values * scale + offset


class ResNet18Backbone(nn.Module):
    """Return the stride-32 spatial feature map of a shared ResNet18."""

    output_channels = 512

    def __init__(self, *, pretrained=True):
        super().__init__()
        try:
            from torchvision.models import ResNet18_Weights, resnet18
        except ImportError as error:
            raise ImportError(
                "ACT's paper-faithful image encoder requires torchvision; "
                "install requirements-imitation.txt") from error
        weights = ResNet18_Weights.DEFAULT if pretrained else None
        network = resnet18(weights=weights, norm_layer=FrozenBatchNorm2d)
        self.network = nn.Sequential(*tuple(network.children())[:-2])

    def forward(self, images):
        return self.network(images)


class PositionEmbeddingSine2D(nn.Module):
    """Fixed normalized 2D sine/cosine embedding from DETR."""

    def __init__(self, hidden_dim, temperature=10_000.0):
        super().__init__()
        if hidden_dim % 2:
            raise ValueError("hidden_dim must be even for 2D positions")
        self.feature_dim = hidden_dim // 2
        self.temperature = float(temperature)

    def forward(self, feature_map):
        batch, _, height, width = feature_map.shape
        ones = torch.ones(
            (batch, height, width), dtype=torch.bool,
            device=feature_map.device)
        y_position = ones.cumsum(1, dtype=torch.float32)
        x_position = ones.cumsum(2, dtype=torch.float32)
        scale = 2.0 * math.pi
        y_position = y_position / (y_position[:, -1:, :] + 1e-6) * scale
        x_position = x_position / (x_position[:, :, -1:] + 1e-6) * scale
        frequencies = torch.arange(
            self.feature_dim, dtype=torch.float32,
            device=feature_map.device)
        frequencies = self.temperature ** (
            2 * torch.div(frequencies, 2, rounding_mode="floor")
            / self.feature_dim)

        x_angles = x_position[..., None] / frequencies
        y_angles = y_position[..., None] / frequencies
        x_embedding = torch.stack((
            x_angles[..., 0::2].sin(), x_angles[..., 1::2].cos()),
            dim=-1).flatten(3)
        y_embedding = torch.stack((
            y_angles[..., 0::2].sin(), y_angles[..., 1::2].cos()),
            dim=-1).flatten(3)
        return torch.cat((y_embedding, x_embedding), dim=-1).to(
            dtype=feature_map.dtype)


__all__ = [
    "IMAGENET_MEAN",
    "IMAGENET_STD",
    "FrozenBatchNorm2d",
    "PositionEmbeddingSine2D",
    "ResNet18Backbone",
]
