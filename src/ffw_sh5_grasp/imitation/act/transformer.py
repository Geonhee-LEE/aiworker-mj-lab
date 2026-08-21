"""DETR-style transformer blocks used by ACT.

PyTorch's stock transformer accepts positional embeddings only as part of the
input tensor.  DETR, and therefore the released ACT implementation, adds them
to the attention queries and keys at every layer.  Keeping that small semantic
difference in this module makes the policy architecture easier to compare with
the paper without vendoring the full DETR repository.
"""

from copy import deepcopy

from torch import nn


class PositionalEncoderLayer(nn.Module):
    """Post-normalized transformer encoder layer with DETR positions."""

    def __init__(self, hidden_dim, attention_heads, feedforward_dim, dropout):
        super().__init__()
        self.self_attention = nn.MultiheadAttention(
            hidden_dim, attention_heads, dropout=dropout, batch_first=True)
        self.feedforward = nn.Sequential(
            nn.Linear(hidden_dim, feedforward_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(feedforward_dim, hidden_dim),
        )
        self.attention_norm = nn.LayerNorm(hidden_dim)
        self.feedforward_norm = nn.LayerNorm(hidden_dim)
        self.attention_dropout = nn.Dropout(dropout)
        self.feedforward_dropout = nn.Dropout(dropout)

    def forward(self, source, position, *, padding_mask=None):
        query = key = source + position
        attended = self.self_attention(
            query, key, source, key_padding_mask=padding_mask,
            need_weights=False)[0]
        source = self.attention_norm(
            source + self.attention_dropout(attended))
        return self.feedforward_norm(
            source + self.feedforward_dropout(self.feedforward(source)))


class PositionalEncoder(nn.Module):
    """Stack positional encoder layers without a final layer norm."""

    def __init__(self, layer, layer_count):
        super().__init__()
        self.layers = nn.ModuleList(
            deepcopy(layer) for _ in range(int(layer_count)))

    def forward(self, source, position, *, padding_mask=None):
        output = source
        for layer in self.layers:
            output = layer(output, position, padding_mask=padding_mask)
        return output


class PositionalDecoderLayer(nn.Module):
    """Post-normalized DETR decoder layer for action-query generation."""

    def __init__(self, hidden_dim, attention_heads, feedforward_dim, dropout):
        super().__init__()
        self.self_attention = nn.MultiheadAttention(
            hidden_dim, attention_heads, dropout=dropout, batch_first=True)
        self.cross_attention = nn.MultiheadAttention(
            hidden_dim, attention_heads, dropout=dropout, batch_first=True)
        self.feedforward = nn.Sequential(
            nn.Linear(hidden_dim, feedforward_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(feedforward_dim, hidden_dim),
        )
        self.self_attention_norm = nn.LayerNorm(hidden_dim)
        self.cross_attention_norm = nn.LayerNorm(hidden_dim)
        self.feedforward_norm = nn.LayerNorm(hidden_dim)
        self.self_attention_dropout = nn.Dropout(dropout)
        self.cross_attention_dropout = nn.Dropout(dropout)
        self.feedforward_dropout = nn.Dropout(dropout)

    def forward(self, target, memory, query_position, memory_position):
        query = key = target + query_position
        attended = self.self_attention(
            query, key, target, need_weights=False)[0]
        target = self.self_attention_norm(
            target + self.self_attention_dropout(attended))
        attended = self.cross_attention(
            target + query_position, memory + memory_position, memory,
            need_weights=False)[0]
        target = self.cross_attention_norm(
            target + self.cross_attention_dropout(attended))
        return self.feedforward_norm(
            target + self.feedforward_dropout(self.feedforward(target)))


class PositionalDecoder(nn.Module):
    """Stack DETR decoder layers and normalize the final action tokens."""

    def __init__(self, layer, layer_count, hidden_dim):
        super().__init__()
        self.layers = nn.ModuleList(
            deepcopy(layer) for _ in range(int(layer_count)))
        self.norm = nn.LayerNorm(hidden_dim)

    def forward(self, target, memory, query_position, memory_position):
        output = target
        for layer in self.layers:
            output = layer(
                output, memory, query_position, memory_position)
        return self.norm(output)


__all__ = [
    "PositionalDecoder", "PositionalDecoderLayer", "PositionalEncoder",
    "PositionalEncoderLayer",
]
