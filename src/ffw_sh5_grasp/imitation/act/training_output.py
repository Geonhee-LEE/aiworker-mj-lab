"""Small, dependency-light writers for ACT training metrics."""

import csv
import json

from PIL import Image, ImageDraw


def write_metrics(history, metrics_dir):
    metrics_dir.mkdir(parents=True, exist_ok=True)
    with (metrics_dir / "metrics.jsonl").open("w", encoding="utf-8") as stream:
        for row in history:
            stream.write(json.dumps(row) + "\n")
    with (metrics_dir / "metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(history[0]))
        writer.writeheader()
        writer.writerows(history)


def plot_metric(history, name, path):
    width, height, margin = 960, 540, 60
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    draw.line((margin, margin, margin, height - margin), fill="black", width=2)
    draw.line(
        (margin, height - margin, width - margin, height - margin),
        fill="black",
        width=2,
    )
    series = _metric_series(history, name)
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
            (margin + epoch * xscale, height - margin - (value - low) * yscale)
            for epoch, value in values
        ]
        if len(points) > 1:
            draw.line(points, fill=color, width=3)
        for point in points:
            draw.ellipse(
                (point[0] - 2, point[1] - 2, point[0] + 2, point[1] + 2), fill=color
            )
        draw.text((margin + index * 180, 18), label, fill=color)
    _mark_best_validation(draw, series, name, margin, height, xscale, yscale, low)
    draw.text((width // 2 - 40, height - 30), "epoch", fill="black")
    draw.text((8, 8), name, fill="black")
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def _metric_series(history, name):
    result = []
    for key, color in (
        (f"train/{name}", "#1874d1"),
        (f"val/{name}", "#dc3c3c"),
        (name, "#1874d1"),
    ):
        values = [(row["epoch"], row[key]) for row in history if key in row]
        if values and not any(item[0] == key for item in result):
            result.append((key, values, color))
    return result


def _mark_best_validation(draw, series, name, margin, height, xscale, yscale, low):
    if name != "loss":
        return
    validation = next(
        (values for label, values, _ in series if label == "val/loss"), None
    )
    if not validation:
        return
    epoch, value = min(validation, key=lambda item: item[1])
    point = (margin + epoch * xscale, height - margin - (value - low) * yscale)
    draw.ellipse(
        (point[0] - 6, point[1] - 6, point[0] + 6, point[1] + 6),
        outline="#008800",
        width=3,
    )
    draw.text(
        (margin, height - 35),
        f"best epoch={epoch}, min val={value:.6g}",
        fill="#008800",
    )


__all__ = ["plot_metric", "write_metrics"]
