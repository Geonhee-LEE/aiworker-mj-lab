"""Epoch-aligned Rerun logging for ACT training metrics."""

from pathlib import Path

from .rerun_blueprints import training_blueprint


class TrainingRerunLogger:
    def __init__(self, path, *, enabled=True):
        self.path = Path(path)
        self.enabled = bool(enabled)
        self.recording = None

    def __enter__(self):
        if not self.enabled:
            return self
        try:
            import rerun as rr
        except ImportError as error:
            raise RuntimeError(
                "config rerun=true requires: pip install rerun-sdk"
            ) from error
        self.rr = rr
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.recording = rr.RecordingStream("aiworker_act_training")
        self.recording.__enter__()
        self.recording.save(self.path, default_blueprint=training_blueprint())
        return self

    def log_epoch(self, metrics):
        if self.recording is None:
            return
        self.recording.set_time("epoch", sequence=int(metrics["epoch"]))
        for split in ("train", "val"):
            for name in ("loss", "l1", "kl"):
                key = f"{split}/{name}"
                if key in metrics:
                    self.recording.log(
                        f"training/{name}/{split}", self.rr.Scalars(metrics[key])
                    )
        self.recording.log(
            "training/learning_rate", self.rr.Scalars(metrics["learning_rate"])
        )

    def __exit__(self, type_, value, traceback):
        if self.recording is not None:
            return self.recording.__exit__(type_, value, traceback)
        return False


__all__ = ["TrainingRerunLogger"]
