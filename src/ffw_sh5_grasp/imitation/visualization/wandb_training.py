"""Epoch-aligned Weights & Biases logging for ACT training metrics."""


class TrainingWandbLogger:
    def __init__(self, *, enabled, project, run_name, config, entity=None):
        self.enabled = bool(enabled)
        self.project = project
        self.run_name = run_name
        self.config = config
        self.entity = entity
        self.run = None

    def __enter__(self):
        if not self.enabled:
            return self
        try:
            import wandb
        except ImportError as error:
            raise RuntimeError(
                "config wandb.enabled=true requires: pip install wandb") from error
        options = {
            "project": self.project,
            "name": self.run_name,
            "config": self.config,
        }
        if self.entity:
            options["entity"] = self.entity
        self.run = wandb.init(**options)
        return self

    def log_epoch(self, metrics):
        if self.run is not None:
            self.run.log(metrics, step=int(metrics["global_step"]))

    def __exit__(self, type_, value, traceback):
        if self.run is not None:
            self.run.finish(exit_code=0 if type_ is None else 1)
        return False


__all__ = ["TrainingWandbLogger"]