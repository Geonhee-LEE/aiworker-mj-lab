"""Reusable Rerun Viewer layouts."""


def _blueprint_module():
    try:
        import rerun.blueprint as rrb
    except ImportError as error:
        raise RuntimeError(
            "Rerun visualization requires: pip install rerun-sdk") from error
    return rrb


def dataset_blueprint(camera_names=(
    "cam_high", "cam_right_wrist")):
    rrb = _blueprint_module()
    camera_views = [
        rrb.Spatial2DView(origin=f"/cameras/{name}", name=name)
        for name in camera_names
    ]
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(origin="/robot", name="Robot"),
            rrb.Horizontal(*camera_views),
            rrb.Horizontal(
                rrb.TimeSeriesView(origin="/state", name="Joint state"),
                rrb.TimeSeriesView(origin="/expert", name="Expert action"),
            ),
        ),
        collapse_panels=True,
    )


def live_recording_blueprint(camera_names=(
    "cam_high", "cam_right_wrist")):
    rrb = _blueprint_module()
    camera_views = [
        rrb.Spatial2DView(origin=f"/cameras/{name}", name=name)
        for name in camera_names
    ]
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(*camera_views),
            rrb.Horizontal(
                rrb.TimeSeriesView(origin="/state", name="Joint state"),
                rrb.TimeSeriesView(origin="/expert", name="Expert action"),
                rrb.TimeSeriesView(
                    origin="/task", name="Task / recording status"),
            ),
        ),
        collapse_panels=True,
    )


def training_blueprint():
    rrb = _blueprint_module()
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.TimeSeriesView(origin="/training/loss", name="Loss"),
            rrb.Horizontal(
                rrb.TimeSeriesView(origin="/training/l1", name="L1"),
                rrb.TimeSeriesView(origin="/training/kl", name="KL"),
                rrb.TimeSeriesView(
                    origin="/training/learning_rate", name="Learning rate"),
            ),
        ),
        collapse_panels=True,
    )


def rollout_blueprint(camera_names=(
    "cam_high", "cam_right_wrist")):
    rrb = _blueprint_module()
    camera_views = [
        rrb.Spatial2DView(origin=f"/cameras/{name}", name=name)
        for name in camera_names
    ]
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(*camera_views),
            rrb.Horizontal(
                rrb.TimeSeriesView(origin="/state", name="Robot state"),
                rrb.TimeSeriesView(origin="/policy", name="Policy action"),
                rrb.TimeSeriesView(origin="/task", name="Task"),
            ),
        ),
        collapse_panels=True,
    )


__all__ = [
    "dataset_blueprint",
    "live_recording_blueprint",
    "rollout_blueprint",
    "training_blueprint",
]
