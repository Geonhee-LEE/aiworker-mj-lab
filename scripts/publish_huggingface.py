#!/usr/bin/env python3
"""Publish the validated dataset and ACT policies to Hugging Face Hub."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from huggingface_hub import HfApi

if __package__:
    from .prepare_huggingface_release import (
        DEFAULT_DATASET_DIR,
        DEFAULT_OUTPUTS_DIR,
        DEFAULT_RELEASE_DIR,
        MODEL_ALLOWLIST,
        MODEL_RUNS,
        ROOT,
        prepare,
    )
else:
    from prepare_huggingface_release import (
        DEFAULT_DATASET_DIR,
        DEFAULT_OUTPUTS_DIR,
        DEFAULT_RELEASE_DIR,
        MODEL_ALLOWLIST,
        MODEL_RUNS,
        ROOT,
        prepare,
    )

DATASET_CARD = ROOT / "huggingface" / "dataset" / "README.md"
MODEL_CARD = ROOT / "huggingface" / "model" / "README.md"
EVALUATION_SUMMARY = (
    ROOT
    / "outputs"
    / "evaluation"
    / "can_color_sort_pte_m005"
    / "experiment_summary.csv"
)
CODE_REPO_URL = "https://github.com/ggh-png/aiworker-mj-lab"


def _render_card(path: Path, *, dataset_repo_id: str) -> str:
    return (
        path.read_text(encoding="utf-8")
        .replace("{{DATASET_REPO_ID}}", dataset_repo_id)
        .replace("{{CODE_REPO_URL}}", CODE_REPO_URL)
    )


def _planned_files(dataset_dir: Path, outputs_dir: Path) -> dict:
    dataset_files = sorted(dataset_dir.glob("episode_*.hdf5"))
    model_files = [
        outputs_dir / run_name / relative
        for run_name in MODEL_RUNS.values()
        for relative in MODEL_ALLOWLIST
    ]
    return {
        "dataset_files": len(dataset_files),
        "dataset_bytes": sum(path.stat().st_size for path in dataset_files),
        "model_files": len(model_files),
        "model_bytes": sum(path.stat().st_size for path in model_files),
    }


def _upload_text(api, repo_id, repo_type, path_in_repo, text, message):
    api.upload_file(
        path_or_fileobj=text.encode("utf-8"),
        path_in_repo=path_in_repo,
        repo_id=repo_id,
        repo_type=repo_type,
        commit_message=message,
    )


def set_revision_tag(api, repo_id, repo_type, tag):
    """Move a release tag to the final main commit after all uploads settle."""
    refs = api.list_repo_refs(repo_id, repo_type=repo_type)
    if any(item.name == tag for item in refs.tags):
        api.delete_tag(repo_id, tag=tag, repo_type=repo_type)
    revision = api.repo_info(repo_id, repo_type=repo_type).sha
    api.create_tag(
        repo_id,
        tag=tag,
        tag_message=f"Release {tag}",
        revision=revision,
        repo_type=repo_type,
    )
    return revision


def publish_dataset(api, repo_id, dataset_dir, release_dir, *, private):
    api.create_repo(repo_id, repo_type="dataset", private=private, exist_ok=True)
    _upload_text(
        api,
        repo_id,
        "dataset",
        "README.md",
        _render_card(DATASET_CARD, dataset_repo_id=repo_id),
        "Add dataset card",
    )
    for name in ("dataset_manifest.csv", "dataset_summary.json"):
        api.upload_file(
            path_or_fileobj=release_dir / name,
            path_in_repo=name,
            repo_id=repo_id,
            repo_type="dataset",
            commit_message=f"Add {name}",
        )
    api.upload_folder(
        folder_path=dataset_dir,
        path_in_repo="data",
        repo_id=repo_id,
        repo_type="dataset",
        allow_patterns=["episode_*.hdf5"],
        commit_message="Upload can color sort episodes",
    )


def publish_models(api, repo_id, dataset_repo_id, outputs_dir, release_dir, *, private):
    api.create_repo(repo_id, repo_type="model", private=private, exist_ok=True)
    _upload_text(
        api,
        repo_id,
        "model",
        "README.md",
        _render_card(MODEL_CARD, dataset_repo_id=dataset_repo_id),
        "Add model card",
    )
    api.upload_file(
        path_or_fileobj=release_dir / "model_manifest.json",
        path_in_repo="model_manifest.json",
        repo_id=repo_id,
        repo_type="model",
        commit_message="Add model manifest",
    )
    if EVALUATION_SUMMARY.is_file():
        api.upload_file(
            path_or_fileobj=EVALUATION_SUMMARY,
            path_in_repo="evaluation/experiment_summary.csv",
            repo_id=repo_id,
            repo_type="model",
            commit_message="Add rollout evaluation summary",
        )
    for release_name, run_name in MODEL_RUNS.items():
        api.upload_folder(
            folder_path=outputs_dir / run_name,
            path_in_repo=f"policies/{release_name}",
            repo_id=repo_id,
            repo_type="model",
            allow_patterns=list(MODEL_ALLOWLIST),
            commit_message=f"Upload {release_name} policy",
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-repo-id", required=True, help="namespace/name")
    parser.add_argument("--model-repo-id", required=True, help="namespace/name")
    parser.add_argument("--dataset-dir", type=Path, default=DEFAULT_DATASET_DIR)
    parser.add_argument("--outputs-dir", type=Path, default=DEFAULT_OUTPUTS_DIR)
    parser.add_argument("--release-dir", type=Path, default=DEFAULT_RELEASE_DIR)
    parser.add_argument("--dataset-only", action="store_true")
    parser.add_argument("--model-only", action="store_true")
    parser.add_argument(
        "--revision-tag",
        help="optional Hub tag created at the uploaded revision, for example v3.1.0",
    )
    parser.add_argument(
        "--public", action="store_true", help="create public repositories"
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--skip-sha256",
        action="store_true",
        help="skip hashes while regenerating manifests",
    )
    args = parser.parse_args(argv)
    if args.dataset_only and args.model_only:
        parser.error("--dataset-only and --model-only are mutually exclusive")

    summary, models = prepare(
        args.dataset_dir,
        args.outputs_dir,
        args.release_dir,
        include_hash=not args.skip_sha256,
    )
    plan = {
        "dataset_repo_id": args.dataset_repo_id,
        "model_repo_id": args.model_repo_id,
        "private": not args.public,
        "upload_dataset": not args.model_only,
        "upload_models": not args.dataset_only,
        "revision_tag": args.revision_tag,
        **_planned_files(args.dataset_dir, args.outputs_dir),
        "validated_episodes": summary["episode_count"],
        "models": [item["name"] for item in models],
    }
    print(json.dumps(plan, indent=2, ensure_ascii=False))
    if args.dry_run:
        return 0

    api = HfApi()
    identity = api.whoami()
    print(f"Authenticated as {identity['name']}")
    private = not args.public
    if not args.model_only:
        publish_dataset(
            api,
            args.dataset_repo_id,
            args.dataset_dir,
            args.release_dir,
            private=private,
        )
    if not args.dataset_only:
        publish_models(
            api,
            args.model_repo_id,
            args.dataset_repo_id,
            args.outputs_dir,
            args.release_dir,
            private=private,
        )
    tagged_revisions = {}
    if args.revision_tag:
        if not args.model_only:
            tagged_revisions["dataset"] = set_revision_tag(
                api,
                args.dataset_repo_id,
                "dataset",
                args.revision_tag,
            )
        if not args.dataset_only:
            tagged_revisions["model"] = set_revision_tag(
                api,
                args.model_repo_id,
                "model",
                args.revision_tag,
            )
    print(
        json.dumps(
            {
                "dataset_url": (
                    f"https://huggingface.co/datasets/{args.dataset_repo_id}"
                ),
                "model_url": f"https://huggingface.co/{args.model_repo_id}",
                "private": private,
                "tagged_revisions": tagged_revisions,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
