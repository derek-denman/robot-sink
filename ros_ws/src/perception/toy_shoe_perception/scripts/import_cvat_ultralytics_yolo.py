#!/usr/bin/env python3
"""Import and merge CVAT Ultralytics YOLO exports into a canonical per-sample dataset.

CVAT split artifacts are intentionally ignored; split generation is delegated to
prepare_dataset_split.py.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import shutil
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple

CANONICAL_CLASSES = ["toy", "shoe"]
CANONICAL_NAME_TO_ID = {name: idx for idx, name in enumerate(CANONICAL_CLASSES)}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
SPLIT_ARTIFACTS = {"train.txt", "val.txt", "valid.txt", "test.txt", "data.yaml"}


@dataclass(frozen=True)
class SessionLookup:
    by_basename: Mapping[str, str]
    by_stem: Mapping[str, str]


def _deterministic_id(*parts: str, length: int = 16) -> str:
    digest = hashlib.sha256("||".join(parts).encode("utf-8")).hexdigest()
    return digest[:length]


def _safe_slug(text: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9_\-]", "_", text)
    return slug.strip("_") or "session"


def load_session_lookup(session_map_path: Optional[Path]) -> SessionLookup:
    if session_map_path is None:
        return SessionLookup(by_basename={}, by_stem={})

    payload = json.loads(session_map_path.read_text(encoding="utf-8"))
    frames = payload.get("frames", [])

    by_basename: Dict[str, str] = {}
    by_stem: Dict[str, str] = {}

    for frame in frames:
        rel = Path(str(frame["image_relpath"]))
        session_id = str(frame["session_id"])
        basename = rel.name
        stem = rel.stem

        if basename in by_basename and by_basename[basename] != session_id:
            raise ValueError(f"Ambiguous session for basename {basename}")
        by_basename[basename] = session_id

        if stem in by_stem and by_stem[stem] != session_id:
            # Keep only unambiguous stems.
            by_stem.pop(stem, None)
        elif stem not in by_stem:
            by_stem[stem] = session_id

    return SessionLookup(by_basename=by_basename, by_stem=by_stem)


def parse_data_yaml_for_class_names(data_yaml_path: Path) -> Optional[Dict[int, str]]:
    if not data_yaml_path.exists():
        return None
    try:
        import yaml  # type: ignore
    except ImportError:
        return None

    data = yaml.safe_load(data_yaml_path.read_text(encoding="utf-8")) or {}
    names = data.get("names")
    if isinstance(names, list):
        return {idx: str(name) for idx, name in enumerate(names)}
    if isinstance(names, dict):
        out: Dict[int, str] = {}
        for key, value in names.items():
            out[int(key)] = str(value)
        return out
    return None


def remap_label_lines(
    lines: Sequence[str],
    export_id_to_name: Optional[Mapping[int, str]],
    strict: bool,
) -> Tuple[List[str], Dict[str, int]]:
    rewritten: List[str] = []
    counts = {"toy": 0, "shoe": 0}

    for raw_line in lines:
        line = raw_line.strip()
        if not line:
            continue

        parts = line.split()
        if len(parts) < 5:
            if strict:
                raise ValueError(f"Invalid YOLO label row (expected >=5 columns): {line}")
            continue

        source_idx = int(float(parts[0]))
        if export_id_to_name is None:
            if source_idx not in (0, 1):
                raise ValueError(
                    f"Class index {source_idx} found with no data.yaml class map; expected only 0/1"
                )
            target_idx = source_idx
            class_name = CANONICAL_CLASSES[target_idx]
        else:
            if source_idx not in export_id_to_name:
                raise ValueError(f"Class index {source_idx} missing from export data.yaml mapping")
            class_name = export_id_to_name[source_idx]
            if class_name not in CANONICAL_NAME_TO_ID:
                raise ValueError(f"Unexpected class '{class_name}' in export; expected only toy/shoe")
            target_idx = CANONICAL_NAME_TO_ID[class_name]

        coords = [float(v) for v in parts[1:5]]
        if strict and any(v < 0.0 or v > 1.0 for v in coords):
            raise ValueError(f"Normalized bbox coordinates out of [0,1]: {line}")

        extras = parts[5:]
        rewritten_line = " ".join([str(target_idx)] + [f"{v:.6f}" for v in coords] + extras)
        rewritten.append(rewritten_line)
        counts[class_name] += 1

    return rewritten, counts


def infer_session_id(
    image_name: str,
    image_stem: str,
    export_zip: Path,
    session_lookup: SessionLookup,
) -> str:
    if image_name in session_lookup.by_basename:
        return session_lookup.by_basename[image_name]
    if image_stem in session_lookup.by_stem:
        return session_lookup.by_stem[image_stem]

    match = re.match(r"^(s\d{2,})[_\-].*", image_stem)
    if match:
        return match.group(1)

    return _safe_slug(export_zip.stem)


def discover_exports(exports_dir: Path) -> List[Path]:
    return sorted(path for path in exports_dir.iterdir() if path.is_file() and path.suffix.lower() == ".zip")


def collect_files(root: Path) -> Tuple[List[Path], Dict[str, Path]]:
    image_files: List[Path] = []
    label_by_stem: Dict[str, Path] = {}

    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        suffix = path.suffix.lower()

        if suffix in IMAGE_SUFFIXES:
            image_files.append(path)
            continue

        if suffix == ".txt":
            if path.name.lower() in SPLIT_ARTIFACTS:
                continue
            stem = path.stem
            if stem not in label_by_stem:
                label_by_stem[stem] = path

    return image_files, label_by_stem


def build_outputs(out_dir: Path, clear_existing: bool) -> Tuple[Path, Path]:
    images_all = out_dir / "images" / "all"
    labels_all = out_dir / "labels" / "all"

    if clear_existing and out_dir.exists():
        shutil.rmtree(out_dir)

    images_all.mkdir(parents=True, exist_ok=True)
    labels_all.mkdir(parents=True, exist_ok=True)
    return images_all, labels_all


def import_exports(
    exports: Sequence[Path],
    out_dir: Path,
    session_lookup: SessionLookup,
    strict: bool,
    clear_existing: bool,
) -> Dict[str, Any]:
    images_all, labels_all = build_outputs(out_dir, clear_existing=clear_existing)

    sample_rows: List[Dict[str, Any]] = []
    class_totals = {"toy": 0, "shoe": 0}
    ignored_artifacts = sorted(SPLIT_ARTIFACTS)

    for export_zip in exports:
        with tempfile.TemporaryDirectory(prefix="cvat_import_") as tmp:
            tmp_dir = Path(tmp)
            with zipfile.ZipFile(export_zip, "r") as zf:
                zf.extractall(tmp_dir)

            data_yaml = next(iter(sorted(tmp_dir.rglob("data.yaml"))), None)
            export_id_to_name = parse_data_yaml_for_class_names(data_yaml) if data_yaml else None

            image_files, label_by_stem = collect_files(tmp_dir)
            for image_file in sorted(image_files):
                stem = image_file.stem
                session_id = infer_session_id(
                    image_name=image_file.name,
                    image_stem=stem,
                    export_zip=export_zip,
                    session_lookup=session_lookup,
                )

                label_file = label_by_stem.get(stem)
                label_lines = label_file.read_text(encoding="utf-8").splitlines() if label_file else []
                rewritten_labels, counts = remap_label_lines(
                    lines=label_lines,
                    export_id_to_name=export_id_to_name,
                    strict=strict,
                )

                sample_id = _deterministic_id(export_zip.name, image_file.as_posix())
                image_relpath = f"images/all/{session_id}__{sample_id}{image_file.suffix.lower()}"
                label_relpath = f"labels/all/{session_id}__{sample_id}.txt"

                image_out_path = out_dir / image_relpath
                label_out_path = out_dir / label_relpath

                image_out_path.write_bytes(image_file.read_bytes())
                label_out_path.write_text("\n".join(rewritten_labels) + ("\n" if rewritten_labels else ""), encoding="utf-8")

                class_totals["toy"] += counts["toy"]
                class_totals["shoe"] += counts["shoe"]

                sample_rows.append(
                    {
                        "sample_id": sample_id,
                        "session_id": session_id,
                        "source_export": export_zip.name,
                        "source_image": image_file.relative_to(tmp_dir).as_posix(),
                        "image_relpath": image_relpath,
                        "label_relpath": label_relpath,
                        "num_boxes": counts["toy"] + counts["shoe"],
                        "num_toy": counts["toy"],
                        "num_shoe": counts["shoe"],
                    }
                )

    sample_rows.sort(key=lambda row: (row["session_id"], row["sample_id"]))

    samples_csv = out_dir / "samples.csv"
    with samples_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "sample_id",
                "session_id",
                "source_export",
                "source_image",
                "image_relpath",
                "label_relpath",
                "num_boxes",
                "num_toy",
                "num_shoe",
            ],
        )
        writer.writeheader()
        writer.writerows(sample_rows)

    classes_json = out_dir / "classes.json"
    classes_json.write_text(
        json.dumps(
            {
                "names": CANONICAL_CLASSES,
                "name_to_id": CANONICAL_NAME_TO_ID,
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    import_manifest = {
        "exports": [path.name for path in exports],
        "num_exports": len(exports),
        "num_samples": len(sample_rows),
        "class_totals": class_totals,
        "ignored_cvat_split_artifacts": ignored_artifacts,
        "canonical_classes": CANONICAL_CLASSES,
    }
    (out_dir / "import_manifest.json").write_text(
        json.dumps(import_manifest, indent=2, sort_keys=True),
        encoding="utf-8",
    )

    return import_manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Import/merge CVAT Ultralytics YOLO exports")
    parser.add_argument("--exports-dir", type=Path, required=True, help="Directory containing CVAT export zip files")
    parser.add_argument(
        "--session-map",
        type=Path,
        default=None,
        help="Optional extractor session_index.json to map frames to session IDs",
    )
    parser.add_argument("--out-dir", type=Path, required=True, help="Output canonical dataset directory")
    parser.add_argument(
        "--strict",
        action="store_true",
        default=False,
        help="Fail on malformed bbox rows or out-of-range normalized coordinates",
    )
    parser.add_argument(
        "--clear-existing",
        action="store_true",
        default=False,
        help="Delete existing output directory before import",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.exports_dir.exists() or not args.exports_dir.is_dir():
        raise SystemExit(f"--exports-dir not found or not a directory: {args.exports_dir}")

    exports = discover_exports(args.exports_dir)
    if not exports:
        raise SystemExit(f"No .zip CVAT exports found in {args.exports_dir}")

    session_lookup = load_session_lookup(args.session_map)
    manifest = import_exports(
        exports=exports,
        out_dir=args.out_dir,
        session_lookup=session_lookup,
        strict=bool(args.strict),
        clear_existing=bool(args.clear_existing),
    )

    print(f"Imported {manifest['num_samples']} samples from {manifest['num_exports']} exports")
    print(f"Classes: {manifest['class_totals']}")
    print(f"Ignored CVAT split artifacts: {', '.join(manifest['ignored_cvat_split_artifacts'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
