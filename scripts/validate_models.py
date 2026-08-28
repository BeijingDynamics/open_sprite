#!/usr/bin/env python3
"""Load the public Sprite models and print compact MuJoCo sanity checks."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODELS = (
    ROOT / "models/sprite0825/mujoco/scene.xml",
    ROOT / "models/sprite0825/mujoco/scene_external_pd.xml",
)


def validate(path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    print(
        f"OK {path.relative_to(ROOT)} "
        f"nbody={model.nbody} njnt={model.njnt} nv={model.nv} "
        f"ngeom={model.ngeom} nmesh={model.nmesh} nu={model.nu} "
        f"timestep={model.opt.timestep:g}"
    )


def validate_urdf(path: Path, package_root: Path) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    references: list[tuple[str, Path]] = []
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://sprite0825/"):
            relative = filename.removeprefix("package://sprite0825/")
            references.append((filename, package_root / relative))
        elif filename:
            references.append((filename, path.parent / filename))
    missing = [filename for filename, resolved in references if not resolved.resolve().is_file()]
    if missing:
        raise FileNotFoundError(f"URDF mesh references are missing: {missing}")
    print(
        f"OK {path.relative_to(ROOT)} links={len(root.findall('link'))} "
        f"joints={len(root.findall('joint'))} mesh_refs={len(references)} "
        f"unique_mesh_refs={len({filename for filename, _ in references})}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", nargs="*", type=Path, help="MJCF files to load")
    args = parser.parse_args()
    paths = args.models or DEFAULT_MODELS
    for path in paths:
        validate(path if path.is_absolute() else ROOT / path)
    validate_urdf(
        ROOT / "models/sprite0825/sprite0825_sanitized_v4.urdf",
        ROOT / "models/sprite0825",
    )


if __name__ == "__main__":
    main()
