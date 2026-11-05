#!/usr/bin/env python3
"""Load the public Sprite models and print compact MuJoCo sanity checks."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from build_waist_inertia_revision import REVISION_NAME, WAIST_COM, WAIST_INERTIA, WAIST_MASS_KG


ROOT = Path(__file__).resolve().parents[1]
V5 = ROOT / "models/sprite0825_v5_4340_shoulders"
LATEST_V5 = ROOT / "models" / REVISION_NAME
DEFAULT_MODELS = (
    LATEST_V5 / "mujoco/scene.xml",
    LATEST_V5 / "mujoco/scene_external_pd.xml",
    V5 / "mujoco/scene.xml",
    V5 / "mujoco/scene_external_pd.xml",
    ROOT / "models/sprite0825/mujoco/scene.xml",
    ROOT / "models/sprite0825/mujoco/scene_external_pd.xml",
)


def validate(path: Path, expected_nv: int | None = None) -> None:
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    if expected_nv is not None and (model.nv != expected_nv or model.nu != 0):
        raise ValueError(f"Unexpected DoF/actuator count: {path}: nv={model.nv} nu={model.nu}")
    label = path.relative_to(ROOT) if path.is_relative_to(ROOT) else path
    print(
        f"OK {label} "
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


def validate_hashes(root: Path = ROOT) -> None:
    listed = set()
    for line in (root / "ASSET_SHA256SUMS").read_text(encoding="ascii").splitlines():
        expected, relative = line.split("  ", 1)
        path = root / relative
        if relative in listed or not path.resolve().is_relative_to((root / "models").resolve()):
            raise ValueError(f"Invalid checksum entry: {relative}")
        listed.add(relative)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Asset checksum mismatch: {relative}")
    actual = {p.relative_to(root).as_posix() for p in (root / "models").rglob("*") if p.is_file()}
    if listed != actual:
        raise ValueError(f"Checksum inventory mismatch: {sorted(listed ^ actual)}")
    for version in (V5.name, REVISION_NAME):
        asset = root / "models" / version
        manifest = json.loads((asset / "manifest.json").read_text(encoding="utf-8"))
        if hashlib.sha256((asset / manifest["urdf"]).read_bytes()).hexdigest() != manifest["urdf_sha256"]:
            raise ValueError(f"URDF differs from version manifest: {version}")
    print(f"OK asset checksums files={len(listed)}")


def validate_v5_contract(urdf_path: Path, mjcf_path: Path) -> None:
    urdf = ET.parse(urdf_path).getroot()
    mjcf = ET.parse(mjcf_path).getroot()
    joints = {j.get("name"): j for j in urdf.findall("joint") if j.get("type") == "revolute"}
    hinges = {j.get("name"): j for j in mjcf.findall(".//joint") if j.get("type") != "free"}
    if len(joints) != 31 or joints.keys() != hinges.keys():
        raise ValueError("v5 must contain the same 31 serial hinge joints in URDF and MJCF")
    for name, joint in joints.items():
        limit = joint.find("limit")
        hinge = hinges[name]
        axis = [float(v) for v in joint.find("axis").get("xyz").split()]
        other_axis = [float(v) for v in hinge.get("axis").split()]
        bounds = [float(limit.get(k)) for k in ("lower", "upper")]
        if not math.isclose(sum(v * v for v in axis), 1, abs_tol=1e-8):
            raise ValueError(f"Non-unit axis: {name}")
        if not np.allclose(axis, other_axis, rtol=0, atol=1e-8):
            raise ValueError(f"URDF/MJCF axis mismatch: {name}")
        if not np.allclose(bounds, [float(v) for v in hinge.get("range").split()], rtol=0, atol=1e-8):
            raise ValueError(f"URDF/MJCF position-limit mismatch: {name}")
        if not bounds[0] < bounds[1]:
            raise ValueError(f"Invalid position limits: {name}")
        effort = float(limit.get("effort"))
        if not np.allclose([-effort, effort], [float(v) for v in hinge.get("actuatorfrcrange").split()]):
            raise ValueError(f"URDF/MJCF effort metadata mismatch: {name}")
        if "shoulder_pitch" in name or "shoulder_roll" in name:
            if effort != 14 or float(limit.get("velocity")) != 9.3:
                raise ValueError(f"v5 J4340P shoulder contract changed: {name}")
    for link in urdf.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        i = {k: float(v) for k, v in inertial.find("inertia").attrib.items()}
        matrix = np.array([[i["ixx"], i["ixy"], i["ixz"]],
                           [i["ixy"], i["iyy"], i["iyz"]],
                           [i["ixz"], i["iyz"], i["izz"]]])
        if not math.isfinite(mass) or mass <= 0 or not np.isfinite(matrix).all() or np.linalg.eigvalsh(matrix).min() <= 0:
            raise ValueError(f"Invalid mass/inertia: {link.get('name')}")
    print("OK v5 contract hinges=31 J4340P_shoulders=4 axes/limits/inertias")


def validate_waist_revision(asset: Path = LATEST_V5) -> None:
    urdf = ET.parse(asset / "sprite0825_float.urdf").getroot()
    inertial = urdf.find("link[@name='waist_yaw_link']/inertial")
    mass = float(inertial.find("mass").get("value"))
    com = [float(v) for v in inertial.find("origin").get("xyz").split()]
    rpy = [float(v) for v in inertial.find("origin").get("rpy").split()]
    values = inertial.find("inertia").attrib
    if mass != WAIST_MASS_KG or not np.allclose(com, WAIST_COM, rtol=0, atol=1e-12) or rpy != [0, 0, 0]:
        raise ValueError("Owner-supplied waist mass/COM/frame mismatch")
    if any(not math.isclose(float(values[k]), v, rel_tol=0, abs_tol=1e-12) for k, v in WAIST_INERTIA.items()):
        raise ValueError("Owner-supplied waist inertia mismatch")
    i = WAIST_INERTIA
    expected_tensor = np.array([[i['ixx'], i['ixy'], i['ixz']],
                                [i['ixy'], i['iyy'], i['iyz']],
                                [i['ixz'], i['iyz'], i['izz']]])
    for filename in ("sprite0825_v5_external_pd.xml", "sprite0825_v5_visual_locked.xml"):
        path = asset / "mujoco" / filename
        source = ET.parse(path).find(".//body[@name='waist_yaw_link']/inertial")
        raw = [float(v) for v in source.get("fullinertia", "").split()]
        expected_raw = [i[key] for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        if len(raw) != 6 or not np.allclose(raw, expected_raw, rtol=0, atol=1e-12):
            raise ValueError(f"MuJoCo source waist inertia mismatch: {filename}")
        model = mujoco.MjModel.from_xml_path(str(path.resolve()))
        body = model.body("waist_yaw_link").id
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.body_iquat[body])
        rotation = rotation.reshape(3, 3)
        tensor = rotation @ np.diag(model.body_inertia[body]) @ rotation.T
        if not math.isclose(model.body_mass[body], WAIST_MASS_KG, rel_tol=0, abs_tol=1e-12):
            raise ValueError(f"Compiled MuJoCo waist mass mismatch: {filename}")
        if not np.allclose(model.body_ipos[body], WAIST_COM, rtol=0, atol=1e-12):
            raise ValueError(f"Compiled MuJoCo waist COM mismatch: {filename}")
        # XML values are checked exactly above; MuJoCo's principal-axis
        # eigensolver introduces approximately 2e-9 here on MuJoCo 3.8.
        if not np.allclose(tensor, expected_tensor, rtol=0, atol=1e-8):
            raise ValueError(f"Compiled MuJoCo waist inertia mismatch: {filename}")
    print(f"OK latest waist inertial URDF/compiled MJCF mass={mass}kg COM/tensor match")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", nargs="*", type=Path, help="MJCF files to load")
    args = parser.parse_args()
    paths = args.models or DEFAULT_MODELS
    expected_counts = dict(zip(DEFAULT_MODELS, (0, 37, 0, 37, 0, 37)))
    for path in paths:
        resolved = path if path.is_absolute() else ROOT / path
        validate(resolved, expected_nv=expected_counts.get(resolved))
    validate_urdf(
        ROOT / "models/sprite0825/sprite0825_sanitized_v4.urdf",
        ROOT / "models/sprite0825",
    )
    validate_urdf(V5 / "sprite0825_float.urdf", V5)
    validate_v5_contract(V5 / "sprite0825_float.urdf", V5 / "mujoco/sprite0825_v5_external_pd.xml")
    validate_urdf(LATEST_V5 / "sprite0825_float.urdf", LATEST_V5)
    validate_v5_contract(LATEST_V5 / "sprite0825_float.urdf", LATEST_V5 / "mujoco/sprite0825_v5_external_pd.xml")
    validate_waist_revision()
    validate_hashes()


if __name__ == "__main__":
    main()
