"""Derive a versioned waist-inertia update without changing historical assets."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
BASE_NAME = "sprite0825_v5_4340_shoulders"
REVISION_NAME = "sprite0825_v5_4340_shoulders_waist20261105"
BASE_URDF_SHA256 = "6c753d563b54103e4278c27a11a22e0f4f1076e4156f0873cb2325f1701a921b"
WAIST_MASS_KG = 3.7391
WAIST_COM = (0.02454, 6.3252e-05, 0.11823)
WAIST_INERTIA = {
    "ixx": 0.006815, "iyy": 0.004205, "izz": 0.0040106,
    "ixy": 3.8252e-07, "ixz": 5.083e-05, "iyz": -1.0844e-06,
}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numbers(values) -> str:
    return " ".join(format(v, ".12g") for v in values)


def write_xml(tree: ET.ElementTree, path: Path) -> None:
    ET.indent(tree, space="  ")
    path.write_bytes(ET.tostring(tree.getroot(), encoding="utf-8", xml_declaration=True) + b"\n")


def build(destination: Path) -> None:
    base = ROOT / "models" / BASE_NAME
    if sha(base / "sprite0825_float.urdf") != BASE_URDF_SHA256:
        raise ValueError("Historical v5 URDF changed; review the source before deriving a revision")
    if destination.exists():
        raise FileExistsError(f"Refuse to overwrite an asset revision: {destination}")
    shutil.copytree(base, destination)
    tree = ET.parse(destination / "sprite0825_float.urdf")
    tree.getroot().set("name", REVISION_NAME)
    inertial = tree.find("link[@name='waist_yaw_link']/inertial")
    inertial.find("origin").attrib = {"xyz": numbers(WAIST_COM), "rpy": "0 0 0"}
    inertial.find("mass").set("value", str(WAIST_MASS_KG))
    inertial.find("inertia").attrib = {key: format(value, ".12g") for key, value in WAIST_INERTIA.items()}
    write_xml(tree, destination / "sprite0825_float.urdf")
    for filename in ("sprite0825_v5_external_pd.xml", "sprite0825_v5_visual_locked.xml"):
        path = destination / "mujoco" / filename
        tree = ET.parse(path)
        tree.getroot().set("model", REVISION_NAME + ("_locked" if "locked" in filename else ""))
        inertial = tree.find(".//body[@name='waist_yaw_link']/inertial")
        inertial.attrib = {
            "pos": numbers(WAIST_COM), "mass": str(WAIST_MASS_KG),
            "fullinertia": numbers(WAIST_INERTIA[key] for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
        }
        write_xml(tree, path)
    for filename in ("scene.xml", "scene_external_pd.xml"):
        path = destination / "mujoco" / filename
        tree = ET.parse(path)
        tree.getroot().set("model", REVISION_NAME + "_" + path.stem)
        write_xml(tree, path)
    files = {p.relative_to(destination).as_posix(): sha(p)
             for p in sorted(destination.rglob("*")) if p.is_file() and p.name != "manifest.json"}
    manifest = dict(
        schema="open_sprite_public_robot_asset_v2", asset_revision=REVISION_NAME,
        base_asset=BASE_NAME, base_publication_commit="afc882b",
        base_manifest_sha256=sha(base / "manifest.json"), base_urdf_sha256=BASE_URDF_SHA256,
        source_repository="https://github.com/BeijingDynamics/open_sprite",
        parameter_source="Owner-supplied waist_yaw_link inertial block, incorporated in this revision on 2026-11-05",
        urdf="sprite0825_float.urdf", urdf_sha256=sha(destination / "sprite0825_float.urdf"),
        revolute_joint_count=31, mesh_count=len(list((destination / "meshes").glob("*.STL"))),
        changed_link="waist_yaw_link", mass_kg=WAIST_MASS_KG, com_m=WAIST_COM,
        inertia_kg_m2=WAIST_INERTIA, inertia_frame="link-aligned frame at COM; URDF inertial rpy=0 0 0",
        unchanged="All other link inertials, joint axes/origins/ranges, effort/velocity metadata, armature, collision geometry and mesh bytes",
        policy_compatibility="New inertial revision, not a checkpoint qualification. No policy or runtime configuration is changed or supplied.",
        training_asset_scope="Waist inertia matches the owner-supplied training update. This is not a byte-identical copy of any experiment's collision/actuator adapter.",
        generated_by="scripts/build_waist_inertia_revision.py", files_sha256=files,
    )
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Rebuild in a temporary directory and compare every packaged byte")
    args = parser.parse_args()
    target = ROOT / "models" / REVISION_NAME
    if args.check:
        with tempfile.TemporaryDirectory() as temporary:
            expected = Path(temporary) / REVISION_NAME
            build(expected)
            files = {p.relative_to(expected) for p in expected.rglob("*") if p.is_file()}
            actual = {p.relative_to(target) for p in target.rglob("*") if p.is_file()}
            if files != actual or any(sha(expected / name) != sha(target / name) for name in files):
                raise ValueError("Published waist revision differs from deterministic rebuild")
        print("OK deterministic waist revision rebuild")
        return
    build(target)
    entries = [f"{sha(p)}  {p.relative_to(ROOT).as_posix()}" for p in sorted((ROOT / "models").rglob("*")) if p.is_file()]
    (ROOT / "ASSET_SHA256SUMS").write_text("\n".join(entries) + "\n", encoding="ascii", newline="\n")
    print(f"BUILT {target.relative_to(ROOT)} waist_mass_kg={WAIST_MASS_KG}")


if __name__ == "__main__":
    main()
