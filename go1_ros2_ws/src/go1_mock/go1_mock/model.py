"""The joint contract of the pinned Phanthy Motus Go1 URDF."""
from dataclasses import dataclass
from math import cos, isfinite
from pathlib import Path
from xml.etree import ElementTree as ET

LEGS = ("FR", "FL", "RR", "RL")
SEGMENTS = ("hip", "thigh", "calf")
JOINT_NAMES = tuple(f"{leg}_{segment}_joint" for leg in LEGS for segment in SEGMENTS)
STAND = (0.0, 0.75, -1.50) * 4


@dataclass(frozen=True)
class Go1Model:
    joint_limits: tuple[tuple[float, float], ...]
    standing_height: float

    @classmethod
    def load(cls, path: Path):
        root = ET.parse(path).getroot()
        joints = root.findall("joint")
        movable = [joint for joint in joints if joint.attrib["type"] != "fixed"]
        if tuple(joint.attrib["name"] for joint in movable) != JOINT_NAMES:
            raise ValueError("URDF does not have the expected Go1 joint order")
        limits = tuple(
            (float(joint.find("limit").attrib["lower"]), float(joint.find("limit").attrib["upper"]))
            for joint in movable
        )
        for position, (lower, upper) in zip(STAND, limits):
            if not all(map(isfinite, (lower, upper))) or not lower < position < upper:
                raise ValueError("standing position must lie inside each URDF limit")

        children = {joint.find("child").attrib["link"]: joint for joint in joints}
        links = {link.attrib["name"]: link for link in root.findall("link")}
        if set(links) - children.keys() != {"trunk"} or len(children) != len(joints):
            raise ValueError("invalid Go1 URDF root or duplicate child link")
        heights = []
        for index, leg in enumerate(LEGS):
            parent = "trunk"
            for segment in (*SEGMENTS, "foot"):
                child = f"{leg}_{segment}"
                joint = children[child]
                if joint.find("parent").attrib["link"] != parent:
                    raise ValueError(f"unexpected parent of {child}")
                parent = child
            thigh, calf = STAND[3 * index + 1:3 * index + 3]
            thigh_length = -float(children[f"{leg}_calf"].find("origin").attrib["xyz"].split()[2])
            calf_length = -float(children[f"{leg}_foot"].find("origin").attrib["xyz"].split()[2])
            radius = float(links[f"{leg}_foot"].find("visual/geometry/sphere").attrib["radius"])
            heights.append(thigh_length * cos(thigh) + calf_length * cos(thigh + calf) + radius)
        return cls(limits, max(heights))
