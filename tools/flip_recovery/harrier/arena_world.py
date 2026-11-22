"""Copy the Harrier arena without its pre-spawned aircraft."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("source", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
tree = ET.parse(args.source)
world = tree.getroot().find("world")
aircraft = [part for part in world.findall("include")
            if part.findtext("name") == "Drone-Harrier"]
if len(aircraft) != 1:
    parser.error("expected one Drone-Harrier in the arena")
world.remove(aircraft[0])
tree.write(args.output, encoding="utf-8", xml_declaration=True)
