import cv2
import yaml
import sys
from pathlib import Path

MAP_PGM = Path(__file__).parent.parent / "src/view_robot/maps/X5_19032026.pgm"
MAP_YAML = Path(__file__).parent.parent / "src/view_robot/maps/X5_19032026.yaml"
OUTPUT = Path(__file__).parent.parent / "src/view_robot/maps/areas.yaml"

def load_map_meta():
    with open(MAP_YAML) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(str(MAP_PGM), cv2.IMREAD_GRAYSCALE)
    return meta["resolution"], meta["origin"][0], meta["origin"][1], img.shape[0]

def pixel_to_world(px, py, resolution, ox, oy, map_height):
    return round(ox + px * resolution, 4), round(oy + (map_height - py) * resolution, 4)

def main():
    resolution, ox, oy, map_height = load_map_meta()
    areas = []

    print(f"Map: {MAP_PGM.name}  resolution={resolution}  origin=({ox}, {oy})  height={map_height}px")
    print("Enter areas. Type area name then 4 corners as 'px py'. Empty name to finish.\n")

    while True:
        name = input("Area name (or Enter to finish): ").strip()
        if not name:
            break
        corners = []
        for i in range(4):
            raw = input(f"  Corner {i+1} (px py): ").strip().split()
            px, py = int(raw[0]), int(raw[1])
            wx, wy = pixel_to_world(px, py, resolution, ox, oy, map_height)
            corners.append([wx, wy])
            print(f"    → world ({wx}, {wy})")
        areas.append({"name": name, "corners": corners})

    with open(OUTPUT, "w") as f:
        yaml.dump({"areas": areas}, f, default_flow_style=False, allow_unicode=True)

    print(f"\nSaved to {OUTPUT}")

if __name__ == "__main__":
    main()
