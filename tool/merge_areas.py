import json
import yaml
from pathlib import Path

WAYPOINTS = Path(__file__).parent.parent / "robot_ui/waypoints.json"
AREAS_YAML = Path(__file__).parent.parent / "src/view_robot/maps/areas.yaml"

def point_in_polygon(x, y, corners):
    n = len(corners)
    inside = False
    px, py = x, y
    j = n - 1
    for i in range(n):
        xi, yi = corners[i]
        xj, yj = corners[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

def find_area(x, y, areas):
    for area in areas:
        if point_in_polygon(x, y, area["corners"]):
            return area["name"]
    return None

def main():
    with open(WAYPOINTS) as f:
        data = json.load(f)

    with open(AREAS_YAML) as f:
        areas_data = yaml.safe_load(f)
    areas = areas_data["areas"]

    waypoints = {k: v for k, v in data.items() if k != "areas"}
    for slot, wp in waypoints.items():
        wp["area"] = find_area(wp["x"], wp["y"], areas)

    data = {**waypoints, "areas": areas}

    with open(WAYPOINTS, "w") as f:
        json.dump(data, f, indent=4)

    print(f"Updated {WAYPOINTS}")
    for slot, wp in waypoints.items():
        print(f"  Slot {slot}: area = {wp['area']}")

if __name__ == "__main__":
    main()
