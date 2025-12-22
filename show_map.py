import json
import sys

filename = sys.argv[1] if len(sys.argv) > 1 else "map_jetank_60x60.json"

with open(filename, "r") as f:
    data = json.load(f)

print("width:", data["width"], "height:", data["height"])
print("resolution:", data["resolution"], "m per cell")

print("\nMap (top row first):")

for row in reversed(data["cells"]):
    print(row)
