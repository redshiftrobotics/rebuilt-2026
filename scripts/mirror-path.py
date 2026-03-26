import sys
import json
import pathlib

FIELD_SIZE_Y = 8.07

if len(sys.argv) < 3:
	raise SystemExit("Usage: python[3] " + sys.argv[0] + " <path of input file> <path of new output file>")

in_path = pathlib.Path(sys.argv[1])
if not in_path.exists():
	raise SystemExit("Specified auto path does not exist!")

out_path = pathlib.Path(sys.argv[2])

def mirror(inPath):
  file = open(inPath)
  
  doc = json.load(file)

  for waypoint in doc["waypoints"]:
    waypoint["linkedName"] = None
    waypoint["anchor"]["y"] = FIELD_SIZE_Y - waypoint["anchor"]["y"]
    if not waypoint["prevControl"] is None:
      waypoint["prevControl"]["y"] = FIELD_SIZE_Y - waypoint["prevControl"]["y"]
    if not waypoint["nextControl"] is None:
      waypoint["nextControl"]["y"] = FIELD_SIZE_Y - waypoint["nextControl"]["y"]
  
  for rotationTarget in doc["rotationTargets"]:
    rotationTarget["rotationDegrees"] = -rotationTarget["rotationDegrees"]

  doc["goalEndState"]["rotation"] *= -1

  doc["idealStartingState"]["rotation"] *= -1
  
  return doc 
  
outfile = open(out_path, "w")
# try:
  # outfile = open(out_path, "x")
# except:
#   raise SystemExit("Specified out path already exists")

json.dump(mirror(in_path), outfile, indent=2)