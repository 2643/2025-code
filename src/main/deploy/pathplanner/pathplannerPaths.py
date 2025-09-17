import re
import json
import os
from itertools import product

def parse_input(data: str):
    result = {}
    entries = data.split("; ")
    
    for entry in entries:
        match = re.match(r"([\w\s]+):X:([-\d.]+),Y:([-\d.]+)", entry)
        if match:
            name, x, y = match.groups()
            result[name.strip()] = (float(x), float(y))
        else:
            print(f"Skipping invalid entry: {entry}")
    
    return result

def get_rotation(name: str):
    # Define rotation values for each waypoint
    rotations = {
        "S1": 180,
        "S2": 180,
        "S3": 180,
        "Algae T": 180,
        "Algae M": 180,
        "Algae B": 180,
        "MR": 180,  # Updated to 180
        "ML": 0,    # Updated to 0
        "TL": -60,
        "TR": -120,
        "BR": 120,
        "BL": 60,
        "Processor": -90,
        "FT": 130,
        "FB": -130
    }
    return rotations.get(name, 0)  # Default to 0 if name not found

def generate_combinations(parsed_data):
    start_input = input("Enter start elements separated by commas: ").strip().split(", ")
    end_input = input("Enter end elements separated by commas: ").strip().split(", ")
    
    # Filter valid start and end points
    starts = [s for s in start_input if s in parsed_data]
    ends = [e for e in end_input if e in parsed_data]
    
    if not starts:
        print("No valid start elements found in the data.")
        return
    if not ends:
        print("No valid end elements found in the data.")
        return
    
    # Create the 'paths' folder if it doesn't exist
    if not os.path.exists("paths"):
        os.makedirs("paths")
        print("Created 'paths' folder.")
    
    for start, end in product(starts, ends):
        # Get rotations for start and end waypoints
        start_rotation = get_rotation(start)
        end_rotation = get_rotation(end)
        
        waypoints = [
            {
                "anchor": {"x": parsed_data[start][0], "y": parsed_data[start][1]},
                "prevControl": None,
                "nextControl": {"x": parsed_data[end][0] +1, "y": parsed_data[end][1]},
                "isLocked": False,
                "linkedName": start
            },
            {
                "anchor": {"x": parsed_data[end][0], "y": parsed_data[end][1]},
                "prevControl": {"x": parsed_data[start][0] +1, "y": parsed_data[start][1]},
                "nextControl": None,
                "isLocked": False,
                "linkedName": end
            }
        ]
        
        output_data = {
            "version": "2025.0",
            "waypoints": waypoints,
            "rotationTargets": [],
            "constraintZones": [],
            "pointTowardsZones": [],
            "eventMarkers": [],
            "globalConstraints": {
                "maxVelocity": 3.0,
                "maxAcceleration": 3.0,
                "maxAngularVelocity": 540.0,
                "maxAngularAcceleration": 720.0,
                "nominalVoltage": 12.0,
                "unlimited": False
            },
            "goalEndState": {
                "velocity": 0,
                "rotation": end_rotation
            },
            "reversed": False,
            "idealStartingState": {
                "velocity": 0,
                "rotation": start_rotation
            },
            "useDefaultConstraints": True
        }
        
        filename = f"paths/{start.replace(' ', '_')}-{end.replace(' ', '_')}.path"
        with open(filename, "w") as file:
            json.dump(output_data, file, indent=2)
        
        print(f"Combination saved to {filename}")

# Example usage
data = (
    "S1:X:8.103688524590162,Y:6.141; "
    "S2:X:8.103688524590162,Y:4.091; "
    "S3:X:8.103688524590162,Y:1.9; "
    "TR:X:5.5383196721311485,Y:5.63734631147541; "
    "TL:X:3.500409836065574,Y:5.63734631147541; "
    "MR:X:6.149692622950819,Y:4.019006147540983; "
    "ML:X:2.6372950819672134,Y:4.019006147540983; "
    "BR:X:5.5383196721311485,Y:2.5684938524590155; "
    "BL:X:3.500409836065574,Y:2.436629098360655; "
    "Algae T:X:2.133811475409836,Y:5.889088114754099; "
    "Algae M:X:1.9141676469162732,Y:4.019006147540983; "
    "Algae B:X:1.9141676469162732,Y:2.1848872950819667; "
    "Processor:X:6.017827868852459,Y:0.8063012295081955; "
    "FT:X:1.65400000,Y:6.82400000; "
    "FB:X:1.33100000,Y:1.29800000; "
)
parsed_data = parse_input(data)
print("Parsed Data:", parsed_data)

generate_combinations(parsed_data)