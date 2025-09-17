import json
import os

# Ensure the 'autos' folder exists
os.makedirs("autos", exist_ok=True)


data_names = [
    "S1", "S2", "S3", "TR", "TL", "MR", "ML", "BR", "BL", "Processor", "FT", "FB"
]

commands = [
    "AutoAimLeft", "AutoAimRight", "L1", "L2", "L3", "L4", "Outtake", "Intake", "Wait", "AutoAimOn", "AutoAimOff", "AutoAimMiddle"
]

def generate_auto_file(start, middle, end, command_sequence, suffix, wait):
    # Replace names for file naming purposes
    suffix = suffix.replace("AutoAimLeft", "Left").replace("AutoAimRight", "Right")
    if wait:
        file_name = f"autos/{start}-{middle}-{end}-{suffix}-Wait.auto"
    else:
        file_name = f"autos/{start}-{middle}-{end}-{suffix}.auto"
    file_content = {
        "version": "2025.0",
        "command": {
            "type": "sequential",
            "data": {
                "commands": command_sequence
            }
        },
        "resetOdom": True,
        "folder": None,
        "choreoAuto": False
    }
    
    with open(file_name, "w") as f:
        json.dump(file_content, f, indent=2)
    # print(f"Generated: {file_name}")
    print(f"Successfully Hacked: {file_name}")

def generate_combinations():
    wait = False
    starts_input = "S1, S2, S3".split(",")
    starts = [s.strip() for s in starts_input if s.strip() in data_names]
    
    middles_input = "TR, TL, ML, MR, BL, BR, FT, FB".split(",")
    middles = [m.strip() for m in middles_input if m.strip() in data_names]
    
    end_input = "TR, TL, ML, MR, BL, BR, FT, FB, Processor".split(",")
    ends = [e.strip() for e in end_input if e.strip() in data_names]
    
    if not starts or not middles or not ends:
        print("ur a bum")
        return
    for _ in range(0, 2):
        for start in starts:
            for middle in middles:
                for end in ends:
                    if middle == end:
                        continue
                    for autoaim in ["AutoAimLeft", "AutoAimRight"]:
                        for l_command in ["L1", "L2", "L3", "L4"]:
                            for algae in ["L2", "L3"]:
                                if algae != l_command:
                                    if wait:
                                        command_sequence = [
                                            {
                                                "type": "named",
                                                "data": {
                                                    "name": "ResetAll"
                                                }
                                            },
                                            {
                                                "type": "wait",
                                                "data": {
                                                "waitTime": 1.0
                                                }
                                            },
                                            {"type": "path", "data": {"pathName": f"{start}-{middle}"}},
                                            {
                                                "type": "named",
                                                "data": {
                                                    "name": autoaim
                                                }
                                            },
                                            {
                                                "type": "deadline",
                                                "data": {
                                                    "commands": [
                                                        {"type": "named", "data": {"name": "AutoAimOn"}},
                                                    ]
                                                }
                                            },
                                            {"type": "named", "data": {"name": "AutoAimOff"}},
                                            {"type": "named", "data": {"name": l_command}},
                                            {
                                                "type": "wait",
                                                "data": {
                                                "waitTime": 1.0
                                                }
                                            },
                                            {"type": "named", "data": {"name": "Outtake"}},
                                            {
                                                "type": "wait",
                                                "data": {
                                                "waitTime": .5
                                                }
                                            },
                                            {"type": "named", "data": {"name": algae}},
                                            {
                                                "type": "wait",
                                                "data": {
                                                "waitTime": 1.0
                                                }
                                            },
                                            {"type": "named", "data": {"name": "Intake"}},
                                            {"type": "path", "data": {"pathName": f"{middle}-{end}"}}
                                        
                                        ]
                                    else:
                                        command_sequence = [
                                            {
                                                "type": "named",
                                                "data": {
                                                    "name": "ResetAll"
                                                }
                                            },
                                            {"type": "path", "data": {"pathName": f"{start}-{middle}"}},
                                            {
                                                "type": "named",
                                                "data": {
                                                    "name": autoaim
                                                }
                                            },
                                            {
                                                "type": "deadline",
                                                "data": {
                                                    "commands": [
                                                        {"type": "named", "data": {"name": "AutoAimOn"}},
                                                    ]
                                                }
                                            },
                                            {"type": "named", "data": {"name": "AutoAimOff"}},
                                            {"type": "named", "data": {"name": l_command}},
                                               {
                                                "type": "wait",
                                                "data": {
                                                "waitTime": 1.0
                                                }
                                            },
                                            {"type": "named", "data": {"name": "Outtake"}},
                                            {
                                                "type": "wait",
                                                "data": {
                                                "waitTime": .5
                                                }
                                            },
                                            {"type": "named", "data": {"name": algae}},
                                            {
                                                "type": "wait",
                                                "data": {
                                                "waitTime": 1.0
                                                }
                                            },
                                            {"type": "named", "data": {"name": "Intake"}},
                                            {"type": "path", "data": {"pathName": f"{middle}-{end}"}}
                                        ]
                                    if end == "Processor":
                                        command_sequence.append({"type": "named", "data": {"name": "Outtake"}})
                                    suffix = f"{autoaim}-{l_command}"
                                    generate_auto_file(start, middle, end, command_sequence, suffix, wait)
        wait = True

generate_combinations()