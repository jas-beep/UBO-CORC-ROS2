# driver.py
import subprocess, sys, os,json

SCRIPT = os.path.join(os.path.dirname(__file__), "sts_gui.py")

exp_id = "exp1"

# Fixed condition order: chair height -> foot position -> momentum
CHAIR_HEIGHTS = ["KneeH", "Low"]
FOOT_POSITIONS = ["Normal", "Back", "Forward"]
MOMENTUM_CONDITIONS = ["N", "QS", "MT"]
CONDITIONS = [(ch, fp, mc) for ch in CHAIR_HEIGHTS for fp in FOOT_POSITIONS for mc in MOMENTUM_CONDITIONS]
var_num = len(CONDITIONS)

subject_id = input("subject id?: sub<>: ")

print(f"Starting {exp_id} - {subject_id}")
print("\nTrial order:")
for i, (ch, fp, mc) in enumerate(CONDITIONS):
    print(f"  var_{i+1:<2}  {ch:<6} {fp:<8} {mc}")

start_var = input(f"\nStart from var? (1-{var_num}, enter for 1): ").strip()
start_var = int(start_var) if start_var.isdigit() and 1 <= int(start_var) <= var_num else 1

for var in range(start_var - 1, var_num):
    print("========================================================\n")
    chair_height, foot_position, momentum_condition = CONDITIONS[var]
    print(f"var_{var+1}: chair height = {chair_height}, foot position = {foot_position}, momentum = {momentum_condition}")
    go_on = input(f"Start var_{var+1}?: y/n: ")
    while (go_on != "y" and go_on != "n"):
        go_on = input("Continue?: Reenter y/n: ")
    if go_on == "y":
        p = "Y"
        pass
    elif go_on == "n":
        p = "N"
        break
    print("========================================================\n")
    argv ={
           "init_flags":{"corc":{"on":False,
                                 "ip":"127.0.0.1",
                                 "port":2048},
                        "xsens":{"on":True,
                                 "ip":"0.0.0.0",
                                 "port":9764},
                        "forceplate":{"on":True,
                                 "ip":"127.0.0.1",
                                 "port":2048,
                                 "num_plates":3},
                         "gui":{"on":True,
                                "freq":30,
                                "3d":False,
                                "vel":True},
                         "log":{"on":True}
                         },
            "session_data":{
                "exp_id": exp_id,
                "subject_id":subject_id,
                "take_num":0,
                "var_num":var+1,
                "chair_height":chair_height,
                "foot_position":foot_position,
                "momentum_condition":momentum_condition,}
           }

    argv = json.dumps(argv)

    print(f"=== var_{var+1} started ===")
    rc = subprocess.call(
        [sys.executable, SCRIPT,str(argv)],
        env=os.environ,
    )
    if rc != 0:
        print(f" var_{var+1} exited with {rc}, stopping.")
        break
    import time
    time.sleep(1)
    print(f"\n=== var_{var+1} ended ===\n")

    if p == "N":
        break
print("\nExperiment Ended")
print("\nPlease Ask How They Would Interact")
