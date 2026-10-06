# driver.py
import subprocess, sys, os,json

SCRIPT = os.path.join(os.path.dirname(__file__), "sts_gui.py")

#Remove?
var_num = 18
exp_id = "exp1"

subject_id = input("subject id?: sub<>: ")

print(f"Starting {exp_id} - {subject_id}")

for var in range(var_num):
    # var = 5
    print("========================================================\n")
    chair_height = input("chair height?: ")
    foot_position = input("foot position?: ")
    momentum_condition = input("momentum condition?: ")
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
                        "xsens":{"on":False,
                                 "ip":"0.0.0.0",
                                 "port":9764},
                        "forceplate":{"on":False,
                                 "ip":"127.0.0.1",
                                 "port":2048,
                                 "num_plates":2},
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
