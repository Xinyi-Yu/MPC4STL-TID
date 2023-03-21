from gurobipy import *
from Parameter import *
from Print_plot import *
from Mpc import *
from pro_set import *
import json
import pandas
import time
for k in range(reps):  # in Parameter.py you can decide how many reps you want to excecute the project
    k = k + 1
    time_start = time.time()
    for k in range(0, Stl_Hrizon):
        mpc = MPC(Stl_Hrizon, k) # build an mpc model
        # Get an array of random disturb to calculate an input set
        disturb = generate_disturb(Stl_Hrizon)
        cost = mpc.set(disturb)

        disturb_var, obj = mpc.check()
        disturb_vec = []
        for i in range(Stl_Hrizon):
            disturb_vec.append([disturb_var[i, 0], disturb_var[i, 1], disturb_var[i, 2], disturb_var[i, 3]])
            # tupleditct to list transform

        while (obj < 0): # indicating that this set of inputs can not resist all disturbs
            cost = mpc.set(disturb_vec)  # add this disturb to intrigue a better set of inputs
            disturb_var, obj = mpc.check()
            disturb_vec = []
            for i in range(Stl_Hrizon):
                disturb_vec.append([disturb_var[i, 0], disturb_var[i, 1], disturb_var[i, 2], disturb_var[i, 3]])
        mpc.apply()
        if k == 1:
            time_end1 = time.time()
        if k == 2:
            time_end2 = time.time()
        if k == Stl_Hrizon - 1:
            time_end = time.time()
            print("Time cost 1:", time_end1 - time_start)
            print("Time cost 2:", time_end2 - time_end1)
            print("Time cost:", time_end - time_start)
            print("cost is ", cost)
            file_name = 'data.json'
            test_dict = {
                    'time1': time_end1 - time_start,
                    'time2': time_end2 - time_end1,
                    'total': time_end - time_start,
                    'cost ': cost
            }
            json_str = json.dumps(test_dict)
            with open(file_name, 'a')as json_file:
                json_file.write(json_str)
                json_file.write('\n')
mpc.print()
df = pandas.read_json("data.json", lines=True)
df.to_excel("datasets.xlsx")
