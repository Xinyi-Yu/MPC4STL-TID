from gurobipy import *
from Parameter import *
from Mpc import *
from pro_set import *
import time

time_start = time.time()

for stage in range(len(spl_t)-1):
    len_stl = spl_t[stage+1] - spl_t[stage]
    for j in range(len_stl):
        print("-----------------current time is", spl_t[stage] + j)
        # Get an array of random disturb to calculate an input set
        disturb = generate_disturb(len_stl)
        mpc = MPC(len_stl, spl_t[stage] + j, stage)     # build an mpc model
        mpc.set(disturb)

        disturb_var, obj = mpc.check()    # find the array of disturbs which results the worst robustness
        disturb_vec = []
        for i in range(len_stl):
            disturb_vec.append([disturb_var[i, 0], disturb_var[i, 1], disturb_var[i, 2], disturb_var[i, 3]])
        # tupleditct to list transform

        while (obj < 0):    # indicating that this set of inputs can not resist all disturbs
            mpc.set(disturb_vec)      # add this disturb to intrigue a better set of inputs
            disturb_var, obj = mpc.check()

            disturb_vec = []
            for i in range(len_stl):
                disturb_vec.append([disturb_var[i, 0], disturb_var[i, 1], disturb_var[i, 2], disturb_var[i, 3]])
        # if obj>0, means that this set of inputs can resist all disturbs
        mpc.apply()
        if spl_t[stage] + j == 1:
            time_end1 = time.time()
            print("Time cost 1:", time_end1 - time_start)
        if spl_t[stage] + j == 2:
            time_end2 = time.time()
            print("Time cost 2:", time_end2 - time_end1)

        if (spl_t[stage] + j == Stl_Hrizon - 1):
            time_end = time.time()
            print("Time cost 1:", time_end1 - time_start)
            print("Time cost 2:", time_end2 - time_end1)
            print("Time cost total:", time_end - time_start)
            mpc.print()
