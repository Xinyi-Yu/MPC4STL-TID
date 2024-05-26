import sys
from gurobipy import *
from Parameter import *
from Print_plot import *
def Set_Prob_Robust(curr_t, input, len_stl):
    model_robust = Model("robust_problem")
    model_robust.setParam('OutputFlag', 0)
    state = []
    zr = model_robust.addVars(len_stl + 1, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="z_robust") 
    w = model_robust.addVars(len_stl, 4, lb=-Disturb_max, ub=Disturb_max, vtype=GRB.CONTINUOUS, name="w_robust") 
    add_model_constrs_robust(model_robust, input, zr, w, curr_t, len_stl)
    obj = Add_obj_robust(model_robust, zr, curr_t)
    model_robust.optimize()
    if (model_robust.status == GRB.Status.OPTIMAL):
        disturb = model_robust.getAttr('x', w)
        zz = model_robust.getAttr('x', zr)
        for i in range(len_stl+1):
            state.append([zz[i, 0], zz[i, 1], zz[i, 2], zz[i, 3]])
        #printSolution(state)
        print(" it is feasible")
        return disturb, obj.X
    else:
        print(" ++++++++++++++++no worst feasible disturb")
        sys.exit(0)
                    
    
def add_model_constrs_robust(model_robust, input, zr, w, curr_t, len_stl):
    # Known system state at the current time
    for i in range(curr_t+1):
        model_robust.addConstr((zr[i, 0] == optimal_state_sequence[i][0]), 'Known_state01')
        model_robust.addConstr((zr[i, 1] == optimal_state_sequence[i][1]), 'Known_state02')
        model_robust.addConstr((zr[i, 2] == optimal_state_sequence[i][2]), 'Known_state03')
        model_robust.addConstr((zr[i, 3] == optimal_state_sequence[i][3]), 'Known_state04')
    # system model_robust
    model_robust.addConstrs((zr[i + 1, 0] == zr[i, 0] + 0.5*zr[i, 2] + 0.125*input[i][0] + w[i, 0]for i in range(curr_t, len_stl)), 'Dynamic1')
    model_robust.addConstrs((zr[i + 1, 1] == zr[i, 1] + 0.5*zr[i, 3] + 0.125*input[i][1] + w[i, 1] for i in range(curr_t, len_stl)), 'Dynamic2')
    model_robust.addConstrs((zr[i + 1, 2] == zr[i, 2] + 0.5*input[i][0] + w[i, 2] for i in range(curr_t, len_stl)), 'Dynamic3')
    model_robust.addConstrs((zr[i + 1, 3] == zr[i, 3] + 0.5*input[i][1] + w[i, 3] for i in range(curr_t, len_stl)), 'Dynamic4')
    # physical constraints - workspace
    model_robust.addConstrs((xmap[0] <= zr[i, 0] for i in range(curr_t, len_stl + 1)), 'Workspace1')
    model_robust.addConstrs((xmap[1] >= zr[i, 0] for i in range(curr_t, len_stl + 1)), 'Workspace2')
    model_robust.addConstrs((ymap[0] <= zr[i, 1] for i in range(curr_t, len_stl + 1)), 'Workspace3')
    model_robust.addConstrs((ymap[1] >= zr[i, 1] for i in range(curr_t, len_stl + 1)), 'Workspace4')
    # physical constraints - speed
    model_robust.addConstrs((-vmax <= zr[i, 2] for i in range(curr_t, len_stl + 1)), 'Speed1')
    model_robust.addConstrs((zr[i, 2] <= vmax for i in range(curr_t, len_stl + 1)), 'Speed2')
    model_robust.addConstrs((-vmax <= zr[i, 3] for i in range(curr_t, len_stl + 1)), 'Speed3')
    model_robust.addConstrs((zr[i, 3] <= vmax for i in range(curr_t, len_stl + 1)), 'Speed4')

def Add_obj_robust(model_robust, zr, curr_t):
    if curr_t <= G1_tmax:
        temp_G1 = model_robust.addVars(G1_t + 1, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="temp")
        rho_G1_temp = model_robust.addVars(G1_t + 1, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_G1_temp")
        rho_G1 = model_robust.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_G1")
        for i in range(G1_t + 1):
            model_robust.addConstr((temp_G1[i, 0] == zr[G1_tmin + i, 0] - xA3[0]), "temp1")
            model_robust.addConstr((temp_G1[i, 1] == xA3[1] - zr[G1_tmin + i, 0]), "temp1")
            model_robust.addConstr((temp_G1[i, 2] == zr[G1_tmin + i, 1] - yA3[0]), "temp1")
            model_robust.addConstr((temp_G1[i, 3] == yA3[1] - zr[G1_tmin + i, 1]), "temp1")
        for i in range(G1_t + 1):
            model_robust.addGenConstrMin(rho_G1_temp[i, 0], [temp_G1[i, j] for j in range(4)], name="minconstr")
        #model_robust.addGenConstrMax(rho_G1, [rho_G1_temp[i, 0] for i in range(G1_t + 1)], name="minconstr")
        model_robust.addGenConstrMin(rho_G1, [rho_G1_temp[i, 0] for i in range(G1_t + 1)], name="minconstr2")
    if curr_t <= F0_t:
        # F[0,6]A1
        temp_F0 = model_robust.addVars(F0_t + 1, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="temp")
        rho_F0_temp = model_robust.addVars(F0_t + 1, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_F0_temp")
        rho_F0 = model_robust.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_F0")
        for i in range(F0_t + 1):
            model_robust.addConstr((temp_F0[i, 0] == zr[i, 0] - xA1[0]), "temp1")
            model_robust.addConstr((temp_F0[i, 1] == xA1[1] - zr[i, 0]), "temp1")
            model_robust.addConstr((temp_F0[i, 2] == zr[i, 1] - yA1[0]), "temp1")
            model_robust.addConstr((temp_F0[i, 3] == yA1[1] - zr[i, 1]), "temp1")
        for i in range(F0_t + 1):
            model_robust.addGenConstrMin(rho_F0_temp[i, 0], [temp_F0[i, j] for j in range(4)], name="minconstr1")
        model_robust.addGenConstrMax(rho_F0, [rho_F0_temp[i, 0] for i in range(F0_t + 1)], name="minconstr2")

    temp_F1 = model_robust.addVars(F1_t + 1, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="temp")
    rho_F1_temp = model_robust.addVars(F1_t + 1, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_F1_temp")
    rho_F1 = model_robust.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_F1")
    for i in range(F1_t + 1):
        model_robust.addConstr((temp_F1[i, 0] == zr[F1_tmin  + i, 0] - xA2[0]), "temp1")
        model_robust.addConstr((temp_F1[i, 1] == xA2[1] - zr[F1_tmin + i, 0]), "temp1")
        model_robust.addConstr((temp_F1[i, 2] == zr[F1_tmin + i, 1] - yA2[0]), "temp1")
        model_robust.addConstr((temp_F1[i, 3] == yA2[1] - zr[F1_tmin + i, 1]), "temp1")
    for i in range(F1_t + 1):
        model_robust.addGenConstrMin(rho_F1_temp[i, 0], [temp_F1[i, j] for j in range(4)], name="minconstr")
    model_robust.addGenConstrMax(rho_F1, [rho_F1_temp[i, 0] for i in range(F1_t + 1)], name="maxconstr")

    obj = model_robust.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho")
    if curr_t <= F0_t:
        model_robust.addGenConstrMin(obj, [rho_F0, rho_G1, rho_F1], name="minconstr")
    else:
        if stage == 1 &  curr_t <= G1_tmax:
            model_robust.addGenConstrMin(obj, [rho_F1, rho_G1], name="minconstr")
        else:
            model_robust.addConstr(obj == rho_F1,  name="minconstr")
    model_robust.setObjective(obj, GRB.MINIMIZE)
    return obj
