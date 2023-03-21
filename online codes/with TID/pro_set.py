from Parameter import *
from gurobipy import *
import random


#generate disturb sequences
def generate_disturb(length):
    disturb_sequence = []
    for i in range(length):
        disturb_x = Disturb_max * 0.1 * random.randint(-10, 10)
        disturb_y = Disturb_max * 0.1 * random.randint(-10, 10)
        disturb_vx = Disturb_max * 0.1 * random.randint(-10, 10)
        disturb_vy = Disturb_max * 0.1 * random.randint(-10, 10)
        disturb = [disturb_x, disturb_y, disturb_vx, disturb_vy]
        disturb_sequence.append(disturb)
    return disturb_sequence


def Set_Prob(curr_t, disturb_set, len_stl, stage):
    model = Model("opti_prob")
    model.setParam('OutputFlag', 0)
    len_disturb_set = len(disturb_set)
    # add variants
    z = model.addVars(len_disturb_set * (len_stl + 1), 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="z")  # system state
    u = model.addVars(len_stl, 2, lb=-umax, ub=umax, vtype=GRB.CONTINUOUS, name="u")  # control input

    add_model_constrs_with_disturb(model, z, u, curr_t, disturb_set, len_disturb_set, len_stl, stage)
    add_stl_constrs_binary(model, z, len_disturb_set, len_stl, stage)

    if stage == 0:
        add_terminal_constrs(model, z, len_disturb_set, len_stl) 
    
    # add optimize function
    obj = Add_obj(model, z, u, len_disturb_set, len_stl, stage)
    print(model)
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        state = model.getAttr('x', z)
        input = model.getAttr('x', u)
        return state, input, obj.X     # obj.X is the optimized objective function
    else:
        print('Optimization was stopped with status ' + str(model.status))
        sys.exit(0)


def add_model_constrs_with_disturb(model, z, u, curr_t, disturb_set, len_disturb_set, len_stl, stage):
    for j in range(len_disturb_set):
        model.addConstrs((z[i + j*(len_stl+1), 0] == optimal_state_sequence[i + spl_t[stage]][0] for i in range(curr_t+1 - spl_t[stage])), 'Known_state01')
        model.addConstrs((z[i + j*(len_stl+1), 1] == optimal_state_sequence[i + spl_t[stage]][1] for i in range(curr_t+1 - spl_t[stage])), 'Known_state02')
        model.addConstrs((z[i + j*(len_stl+1), 2] == optimal_state_sequence[i + spl_t[stage]][2] for i in range(curr_t+1 - spl_t[stage])), 'Known_state03') 
        model.addConstrs((z[i + j*(len_stl+1), 3] == optimal_state_sequence[i + spl_t[stage]][3] for i in range(curr_t+1 - spl_t[stage])), 'Known_state04')
        model.update()
    model.addConstrs((u[i, 0] == optimal_control_sequence[i + spl_t[stage]][0] for i in range(curr_t - spl_t[stage])), 'Known_input01')
    model.addConstrs((u[i, 1] == optimal_control_sequence[i + spl_t[stage]][1] for i in range(curr_t - spl_t[stage])), 'Known_input02')
    model.update()
    # system model
    for j in range(len_disturb_set):
        model.addConstrs((z[i + 1 + j*(len_stl+1), 0] == z[i + j*(len_stl+1), 0] + 0.5*z[i + j*(len_stl+1), 2] + 0.125*u[i, 0] + disturb_set[j][i][0] for i in range(curr_t - spl_t[stage], len_stl)), 'Dynamic_x')
        model.addConstrs((z[i + 1 + j*(len_stl+1), 1] == z[i + j*(len_stl+1), 1] + 0.5*z[i + j*(len_stl+1), 3] + 0.125*u[i, 1] + disturb_set[j][i][1] for i in range(curr_t - spl_t[stage], len_stl)), 'Dynamic_y')
        model.addConstrs((z[i + 1 + j*(len_stl+1), 2] == z[i + j*(len_stl+1), 2] + 0.5*u[i, 0] + disturb_set[j][i][2] for i in range(curr_t - spl_t[stage], len_stl)), 'Dynamic_vx')
        model.addConstrs((z[i + 1 + j*(len_stl+1), 3] == z[i + j*(len_stl+1), 3] + 0.5*u[i, 1] + disturb_set[j][i][3] for i in range(curr_t - spl_t[stage], len_stl)), 'Dynamic_vy')
        # physical constraints - workspace
        model.addConstrs((xmap[0] <= z[i + j*(len_stl+1), 0] for i in range(curr_t + 1 - spl_t[stage], len_stl + 1)), 'Workspace1')
        model.addConstrs((xmap[1] >= z[i + j*(len_stl+1), 0] for i in range(curr_t + 1- spl_t[stage], len_stl + 1)), 'Workspace2')
        model.addConstrs((ymap[0] <= z[i + j*(len_stl+1), 1] for i in range(curr_t + 1 - spl_t[stage], len_stl + 1)), 'Workspace3')
        model.addConstrs((ymap[1] >= z[i + j*(len_stl+1), 1] for i in range(curr_t + 1- spl_t[stage], len_stl + 1)), 'Workspace4')
        # physical constraints - speed constraints
        model.addConstrs((-vmax <= z[i + j*(len_stl+1), 2] for i in range(curr_t + 1 - spl_t[stage], len_stl + 1)), 'Speed1')
        model.addConstrs((z[i + j*(len_stl+1), 2] <= vmax for i in range(curr_t + 1 - spl_t[stage], len_stl + 1)), 'Speed2')
        model.addConstrs((-vmax <= z[i + j*(len_stl+1), 3] for i in range(curr_t + 1 - spl_t[stage], len_stl + 1)), 'Speed3')
        model.addConstrs((z[i + j*(len_stl+1), 3] <= vmax for i in range(curr_t + 1 - spl_t[stage], len_stl + 1)), 'Speed4')
        #physical constraints - input constraints
        model.addConstrs((-umax <= u[i, 0] for i in range(curr_t - spl_t[stage], len_stl)), 'input1')
        model.addConstrs((u[i, 0] <= umax for i in range(curr_t - spl_t[stage], len_stl)), 'input2')
        model.addConstrs((-umax <= u[i, 1] for i in range(curr_t - spl_t[stage], len_stl)), 'input3')
        model.addConstrs((u[i, 1] <= umax for i in range(curr_t - spl_t[stage], len_stl)), 'input4')
        model.update()

def add_stl_constrs_binary(model, z, len_disturb_set, len_stl, stage):
    if stage == 0:
        # -----Eventually temporal operator--------F0[0,7] A1------------
        F0 = model.addVars(len_disturb_set*(F0_t + 1), 1, lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="F0C")
        F0_4 = model.addVars(len_disturb_set*(F0_t + 1), 4, vtype = GRB.BINARY, name="F0_4")
        for j in range(len_disturb_set):
            # the model need to satisfy the constraints only in one step
            model.addConstr((sum(F0[i + j*(F0_t+1), 0] for i in range(F0_t+1)) >= 1), 'F0')
            for i in range(F0_t+1):
                model.addConstrs((F0[i + j*(F0_t+1), 0] <= F0_4[i + j*(F0_t+1), p] for p in range(4)), 'F2')
                model.addConstr((sum(F0_4[i + j*(F0_t+1), p] for p in range(4)) - 3 <= F0[i + j*(F0_t+1), 0]), 'F3')
                model.update()
            # -------(3) the value of F0_4 corresponds to system state
            model.addConstrs((z[j*(len_stl+1) + i, 0] - xA1[0] <= M * F0_4[i + j*(F0_t+1), 0] for i in range(F0_t+1)), 'F4')
            model.addConstrs((-z[j*(len_stl+1) + i, 0] + xA1[0] <= M * (1 - F0_4[i + j*(F0_t+1), 0]) for i in range(F0_t+1)), 'F5')
            model.addConstrs((-z[j*(len_stl+1) + i, 0] + xA1[1] <= M * F0_4[i + j*(F0_t+1), 1] for i in range(F0_t+1)), 'F6')
            model.addConstrs((z[j*(len_stl+1) + i, 0] - xA1[1] <= M * (1 - F0_4[i + j*(F0_t+1), 1]) for i in range(F0_t+1)), 'F7')
            model.addConstrs((z[j*(len_stl+1) + i, 1] - yA1[0] <= M * F0_4[i + j * (F0_t+1), 2] for i in range(F0_t+1)), 'F8')
            model.addConstrs((-z[j*(len_stl+1) + i, 1] + yA1[0] <= M * (1 - F0_4[i + j * (F0_t+1), 2]) for i in range(F0_t+1)), 'F9')
            model.addConstrs((-z[j*(len_stl+1) + i, 1] + yA1[1] <= M * F0_4[i + j * (F0_t+1), 3] for i in range(F0_t+1)), 'F00')
            model.addConstrs((z[j*(len_stl+1) + i, 1] - yA1[1] <= M * (1 - F0_4[i + j * (F0_t+1), 3]) for i in range(F0_t+1)), 'F01')
            model.update()
        # -----always temporal operator--------G1[12,13] A3------------
        
    if (stage == 1):
        # -----Eventually temporal operator--------F1[22,25] A2------------
        F1 = model.addVars(len_disturb_set*(F1_t+1), 1,  lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="F1C")
        F1_4 = model.addVars(len_disturb_set*(F1_t+1), 4, vtype=GRB.BINARY, name="F1_4")
        for j in range(len_disturb_set):
            # the model need to satisfy the constraints only in one step
            model.addConstr((sum(F1[i + j*(F1_t+1), 0] for i in range(F1_t+1)) >= 1), 'F11')
            for i in range(F1_t+1):
                model.addConstrs((F1[i + j*(F1_t+1), 0] <= F1_4[i + j*(F1_t+1), p] for p in range(4)), 'F12')
                model.addConstr((sum(F1_4[i + j*(F1_t+1), p] for p in range(4)) - 3 <= F1[i + j*(F1_t+1), 0]), 'F13')
            # -------(3) the value of F1_4 corresponds to system state  
            model.addConstrs((z[j*(len_stl+1) + F1_tmin - spl_t[stage] + i , 0] - xA2[0] <= M * F1_4[i + j*(F1_t+1), 0] for i in range(F1_t+1)), 'F14')
            model.addConstrs((-z[j*(len_stl+1) + F1_tmin - spl_t[stage] + i, 0] + xA2[0] <= M * (1 - F1_4[i + j*(F1_t+1), 0]) for i in range(F1_t+1)), 'F15')
            model.addConstrs((-z[j*(len_stl+1) + F1_tmin - spl_t[stage] + i, 0] + xA2[1] <= M * F1_4[i + j*(F1_t+1), 1] for i in range(F1_t+1)), 'F16')
            model.addConstrs((z[j*(len_stl+1) + F1_tmin - spl_t[stage] + i, 0] - xA2[1] <= M * (1 - F1_4[i + j*(F1_t+1), 1]) for i in range(F1_t+1)), 'F17')
            model.addConstrs((z[j*(len_stl+1) + F1_tmin - spl_t[stage] + i, 1] - yA2[0] <= M * F1_4[i + j * (F1_t+1), 2] for i in range(F1_t+1)), 'F18')
            model.addConstrs((-z[j*(len_stl+1) + F1_tmin - spl_t[stage] + i, 1] + yA2[0] <= M * (1 - F1_4[i + j * (F1_t+1), 2]) for i in range(F1_t+1)), 'F19')
            model.addConstrs((-z[j*(len_stl+1) + F1_tmin - spl_t[stage] + i, 1] + yA2[1] <= M * F1_4[i + j * (F1_t+1), 3] for i in range(F1_t+1)), 'F11')
            model.addConstrs((z[j*(len_stl+1) + F1_tmin - spl_t[stage] + i, 1] - yA2[1] <= M * (1 - F1_4[i + j * (F1_t+1), 3]) for i in range(F1_t+1)), 'F12')
        
        G1 = model.addVars(len_disturb_set * (G1_t + 1), 1,  lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="G01")
        G1_4 = model.addVars(len_disturb_set * (G1_t + 1), 4, vtype=GRB.BINARY, name="G02")
        for j in range(len_disturb_set):
            # -------at every steps between G1_min to G1_max, the model should satisfy the constraints
            model.addConstrs((G1[i + j * (G1_t + 1), 0] == 1 for i in range(G1_t + 1)), 'G03')
            for i in range(G1_t + 1):
                model.addConstrs((G1[i + j * (G1_t + 1), 0] <= G1_4[i + j * (G1_t + 1), p] for p in range(4)), 'G04')
                model.addConstr((sum(G1_4[i + j * (G1_t + 1), p] for p in range(4)) - 3 <= G1[i + j * (G1_t + 1), 0]), 'G05')
            # -------(3) the value of G1_4 corresponds to system state
            model.addConstrs((z[j * (len_stl + 1) + G1_tmin - spl_t[stage] + i, 0] - xA3[0] <= M * G1_4[i + j * (G1_t + 1), 0] for i in range(G1_t + 1)),'G06')
            model.addConstrs((-z[j * (len_stl + 1) + G1_tmin - spl_t[stage] + i, 0] + xA3[0] <= M * (1 - G1_4[i + j * (G1_t + 1), 0]) for i in range(G1_t + 1)), 'G07')
            model.addConstrs((-z[j * (len_stl + 1) + G1_tmin - spl_t[stage] + i, 0] + xA3[1] <= M * G1_4[i + j * (G1_t + 1), 1] for i in range(G1_t + 1)), 'G08')
            model.addConstrs((z[j * (len_stl + 1) + G1_tmin - spl_t[stage] + i, 0] - xA3[1] <= M * (1 - G1_4[i + j * (G1_t + 1), 1]) for i in range(G1_t + 1)), 'G09')
            model.addConstrs((z[j * (len_stl + 1) + G1_tmin - spl_t[stage] + i, 1] - yA3[0] <= M * G1_4[i + j * (G1_t + 1), 2] for i in range(G1_t + 1)), 'G010')
            model.addConstrs((-z[j * (len_stl + 1) + G1_tmin - spl_t[stage] + i, 1] + yA3[0] <= M * (1 - G1_4[i + j * (G1_t + 1), 2]) for i in range(G1_t + 1)), 'G011')
            model.addConstrs((-z[j * (len_stl + 1) + G1_tmin - spl_t[stage] + i, 1] + yA3[1] <= M * G1_4[i + j * (G1_t + 1), 3] for i in range(G1_t + 1)), 'G012')
            model.addConstrs((z[j * (len_stl + 1) + G1_tmin - spl_t[stage] + i, 1] - yA3[1] <= M * (1 - G1_4[i + j * (G1_t + 1), 3]) for i in range(G1_t + 1)), 'G013')

# add terminal constraints to gurantee the states can satisfy the formulae in the next period
# those constraints are determined from offline codes written in Julia
def add_terminal_constrs(model, z, len_disturb_set, len_stl):
    for j in range(len_disturb_set):
        model.addConstr((2*z[(j + 1)*len_stl, 0] + 2*z[(j + 1)*len_stl, 2] <= 8.9159), 'tms01')
        model.addConstr((4*z[(j + 1)*len_stl, 0] + 3*z[(j + 1)*len_stl, 2] <= 17.3219), 'tms02')
        model.addConstr((2*z[(j + 1)*len_stl, 0] + 2*z[(j + 1)*len_stl, 2] >= -2.21), 'tms03')
        model.addConstr((4*z[(j + 1)*len_stl, 0] + z[(j + 1)*len_stl, 2] <= 19.296), 'tms03')
        model.addConstr((2*z[(j + 1)*len_stl, 1] + 2*z[(j + 1)*len_stl, 3] <= 8.9159), 'tms04')
        model.addConstr((4*z[(j + 1)*len_stl, 1] + 3*z[(j + 1)*len_stl, 3] <= 17.3219), 'tms05')
        model.addConstr((2*z[(j + 1)*len_stl, 1] + 2*z[(j + 1)*len_stl, 3] >= -2.21), 'tms06')
        model.addConstr((4*z[(j + 1)*len_stl, 1] + z[(j + 1)*len_stl, 3] <= 19.296), 'tms07')

def Add_obj(model, z, u, len_disturb_set, len_stl, stage):
    if stage == 0:
        temp_F0 = model.addVars((F0_t + 1) * len_disturb_set, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="temp")
        rho_F0_temp = model.addVars((F0_t + 1) * len_disturb_set, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_F0_temp")
        rho_F0 = model.addVars(1 * len_disturb_set, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_F0")
        model.update()
    if stage == 1:
        temp_G1 = model.addVars((G1_t + 1) * len_disturb_set, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="temp")
        rho_G1_temp = model.addVars((G1_t + 1) * len_disturb_set, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_G1_temp")
        rho_G1 = model.addVars(1 * len_disturb_set, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_G1")
        temp_F1 = model.addVars(F1_t * len_disturb_set, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="temp")
        rho_F1_temp = model.addVars(F1_t * len_disturb_set, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_F1_temp")
        rho_F1 = model.addVars(1 * len_disturb_set, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho_F1")
    for j in range(len_disturb_set):
        if stage == 0:
            for i in range(F0_t + 1):
                model.addConstr((temp_F0[i + j*(F0_t + 1), 0] == z[i + j*(len_stl+1), 0] - xA1[0]), "temp1")
                model.addConstr((temp_F0[i + j*(F0_t + 1), 1] == xA1[1] - z[i + j*(len_stl+1), 0]), "temp1")
                model.addConstr((temp_F0[i + j*(F0_t + 1), 2] == z[i + j*(len_stl+1), 1] - yA1[0]), "temp1")
                model.addConstr((temp_F0[i + j*(F0_t + 1), 3] == yA1[1] - z[i + j * (len_stl + 1), 1]), "temp1")
                model.update()
            for i in range((F0_t + 1)):
                model.addGenConstrMin(rho_F0_temp[i + j*(F0_t + 1), 0], [temp_F0[i + j*(F0_t + 1), 0], temp_F0[i + j*(F0_t + 1), 1], temp_F0[i + j*(F0_t + 1), 2], temp_F0[i + j*(F0_t + 1), 3]], name="minconstr")
            model.addGenConstrMax(rho_F0[j, 0], [rho_F0_temp[i + j*(F0_t + 1), 0] for i in range((F0_t + 1))], name="minconstr")
            model.update()
        if stage == 1:
            for i in range(F1_t):
                model.addConstr((temp_F1[i + j * F1_t, 0] == z[i + F1_tmin + j * (len_stl + 1) - spl_t[stage], 0] - xA2[0]), "temp1")
                model.addConstr((temp_F1[i + j * F1_t, 1] == xA2[1] - z[i + F1_tmin + j * (len_stl + 1) - spl_t[stage], 0]), "temp1")
                model.addConstr((temp_F1[i + j * F1_t, 2] == z[i + F1_tmin + j * (len_stl + 1) - spl_t[stage], 1] - yA2[0]), "temp1")
                model.addConstr((temp_F1[i + j * F1_t, 3] == yA2[1] - z[i + F1_tmin + j * (len_stl + 1) - spl_t[stage], 1]), "temp1")
                model.update()
            for i in range(F1_t):
                model.addGenConstrMin(rho_F1_temp[i + j * F1_t, 0], [temp_F1[i + j * F1_t, p] for p in range(4)], name="minconstr")
            model.addGenConstrMax(rho_F1[j, 0], [rho_F1_temp[i + j * F1_t, 0] for i in range(F1_t)], name="minconstr")
            model.update()

            for i in range(G1_t + 1):
                model.addConstr((temp_G1[i + j*(G1_t + 1), 0] == z[G1_tmin - spl_t[stage] + i + j*(len_stl+1), 0] - xA3[0]), "temp1")
                model.addConstr((temp_G1[i + j*(G1_t + 1), 1] == xA3[1] - z[G1_tmin - spl_t[stage] + i + j*(len_stl+1), 0]), "temp1")
                model.addConstr((temp_G1[i + j*(G1_t + 1), 2] == z[G1_tmin - spl_t[stage] + i + j*(len_stl+1), 1] - yA3[0]), "temp1")
                model.addConstr((temp_G1[i + j*(G1_t + 1), 3] == yA3[1] - z[G1_tmin - spl_t[stage] + i + j*(len_stl+1), 1]), "temp1")
            for i in range(G1_t + 1):
                model.addGenConstrMin(rho_G1_temp[i + j * (G1_t + 1), 0], [temp_G1[i + j * (G1_t + 1), p] for p in range(4)], name="minconstr")
            model.addGenConstrMax(rho_G1[j, 0], [rho_G1_temp[i + j*(G1_t + 1), 0] for i in range(G1_t + 1)], name="minconstr")
    rho = model.addVars(len_disturb_set, 1, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="rho")
    for i in range(len_disturb_set):
        if stage == 1:
            model.addGenConstrMin(rho[i, 0], [rho_F1[i, 0], rho_G1[i, 0]], name="minconstr")
        if stage == 0:
            model.addConstr(rho[i, 0] == rho_F0[i, 0], name="minconstr")
    # objective function

    obj = model.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="temp")
    model.addConstr(obj == sum(rho[i, 0] for i in range(len_disturb_set))/len_disturb_set -  sum(u[j, 0]*u[j, 0] + u[j,1]*u[j,1] for j in range(len_stl))/(len_stl*len_stl*100000), "temp1")
    model.setObjective(obj, GRB.MAXIMIZE)
    return obj
