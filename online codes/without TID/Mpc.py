
from Print_plot import *
from pro_set import *
from pro_check import *

class MPC:
    def __init__(self, len_stl, cur_t):
        self.len_stl = len_stl
        self.cur_t = cur_t
        self.state = optimal_state_sequence
        self.input = None
        self.disturb_set = []

# add a disturb set to calculate a set of input to satisfy the formuales(the disturbs are only used to find inputs)
    def set(self, disturb): 
        self.disturb_set.append(disturb)
        if (len(self.disturb_set) > 30):
            print("no inputs can satisfy all the disturb")
            sys.exit(0)
        state, input, obj = Set_Prob(self.cur_t, self.disturb_set, self.len_stl)

        self.input = []
        for i in range(self.len_stl):
            self.input.append([input[i, 0], input[i, 1]])

        self.state = []
        for i in range(self.len_stl+1):
            self.state.append([state[i, 0], state[i, 1], state[i, 2], state[i, 3]])

        print("the objective function (stl robustness + input energy) equals to", obj)


# check whether the input can satisfy all the disturbs and return the worst set of disturbs
    def check(self):
        disturb_var, obj = Set_Prob_Robust(self.cur_t, self.input, self.len_stl)
        print("---------------the worst rho is ", obj,"----------------")
        return disturb_var, obj


# after finding an optimized input, give the current states random disturbs
    def apply(self):
        optimal_control_sequence.append([self.input[self.cur_t][0], self.input[self.cur_t][1]])
        self.state[self.cur_t + 1][0] = optimal_state_sequence[self.cur_t][0] + 0.5*optimal_state_sequence[self.cur_t][2] + 0.125*optimal_control_sequence[self.cur_t][0] + Disturb_max * 0.1 * random.randint(-10, 10)
        self.state[self.cur_t + 1][1] = optimal_state_sequence[self.cur_t][1] + 0.5*optimal_state_sequence[self.cur_t][3] + 0.125*optimal_control_sequence[self.cur_t][1] + Disturb_max * 0.1 * random.randint(-10, 10)
        optimal_state_sequence.append([self.state[self.cur_t + 1][0], self.state[self.cur_t + 1][1], self.state[self.cur_t + 1][2], self.state[self.cur_t + 1][3]])
    
    
    def print(self):
        printSolution(optimal_state_sequence)




