import math
M = 1000000000          

# parameters about the map
xA1 = [7.5, 10]
yA1 = [7.5, 10]   # zone A1

xA2 = [7.5, 10]
yA2 = [0, 2.5]    # zone A2

xA3 = [0, 3]
yA3 = [0, 3]      # zone A3
  
xmap = [0, 10]    # workspace
ymap = [0, 10]
umax = 3
vmax = 2.5
Disturb_max = 0.001  # maximal position disturbance

Stl_Hrizon = 25

F0_t = 6
F1_tmin = 22
F1_tmax = 25
F1_t = F1_tmax - F1_tmin
G1_tmin = 14
G1_tmax = 15
G1_t = G1_tmax - G1_tmin

z0 = [3, 8, 0, 0]    # the initial states
optimal_state_sequence = [z0]
optimal_control_sequence = []