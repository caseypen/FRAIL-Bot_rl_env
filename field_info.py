import numpy as np

# statistics of finishing time of ground truth(unit: s)
fast_mean, fast_sigma = 477.1, 42.4
slow_mean, slow_sigma = 941, 144.4
medi_mean, medi_sigma = 692.1, 92.5
N_pickers_fast, N_pickers_medi, N_pickers_slow = 4, 4, 4
# statistics of moving velocity while picking(unit: m/s)
v_fast, v_slow, v_medi = 0.0566, 0.0337, 0.0424
w_fast, w_slow, w_medi = 1.4662, 0.7663, 1.1238

model_truth = {"0":[fast_mean,fast_sigma,v_fast,w_fast],
               "1":[medi_mean,medi_sigma,v_medi,w_medi],
               "2":[slow_mean,slow_sigma,v_slow,w_slow]}

# estimation of finishing time of one tray based on statisctics data collected in the field
fast_mean_est, fast_sigma_est = 477.1, 42.4
slow_mean_est, slow_sigma_est = 941, 144.4
medi_mean_est, medi_sigma_est = 692.1, 92.5
# statistics data of moving velocity while picking(unit: m/s)
v_fast_est, v_slow_est, v_medi_est = 0.0566, 0.0337, 0.0424
w_fast_est, w_slow_est, w_medi_est = 1.4662, 0.7663, 1.1238

# fields information (unit: m)
origin = np.asarray([0,0])
furrow_width = 28*0.0254
berry_width = 14*0.0254
furrow_length = 5400*0.0254
picker_num = N_pickers_fast+N_pickers_medi+N_pickers_slow
robot_num = 4
robot_v = 1 # m/s
PROC_TIME = 10 # second
y_head = 40*0.0254 # y coordinate of furrow head
center_1 = [(furrow_width+berry_width)*picker_num/2, 0]
center_2 = [(furrow_width+berry_width)*picker_num*3/2, 0] # (x_center2, y_center2)
x_offset = furrow_width+berry_width
y_offset = y_head+furrow_length

INF = 999999