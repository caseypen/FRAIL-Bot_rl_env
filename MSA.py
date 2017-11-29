import numpy as np
import math
from scipy.spatial.distance import cityblock
import matplotlib.pyplot as plt
from re import split

# statistics of finishing time of ground truth(unit: s)
fast_mean, fast_sigma = 477.1, 42.4
slow_mean, slow_sigma = 941, 143.9
medium_mean, medium_sigma = 692.1, 92.5
# estimation of finish time from Farangis model
fast_mean_est, fast_sigma_est = 477.1, 42.4
slow_mean_est, slow_sigma_est = 941, 143.9
medium_mean_est, medium_sigma_est = 692.1, 92.5
N_pickers_fast, N_pickers_medium, N_pickers_slow = 4, 4, 4
# statistics of moving velocity while picking(unit: m/s)
# suppose these values are obtained accurately
v_fast, v_slow, v_medium = 0.0566, 0.0337, 0.0424
w_fast, w_slow, w_medium = 1.4662, 0.7663, 1.1238

# sampling of finishing intervals of pickers' finishing time
# resampled from statistics model after finishing 
f_t_sample_fast = np.random.normal(fast_mean,fast_sigma,N_pickers_fast)
f_t_sample_slow = np.random.normal(slow_mean,slow_sigma,N_pickers_slow)
f_t_sample_medium = np.random.normal(medium_mean,medium_sigma,N_pickers_medium)
f_t_initial_state = np.concatenate((f_t_sample_fast,f_t_sample_slow,f_t_sample_medium)) # (12,)

# 0 means fast, 1 means medium, 2 means slow
f_t_model = (np.zeros(N_pickers_fast),np.ones(N_pickers_slow)*2,np.ones(N_pickers_medium))
f_t_model = np.concatenate(f_t_model).astype(int)
# walking velocity and moving velocity of pickers
v_t_pickers = (np.ones(N_pickers_fast)*v_fast,np.ones(N_pickers_slow)*v_slow,np.ones(N_pickers_medium)*v_medium)
v_pickers = np.concatenate(v_t_pickers)
w_t_pickers=(np.ones(N_pickers_fast)*w_fast,np.ones(N_pickers_slow)*w_slow,np.ones(N_pickers_medium)*w_medium)
w_pickers = np.concatenate(w_t_pickers)
# shuffle to initialize the prediction
fur_pos = np.random.permutation(N_pickers_fast+N_pickers_medium+N_pickers_slow)
# index is pickers' position from left to right
f_t_initial_state = f_t_initial_state[fur_pos] # state means finishing interval of current tray
v_pickers = v_pickers[fur_pos]
w_pickers = w_pickers[fur_pos]
f_t_model = f_t_model[fur_pos] # fast, medium or slow
# fields information (unit: m)
origin = np.asarray([0,0])
furrow_width = 28*0.0254
berry_width = 14*0.0254
furrow_length = 5400*0.0254
picker_num = N_pickers_fast+N_pickers_medium+N_pickers_slow
robot_num = 4
robot_v = 1 # m/s
y_head = 40*0.0254 # y coordinate of furrow head

# initialize robots and pickers positions
def field_initialization(picker_num, robot_num, y_head, furrow_width, berry_width, furrow_length):
    center_1 = [(furrow_width+berry_width)*picker_num/2, 0]
    center_2 = [(furrow_width+berry_width)*picker_num*3/2, 0] # (x_center2, y_center2)
    center = np.array([center_1, center_2])
    r_initial_pos = []
    p_initial_pos = []
    for i in range(robot_num):
        r_initial_pos.append(center_1) # robots are initially at center 1
    r_initial_pos = np.asarray(r_initial_pos) # m*2, m1:r_x, m2:r_y

    for i in range(picker_num):
        x = furrow_width/2 + (furrow_width+berry_width)*i
        pos = [x, y_head + furrow_length]
        p_initial_pos.append(pos) 	 # pickers are initially at further end of furrows
    p_initial_pos = np.asarray(p_initial_pos) # n*2, n1:p_x, n2:p_y
    return p_initial_pos, r_initial_pos, center

# sample finishing time based on statistics
def sample_FT(f_t_model, fast_m, fast_s, medium_m, medium_s, slow_m, slow_s):
    sample_ft=np.zeros(len(f_t_model))
    for i, model in enumerate(f_t_model):
        if model==0:
            sample_ft[i]=np.random.normal(fast_m,fast_s)
        if model==1:
            sample_ft[i]=np.random.normal(medium_m,medium_s)
        if model==2:
            sample_ft[i]=np.random.normal(slow_m,slow_s)
    return sample_ft
# obtain furrow number of each picker
def working_fur(p_pos):
    fur_idx=np.ones(len(p_pos))
    for i, pos in enumerate(p_pos):
        fur_idx[i]=math.floor((pos[0]-furrow_width/2)/(furrow_width+berry_width))+1
    return fur_idx
# produce one scenario based on estimation sampling
# sample_ft is sampled interval of next tray;
# current_ft is estimation of finishing interval of current tray: current_ft = sampled_ft-delta_t
def sample_SCENE(sample_ft, p_pos, current_ft, pred_horizon, fur_idx):
    scene={} # target position and finish interval
    x_offset = furrow_width+berry_width
    y_offset = y_head+furrow_length
    # The picker will keep their order in next pick_num furrows
    for i,ft_interval, ft_cur in zip(range(len(sample_ft)),sample_ft, current_ft):
        n = int(math.floor((pred_horizon-ft_cur)/ft_interval) + 1) # obtain sample points on a furrow
        pos_list, ft_list = [], []
        cur_pos = np.copy(p_pos[i].reshape(-1,1))
        nxt_pos = np.copy(cur_pos)
        for j in range(n):
            if j==0:
                nxt_pos[1] = cur_pos[1] - v_pickers[i]*ft_cur # y
            else:
                nxt_pos[1] = cur_pos[1] - v_pickers[i]*ft_interval # y
            ft_tray = ft_interval # finish time of next tray
            # furrow change happened
            if nxt_pos[1] < y_head:
                nxt_pos[0] += picker_num*x_offset # keep the same order
                nxt_pos[1] = y_offset - (y_head-nxt_pos[1]) 
                ft_tray = ft_interval + (cityblock(nxt_pos,cur_pos))/w_pickers[i] # finishing time of current tray should add walking time of picker
            pos_list.append(nxt_pos)
            ft_list.append(ft_tray)
            cur_pos=np.copy(nxt_pos)
            nxt_pos=np.copy(cur_pos)
        if len(pos_list)>1:
            scene['pos'+str(i)]=np.concatenate(pos_list,axis=1)
        else:
            scene['pos'+str(i)]=nxt_pos
        scene['ft'+str(i)]=np.asarray(ft_list) # also add 
    return scene
# render the scenario sampled points and current state points
def render(scene_sample,p_initial_pos):
	sample_points=[]
	x_offset = furrow_width+berry_width
	y_offset = y_head+furrow_length
	for i in scene_sample.keys():
	    char_sp = split('(\d+)',i)
	    if char_sp[0] == 'pos':
	        if len(scene_sample[i]) > 1:
	            sample_points.append(scene_sample[i])
	        print scene_sample[i].shape
	sample_points=np.concatenate(sample_points,axis=1)
	plt.scatter(p_initial_pos[:,0],p_initial_pos[:,1], c='red')
	plt.scatter(sample_points[0],sample_points[1])
	plt.scatter(p_initial_pos[:,0],np.ones(p_initial_pos[:,1].shape)*y_head, c='yellow')
	plt.scatter(p_initial_pos[:,0]+x_offset*picker_num, p_initial_pos[:,1], c='red')
	plt.scatter(p_initial_pos[:,0]+x_offset*picker_num,np.ones(p_initial_pos[:,1].shape)*y_head, c='yellow')
	plt.show()
# obtain initial permutation of decision list in prediction horizon
def produce_perm(scene_sample):
    perm=[]
    for i in scene_sample.keys():
        char_sp = split('(\d+)', i)
        if char_sp[0]=='pos':
            n=scene_sample[i].shape[1]
            num_list=[]
            for j in range(n):
                num_list.append(int(char_sp[1])+j*picker_num)
            perm.append(num_list)
    perm=np.concatenate(perm)
    perm=np.sort(perm)
    return perm  

p_initial_pos, r_initial_pos, center = field_initialization(picker_num, robot_num, y_head, furrow_width, berry_width, furrow_length)
prediction_horizon=60*20 # 20 mins as prediction horizon
sample_ft = sample_FT(f_t_model,fast_mean_est,fast_sigma_est,medium_mean_est,medium_sigma_est,slow_mean_est,slow_sigma_est) # sampled points based on current prediction
p_pos = p_initial_pos # current position of pickers
fur_idx = working_fur(p_pos) # working furrows of current pickers
f_t_state = np.copy(f_t_initial_state) # finishing time of current tray
scene_sample=sample_SCENE(sample_ft, p_pos, f_t_state, prediction_horizon, fur_idx)
print scene_sample
render = False
if render:
	render(scene_sample, p_pos)

perm=produce_perm(scene_sample)
print perm
robot_assign=list(np.ones(robot_num)*(-1)) # -1 means not assigned
robot_pos = r_initial_pos
r_back_time = np.zeros(robot_num)
time_w = np.zeros(picker_num)
# f_t_state is estimation of current tray
def est_wait_time(scene_sample, f_t, r_back_time, robot_assign, perm_a, time_w):
    w_t = np.zeros(f_t.shape)
    # Next idle robot
    for i in perm_a:
        # no available robot, update states to available time
        if robot_assign.count(-1) < 1: 
            delta_t = np.amin(r_back_time)
            print delta_t
            robot_idx = np.argmin(r_back_time)
            # label idle robot
            serve_point = robot_assign[robot_idx]
            fur_num = serve_point%picker_num
            col_num = int(serve_point)/picker_num
            f_t[fur_num] = scene_sample[fur_num][col_num] 
            robot_assign[robot_idx] = -1
            # update f_t
            r_back_time -= delta_t
            for m,ft in enumerate(f_t):
                if ft < delta_t:
                    f_t[m]=0
                    w_t[m] += delta_t-ft
                else:
                    f_t[m] = ft - delta_t
            # do re-sampling
            print f_t,w_t
        k=robot_assign.index(-1) #return first idle robot number
        robot_assign[k]=i
        print robot_assign
        fur_num = i%picker_num
        col_num = int(i)/picker_num
        target_pos=scene_sample['pos'+str(fur_num)][:,col_num]
        target_ft=scene_sample['ft'+str(fur_num)][col_num]
        print target_pos
        if target_pos[0]>picker_num*(furrow_width+berry_width):
            r_to_target=cityblock(target_pos,center[1])/robot_v
            r_back_time[k]=np.maximum(r_to_target,target_ft)+r_to_target
        else:
            r_to_target=cityblock(target_pos,center[0])/robot_v
            r_back_time[k]=np.maximum(r_to_target,target_ft)+r_to_target
        print r_back_time
    return w_t    

f_t = np.copy(f_t_state)
w_t = wait_time(scene_sample, f_t, r_back_time, robot_assign, perm, time_w)
print w_t.mean(), w_t.std()
