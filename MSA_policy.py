import numpy as np
import math
from scipy.spatial.distance import cityblock
import matplotlib.pyplot as plt
from re import split


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
def sample_SCENE(sample_ft, p_pos, current_ft, pred_horizon):
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
def est_wait_time(scene_sample, robots, perm_a, time_w):
    w_t = np.zeros(pick_num)
    robot_assign_m=[robot.p_NO for robot in robots]
    idle_idx=robot_assign_m.index(-1)
    robots[idle_idx].p_NO=perm_a[0]
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
w_t = est_wait_time(scene_sample, f_t, r_back_time, robot_assign, perm, time_w)
print w_t.mean(), w_t.std()

def MSA_policy(pickers, robots, predict_h, sample_times):
    for i in range(sample_times):
        est_ft_tray_n=[]
        p_pos_n2=[]
        est_ft_cur_n=[]
        for picker in pickers:
            p_pos_n2.append(picker.p_pos)
            est_mean, est_sigma = picker.static_est()
            est_ft_tray=0
            while(est_ft_tray >= est_mean+est_sigma or est_ft_tray <= est_mean-est_sigma):
                est_ft_tray = np.random.normal(est_mean,est_sigma)
            est_ft_cur = est_ft_tray-picker.picking_time 
            if est_ft_cur<0 or picker.wait: est_ft_cur = 0 # assuming that picker has done
            est_ft_tray_n.append(est_ft_tray)
            est_ft_cur_n.append(est_ft_cur)
        # scene = sample_SCENE(est_ft_tray_n, p_pos, est_ft_cur_n, pred_horizon)
        # p_NO = meta_heuristic(scene)

    return tar_pos, picker.p_NO