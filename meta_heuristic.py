import numpy as np
import math
from scipy.spatial.distance import cityblock
import matplotlib.pyplot as plt
from re import split
from field_info import *
from collections import Counter

def sample_SCENE(sample_ft, p_pos, v_pickers, w_pickers, current_ft, pred_horizon,centers):
    scene={} # target position and finish interval
    x_offset = furrow_width+berry_width
    y_offset = y_head+furrow_length
    sample_nums=[]
    # The picker will keep their order in next pick_num furrows
    for i,ft_interval, ft_cur in zip(range(len(sample_ft)),sample_ft, current_ft):
        n = int(math.floor((pred_horizon-ft_cur)/ft_interval) + 1) # obtain sample points on a furrow
        sample_nums.append(n)
        pos_list, ft_list, center_list = [], [], []
        cur_pos = np.copy(p_pos[i].reshape(-1,1))
        cur_center = np.copy(centers[i]).reshape(-1,1)
        nxt_pos = np.copy(cur_pos)
        nxt_center = np.copy(cur_center)
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
                fsat_tray = ft_interval + (cityblock(nxt_pos,cur_pos))/w_pickers[i] # finishing time of current tray should add walking time of picker
                nxt_center[0]+=picker_num*x_offset
            pos_list.append(nxt_pos)
            ft_list.append(ft_tray)
            center_list.append(nxt_center)
            cur_pos=np.copy(nxt_pos)
            cur_center=np.copy(nxt_center)
        if len(pos_list)>1:
            scene['picker'+str(i)]=np.concatenate(pos_list,axis=1).T
            scene['center'+str(i)]=np.concatenate(center_list,axis=1).T
        else:
            scene['picker'+str(i)]=nxt_pos.T
            scene['center'+str(i)]=nxt_center.T
        scene['ft'+str(i)]=np.asarray(ft_list) # also add 
    scene['sample_nums']=np.asarray(sample_nums)
    return scene

def produce_perm(sample_nums):
    perm=[]
    for i in range(picker_num):
        print sample_nums[i]
        for j in range(sample_nums[i]):
            perm.append(i+j*picker_num)
    perm=np.sort(perm)
    return perm
def est_wait_time(scene_sample, robots, perm_a, time_w):
    w_t = np.copy(time_w)
    ft_cur = [scene_sample['ft'+str(i)][0] for i in range(picker_num)]  
    fur_orders = [0]*picker_num
    for i in perm_a:
        # Next idle robot
        robot_assign_m=[robot.p_NO for robot in robots]
        n=robot_assign_m.count(-1)
        if n>0:
            j=robot_assign_m.index(-1)
            picker_NO=i%picker_num
            point_order=int(i)/picker_num
            robots[j].tar_pos=scene_sample['picker'+str(picker_NO)][point_order,:]
            robots[j].p_NO=picker_NO
            robots[j].cal_time(scene_sample['ft'+str(picker_NO)][point_order],
                               scene_sample['center'+str(picker_NO)][point_order,:])
            continue
        else:
            
            r_back_time=[robot.back_time for robot in robots]
            r_serve_pickers=[robot.p_NO for robot in robots]
            r_not_serve=[p for p in range(picker_num) if p not in r_serve_pickers]
#             print "current assignment", robot_assign_m
#             print "current finish time", ft_cur
#             print "run back time", r_back_time
            delta_t = np.amin(r_back_time)
            idx_idle = np.argmin(r_back_time)
            for k in r_not_serve:
                if delta_t > ft_cur[k]:
                    ft_cur[k] = 0
                    w_t[k] += (delta_t-ft_cur[k])
                else:
                    ft_cur[k] -= delta_t
            for robot in robots:
                robot.back_time -= delta_t
                if robot.run_time >= delta_t:
                    robot.run_time -=delta_t
                    if ft_cur[robot.p_NO] >= delta_t:
                        ft_cur[robot.p_NO]-=delta_t
                    else:
                        w_t[robot.p_NO] += delta_t-ft_cur[robot.p_NO]
                        ft_cur[robot.p_NO]=0
                else:
                    if ft_cur[robot.p_NO] >= delta_t:
                        ft_cur[robot.p_NO]-=delta_t
                    else:
                        if ft_cur[robot.p_NO] < robot.run_time:
                            w_t[robot.p_NO] += robot.run_time-ft_cur[robot.p_NO]
                        fur_orders[robot.p_NO]+=1
                        order=fur_orders[robot.p_NO]
                        total_orders = len(scene_sample['ft'+str(robot.p_NO)])
                        if order > total_orders-1: # next point is not in the list
                            continue
                        else:
                            ft_cur[robot.p_NO]=scene_sample['ft'+str(robot.p_NO)][order]
                    robot.run_time = 0
            robots[idx_idle].p_NO=-1
    return w_t

def constraint(perm,sample_nums):
    constrain_perm=list(perm)
    for i in range(picker_num):
        fur_perm=[]
        fur_perm_idx=[]
        # obtain all furrow sample points
        if sample_nums[i]>1:
            for j in range(sample_nums[i]):
                perm_el = i+j*picker_num
                fur_perm.append(perm_el)
                fur_perm_idx.append(constrain_perm.index(perm_el))# index of furrow point in perm
            for j,perm_idx in enumerate(np.sort(fur_perm_idx)):
                constrain_perm[perm_idx]=fur_perm[j]
    return constrain_perm

def cost_func(scenario, robots, perm, wait_time_n):
    w_t=est_wait_time(scenario, robots, perm, wait_time_n)
#     return np.mean(w_t)*0.8+np.std(w_t)*0.2
    return np.mean(w_t)

def meta_heuristic_policy(N_search_times, perm, scenario, robots, wait_time_n):
    min_perm=np.copy(perm)
    sample_nums=scenario['sample_nums']
    robots_cp=list(robots)
    for i in range(N_search_times):
        perm_trial=np.copy(min_perm)
        a=np.random.randint(len(perm_trial))
        b=np.random.randint(len(perm_trial))
        if a>b:
            a,b=b,a
        perm_trial[a:b+1]=perm_trial[a:b+1][::-1]
        perm_trial=constraint(perm_trial,sample_nums)
        trial_cost=cost_func(scenario, robots_cp, perm_trial, wait_time_n)
        if trial_cost < min_cost:
            min_perm=np.copy(perm_trial)
            min_cost=trial_cost
            print min_perm
            print min_cost
    return min_perm

def MSA_Policy(pickers, robots, s_nums):
    est_ft_tray_n=[]
    p_pos_n2=[]
    est_ft_cur_n=[]
    est_v_pickers_n=[]
    est_w_pickers_n=[]
    wait_time_n=[]
    center_n=[]
    pickers, robots = field_initialization()    
    for picker in pickers:
        p_pos_n2.append(picker.p_pos)
        est_mean, est_sigma = picker.static_est()
        est_ft_tray=0
        while(est_ft_tray >= est_mean+est_sigma or est_ft_tray <= est_mean-est_sigma):
            est_ft_tray = np.random.normal(est_mean,est_sigma)
        est_ft_cur = est_ft_tray-picker.picking_time 
        if est_ft_cur<0 or picker.wait: est_ft_cur = 0  # assuming that picker has done
        est_ft_tray_n.append(est_ft_tray)
        est_ft_cur_n.append(est_ft_cur)
        est_v_pickers_n.append(picker.p_m_vel_est)
        est_w_pickers_n.append(picker.p_w_vel_est)
        wait_time_n.append(picker.wait_time)
        center_n.append(picker.center)
    est_ft_cur_n = np.asarray(est_ft_cur_n)
    est_ft_tray_n = np.asarray(est_ft_tray_n)
    p_pos_n2 = np.asarray(p_pos_n2)
    center_n = np.asarray(center_n)
    first_selections = []
    N_search_times=10000
    pred_horizon = 1200
    # sampling scenario
    for itr in range(s_nums):
        print("*******sample %i********"%itr)
        scenario = sample_SCENE(est_ft_tray_n, p_pos_n2, est_v_pickers_n, 
                                est_w_pickers_n, est_ft_cur_n, pred_horizon, center_n)
        perm = produce_perm(scenario['sample_nums'])
        perm = np.random.permutation(perm)
        min_perm = meta_heuristic_policy(N_search_times, perm, scenario, robots, wait_time_n)
        first_selections.append(min_perm[0])
        print min_perm
    print first_selections
    count = Counter(first_selections)
    print count
    picker_NO=count.most_common()[0][0]
    tar_pos=scenario['picker'+str(picker_NO)][0]
    print tar_pos
    return tar_pos, picker_NO
