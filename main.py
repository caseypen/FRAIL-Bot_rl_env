import numpy as np
import math
from scipy.spatial.distance import cityblock
import matplotlib.pyplot as plt
from re import split
from field_info import *

class picker_state(object):
    def __init__(self, p_m_vel, p_w_vel, p_m_vel_est, p_w_vel_est, initial_p_pos, predict_model, center, picker_NO): 
        self.p_pos = initial_p_pos
        self._pos_nxt_x = initial_p_pos[0]+picker_num*x_offset
        self.p_m_vel = p_m_vel
        self.p_w_vel = p_w_vel
        self.p_m_vel_est = p_m_vel_est
        self.p_w_vel_est = p_w_vel_est
        self.pred_m = int(predict_model)
        self.ft_est = self.static_est() # initially guess from statistics data 
        self.ft_est_cur = self.ft_est
        self.picking_time=0 # record how long of picking
        self.center = center
        self.wait_time = 0 # this variable is updated from outside signal
        self.picker_NO = picker_NO
        # self.change_fur_t = _walk_t_nxt(initial_p_pos)
        self.walk_time = self._walk_t_nxt(initial_p_pos)
        self.walk = False
        self.serve_ready = False
        self.wait = False
        self.sample_ft()
        # self.cur_fur = _fur_cal()
    def mechanistic_est_update(self,delta_t):
        self.ft_est_cur = self.ft_one_tray - delta_t
        if self.ft_est_cur <= 0:
            self.ft_est_cur = 0
        return self.ft_est_cur
    def static_est(self): # initial guess of finishing time of one tray, it should be updated in bayesian model
        if self.pred_m == 0:
            mean, sigma = fast_mean_est, fast_sigma_est # global variable
        if self.pred_m == 1:
            mean, sigma = medi_mean_est, medi_sigma_est 
        if self.pred_m == 2:
            mean, sigma = slow_mean_est, slow_sigma_est 
        return mean, sigma
    # updating for simulation, true value but not accessible for estimation
    def sample_ft(self):
        sample=-1
        if self.pred_m == 0:
            mean = fast_mean
            sigma = fast_sigma # global variable
        if self.pred_m == 1:
            mean = medi_mean
            sigma = medi_sigma 
        if self.pred_m == 2:
            mean = slow_mean
            sigma = slow_sigma
        while(sample >= mean+sigma or sample <= mean-sigma):
            sample = np.random.normal(mean, sigma) # sampling from a unknown distribution as a ground truth
        self.ft_tru_cur=sample
        self.picking_time=0
        self.wait = False
    def _walk_t_nxt(self, p_pos):
        pos_start = np.asarray([p_pos[0],y_head])
        pos_end = np.asarray([p_pos[0]+x_offset*picker_num, y_offset])
        t = cityblock(pos_start,pos_end)/self.p_w_vel
        return t
    def state_update(self, delta_t): 
        if self.ft_tru_cur > 0:
            if not self.walk:
                ft_cur_fur = (self.p_pos[1]-y_head)/self.p_m_vel
                if self.ft_tru_cur >= ft_cur_fur: 
                    self.ft_tru_cur += self.walk_time # need to change furrow to finsih a tray
                    if delta_t <= ft_cur_fur:
                        self.ft_tru_cur -= delta_t
                        self.p_pos[1] -= delta_t*self.p_m_vel
                        self.picking_time += delta_t
                    elif delta_t >= self.ft_tru_cur:
                        self.p_pos[0] = self._pos_nxt_x
                        d_t = self.ft_tru_cur - self.walk_time - ft_cur_fur 
                        self.p_pos[1] = y_offset - d_t*p_m_vel
                        # if not self.serve_ready:
                        #     self.wait_time += delta_t - self.ft_tru_cur
                        #     # print "here!"
                        # self.ft_tru_cur = 0
                        # self.walk_time = _walk_t_nxt(self.p_pos)
                        # self._pos_nxt_x = self.p_pos[0]+x_offset*picker_num
                        # self.walk = False
                        # self.center += picker_num*x_offset
                        if not self.serve_ready:
                            self.wait_time += delta_t - self.ft_tru_cur
                            self.ft_tru_cur = 0  
                            self.picking_time += self.ft_tru_cur-self.walk_time
                            self.wait = True
                        else:
                            self.sample_ft()  
                            delta_t -= self.ft_tru_cur
                            self.serve_ready=False
                            # self.picking_time = 0
                            self.state_update(delta_t)
                            # print "here!"
                        self._pos_nxt_x = self.p_pos[0]+x_offset*picker_num
                        self.walk = False
                        self.center += picker_num*x_offset
                    else: # walk flag means that picker will keep walking state during delta_t
                        self.walk = True
                        # update to the instant at y_head
                        self.ft_tru_cur -= ft_cur_fur
                        delta_t -= ft_cur_fur
                        self.picking_time += ft_cur_fur
                else: # picker finish the tray in current furrow
                    self.walk = False
                    if self.ft_tru_cur >= delta_t:
                        self.p_pos[1] -= delta_t*self.p_m_vel
                        self.ft_tru_cur -= delta_t
                        self.picking_time += delta_t
                    else:
                        self.p_pos[1] -= self.ft_tru_cur*self.p_m_vel
                        if not self.serve_ready:
                            # print "here"
                            # print delta_t, self.ft_tru_cur
                            self.wait_time += delta_t-self.ft_tru_cur
                            self.picking_time += self.ft_tru_cur
                            self.ft_tru_cur = 0
                            self.wait = True
                        else:
                            self.sample_ft()
                            delta_t -= self.ft_tru_cur
                            # self.picking_time = 0
                            self.serve_ready=False
                            self.state_update(delta_t)
            if self.walk:
                if delta_t <= self.walk_time:
                    self.walk_time -= delta_t
                    self.ft_tru_cur -= delta_t
                    if delta_t*self.p_w_vel <= self._pos_nxt_x-self.p_pos[0]:
                        p_pos[0] += delta_t*self.p_w_vel # means that pickers are walking
                        p_pos[1] = y_head
                    else:
                        self.p_pos[0] = self._pos_nxt_x
                        t_x = (self._pos_nxt_x - self.p_pos[0])/self.p_w_vel
                        p_pos[1] =  y_head + (delta_t-t_x)*self.p_w_vel
                elif delta_t < self.ft_tru_cur:
                    self.ft_tru_cur -= delta_t
                    self.picking_time += delta_t-self.walk_time
                    self.p_pos[0] = self._pos_nxt_x
                    self.p_pos[1] = y_offset - self.p_m_vel*(delta_t-self.walk_time)
                else:
                    self.p_pos[0] = self._pos_nxt_x
                    self.p_pos[1] = y_offset - self.ft_tru_cur*self.p_m_vel
                    if not self.serve_ready:
                            self.wait_time += delta_t - self.ft_tru_cur
                            self.picking_time += self.ft_tru_cur-self.walk_time
                            self.ft_tru_cur = 0
                            self.wait=0
                    else:
                        self.sample_ft()
                        delta_t -= self.ft_tru_cur
                        self.serve_ready = False
                        # self.picking_time = 0
                        self.state_update(delta_t)
                    self.walk = False
                    self.center += picker_num*x_offset
                    # print "here!!!"
                    self._pos_nxt_x = self.p_pos[0]+x_offset*picker_num
        elif not self.serve_ready:
            self.wait_time += delta_t
            self.wait = True
            # print "here!!"
        else: # ready to serve and ft=0
            self.serve_ready = False
            self.sample_ft()
            # self.picking_time=0
            self.state_update(delta_t)


class robot_state(object):
    def __init__(self, r_vel):
        self.vel = r_vel
        self.exe_time = 0 # if 0, means finish task and start running back
        self.tar_pos = np.zeros(2) 
        self.p_NO = -1 # if -1 means, idle, otherwise serving picking NO.
        self.run_time = 0 # if 0 means arrival
        self.back_time = 0 # if 0 means idle
    def cal_time(self, ft_cur_tray, center):
        self.run_time = cityblock(self.tar_pos, center)/self.vel
        self.exe_time = np.maximum(ft_cur_tray, self.run_time)+PROC_TIME
        self.back_time = self.run_time + self.exe_time        

# initialize robots and pickers positions
def field_initialization():
    pickers = [] # for each pickers
    robots = []
    # model for each picker, ground truth and estimation
    # 0 means fast, 1 means medium, 2 means slow
    model_num = (np.zeros(N_pickers_fast),np.ones(N_pickers_slow)*2,np.ones(N_pickers_medi))
    model_num = np.concatenate(model_num).astype(int)   
    v_pickers = (np.ones(N_pickers_fast)*v_fast,np.ones(N_pickers_slow)*v_slow,np.ones(N_pickers_medi)*v_medi)
    v_pickers = np.concatenate(v_pickers)
    w_pickers = (np.ones(N_pickers_fast)*w_fast,np.ones(N_pickers_slow)*w_slow,np.ones(N_pickers_medi)*w_medi)
    w_pickers = np.concatenate(w_pickers)
    v_pickers_est = (np.ones(N_pickers_fast)*v_fast_est, np.ones(N_pickers_slow)*v_slow_est, np.ones(N_pickers_medi)*v_medi_est)
    v_pickers_est = np.concatenate(v_pickers_est)
    w_pickers_est = (np.ones(N_pickers_fast)*w_fast_est, np.ones(N_pickers_slow)*w_slow_est, np.ones(N_pickers_medi)*w_medi_est)
    w_pickers_est = np.concatenate(w_pickers_est)
    perm = np.random.permutation(picker_num)
    
    model_num = model_num[perm]
    v_pickers = v_pickers[perm]
    w_pickers = w_pickers[perm]
    v_pickers_est = v_pickers_est[perm]
    w_pickers_est = w_pickers_est[perm]
    # initialization for pickers position
    for i in range(picker_num):
        x = furrow_width/2 + (furrow_width+berry_width)*i
        pos = [x, y_head + furrow_length] # a list [x,y]
        picker = picker_state(v_pickers[i],w_pickers[i], v_pickers_est[i], w_pickers_est[i], pos, model_num[i], center_1, i)
        pickers.append(picker)
    
    for j in range(robot_num):
        robot = robot_state(robot_v)
        robots.append(robot)

    return pickers, robots
# # # calculate executing time of robots and figure out next event coming of systems
# def cal_exe_time(tar_pos, picker):
#     r_to_picker = cityblock(tar_pos, picker.center)/robot_v
#     r_exe_time = np.maximum(r_to_picker, picker.ft_tru_cur) + PROC_TIME
#     r_back_time = r_exe_time + r_to_picker
#     return r_to_picker, r_exe_time, r_back_time
# def pickers_state_update(delta_t, pickers): 
#     for picker in pickers:
#         picker.state_update(delta_t)
def state_print(pickers):
    ft=[]
    wt=[]
    for picker in pickers:
        ft.append(picker.ft_tru_cur)
        wt.append(picker.wait_time)
    print "waiting time",wt
    print "finishing time",ft
def MSA_policy_manual(pickers, robots):
    
    p_NO = int(raw_input())
    tar_pos = np.zeros(2)
    # print pickers[p_NO].p_m_vel
    # print pickers[p_NO].ft_tru_cur
    tar_pos[1] = pickers[p_NO].p_pos[1]-pickers[p_NO].ft_tru_cur*pickers[p_NO].p_m_vel
    tar_pos[0] = pickers[p_NO].p_pos[0]
    if tar_pos[1]<y_head:
        t_f_fur=(pickers[p_NO].p_pos[1]-y_head)/pickers[p_NO].p_m_vel
        tar_pos[1]=y_offset-(pickers[p_NO].ft_tru_cur-t_f_fur)*pickers[p_NO].p_m_vel
        tar_pos[0]+=picker_num*x_offset
    print tar_pos
    # robot_serving = [robot.p_NO for robot in robots]
    # print robot_serving
    return tar_pos, p_NO
def main():
    pickers, robots = field_initialization()    
    ft_tray = [picker.ft_tru_cur for picker in pickers]
    model = [picker.pred_m for picker in pickers]
    print ft_tray, model
    while True:
        r_assign = [robot.p_NO for robot in robots]
        # print "current assignment", r_assign
        n = r_assign.count(-1)
        if n > 1: # only for initialization with robot_num
            for i in range(n):
                tar_pos, p_NO = MSA_policy(pickers, robots) # assign means pickers' number
                robots[i].tar_pos = np.copy(tar_pos)
                robots[i].p_NO = p_NO
                robots[i].cal_time(pickers[robots[i].p_NO].ft_tru_cur,pickers[robots[i].p_NO].center) # calculate exe time, running time and back time
                print pickers[robots[i].p_NO].ft_tru_cur
            print [robot.back_time for robot in robots]
        elif n==1: # when all robot are assigned, we need to calculate delta_t of next event coming
             state_print(pickers)
             tar_pos, p_NO = MSA_policy(pickers, robots)
             idx_idle = r_assign.index(-1)
             robots[idx_idle].p_NO = p_NO
             robots[idx_idle].tar_pos = np.copy(tar_pos)
             robots[idx_idle].cal_time(pickers[robots[idx_idle].p_NO].ft_tru_cur,pickers[robots[idx_idle].p_NO].center)
        else: # update state to the instant of next robot back
            r_assign = [robot.p_NO for robot in robots]
            print "current assignment", r_assign
            r_back_time=[robot.back_time for robot in robots]
            r_serve_pickers=[robot.p_NO for robot in robots]
            # r_exe_time=[robot.exe_time for robot in robots]
            print r_back_time
            # print r_serve_pickers
            r_not_serve=[p for p in range(picker_num) if p not in r_serve_pickers]
            # print r_not_serve
            delta_t = np.amin(r_back_time) # we want the system update to this instant
            print "passed time", delta_t
            idx_min = np.argmin(r_back_time)
            for i in r_not_serve:
                # print pickers[i].ft_tru_cur
                pickers[i].state_update(delta_t)
            for robot in robots:
                robot.back_time -= delta_t
                if robot.run_time > delta_t:
                    robot.run_time -= delta_t
                    pickers[robot.p_NO].state_update(delta_t)
                else:                     
                    if pickers[robot.p_NO].ft_cur_fur < robot.run_time:
                        pickers[robot.p_NO].state_update(robot.run_time)
                        pickers[robot.p_NO].serve_ready=True
                        pickers[robot.p_NO].state_update(delta_t-robot.run_time)
                    else:
                        pickers[robot.p_NO].serve_ready = True # ready state is not obtained immediately!
                        pickers[robot.p_NO].state_update(delta_t)
                    robot.r_run_time = 0
            robots[idx_min].p_NO=-1
if __name__ == "__main__":
    main()