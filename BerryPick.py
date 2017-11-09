import numpy as np 

class Robot(object):
	def __init__(self, m_robots, initial_assign, r_vel, initial_pos):
		self.arrival = 1
		self.m = m_robots
		self.cur_pos = inital_pos # m*2
		self.speed = r_vel
	def run_to(target_pos, dt): # Manhattan distance 
		# update m robots position: cur_position
	def load_wait():
		# robot waiting at estimating running point
	def run_back():
		# robot running back to the center
class Picker(object):
	def __init__(self, n_pickers, initial_ft_mean, initial_ft_var, p_vel, p_pos, w_vel, center):
		self.wt = 0
		self.n = n_pickers
		self.cur_pos = p_pos
		self.speed = p_vel
		self.center = center
	def picking_move(dt):
		# update position while pickers are picking
	def walk(dt):
		#update position of pickers while it is walking
	def update_wt(dt):
		#updating waiting time of pickers
		self.wt += dt

class BerryPick(object):
	def __init___(self, pick_num, robot_num, ft_mean, ft_var, rb_pos, p_pos):
	# initialize constant values
		self.p_num = pick_num
		self.r_num = robot_num
		self.initial_ft = 
		pickers = []
		# initialize n pickers model with initial speed and initial variance
		for i, mean_ft, var_ft in zip(range(pick_num), ft_mean, ft_var):
			pickers[i] = picker(mean_ft, var_ft)
		# inital assignment
		r = np.ones(num_robots,dtype=int)
		p = np.zeros(num_pickers-num_robots,dtype=int)
		assign = np.concatenate(r,p)
		self.initial_assign = np.random.permutation(assign)
	def reset(self):
		return self.s_t
	# reset environment randomly
	def step(self,action):
		return self.s_t, reward, self.done
	# execute one action
	def render():
	# render current state to picture
	def _update():
	# update states
	def _idle_robot():
	# check if there is idle robot in the center
