# FRAIL-Bot_rl_env
## Introduction of Problems
m robots n pickers working in a field. n pickers picking strawberries in their furrow. Robots only take full plates of strawberr to a center. We want to optimize the waiting time of all pickers in one episode. The episode is defined as starting from picking to center exchange.
## Environment BerryPick
~~~~{.python}
# len(ft_mean)=pick_num
class BerryPick(object):
	def __init___(self, pick_num, robot_num, ft_mean, ft_var):
	# initialize constant values
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
	def _update
~~~~
## States:
 - Mean of finishing time of n pickers (n*1); 
 - Variance of finishing time of n pickers (n*1);
 - Moving velocity of n pickers;
 - Mean of finish position on furrow; 
 - Variance of finish position on furrow;
 - Location of pickers;
 - Location of Robots;
 - Assignment of working robots;
 - Current waiting time of pickers;

## Action:
- The system can only do action after one robot came back to the center finishing their assignment;
- Action space is 1~(n-m+1);

## Reward:
- -(mean_wt)*alpha+(var_wt)*beta, when done;
- 0, otherwise

## Picker's model:
~~~~{.python}
class picker(object):
	def __init__(self,mean_ft,var_ft):
		self.wait = 0 # 1 for waiting, 0 for picking
		self.center = 0
		self.wait_time = 0
		self.initial_mean = mean_ft
		self.initial_var = var_ft
	def update():
		# dynamically update
	def cur_pick_state():
		return (mean_wt,var_wt)
~~~~

## Robots' model:
~~~~{.python}
class robot(object):
	def __init__(num_robots, num_pickers):
	# initialize constant values
		r = np.ones(num_robots,dtype=int)
		p = np.zeros(num_pickers-num_robots,dtype=int)
		assign = np.concatenate(r,p)
		self.initial_assign = np.random.permutation(assign)
	def cur_robot_state():
	# running to picker
		return (x_bot, y_bot)
~~~~