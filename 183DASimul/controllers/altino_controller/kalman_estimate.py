import numpy as np
import Frispy

class StateEstimator:
    '''State estimator class wrapper'''
    def __init__(self, inital_state):
        '''State estimator initializer. 
        Takes the full set of state variables as input. List-like shape (12,)'''
        
        # Initial condition
        self.state = initial_state

        # Initial covariance estimate, E[x0*x0^T],
		# We will use the assumption that the state estimate has:
		# 		1. Uncorrelated initial conditions
		# 		2. High uncertainty
		self.P_covar_estimate = np.eye(4)*9999

		# Keeps track of whether the state is a priori or a posteriori,
		# in case the Kalman filtering steps are taken out of order
		self.SE_is_a_priori = False

    def get_state(self):
        return self.state

    def dynamics_propagation(self,input):
        
    