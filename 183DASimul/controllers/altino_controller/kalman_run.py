from state_estimator import StateEstimator

init_conditions = [ 0,      -4.2,       1,        # x, y, z
                    1,      1,          1,          # dx, dy, dz
                    0,      1,          0,          # phi, theta, gamma
                    0,      0,          100 ]       # phidot, thetadot, gammadot

SE = StateEstimator(init_conditions)

SE.dynamics_propagation(init_conditions)
print('SE_a_priori: ' + str((SE.get_state())))

A = SE.get_state().aslist()
B = A[0:3]+A[6:9]
B[0]+=0.01

SE.measurement_update(B)
print('SE_a_posteori: ' + str((SE.get_state())))