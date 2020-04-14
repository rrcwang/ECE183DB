"""
This file contains an example of how to use FrisPy to visualize a simulated trajectory.
Edited to test visualization.
PLEASE SEE LINK BELOW FOR A DESCRIPTION OF THE MODEL USED
https://repository.arizona.edu/handle/10150/626742
"""
import FrisPy

#Some example initial conditions are in simple_initial_conditions.txt, located a the top level of this repository.
disc = FrisPy.create_disc(filename = "simple_initial_conditions.txt")
times, trajectory = FrisPy.get_trajectory(disc)

#Try plotting
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    x,y,z = trajectory.T
        
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    fig.show()
    print(times)
except Exception:
    print("Matplotplot not installed. Cannot visualize example.")
