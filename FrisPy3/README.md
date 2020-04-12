# FrisPy3

Based on FrisPy by Tom McClintock | tmcclintock/FrisPy
Translated to Python 3 by Richard W.


This repository contains a physical model for a flying disc. Using this code, one can simulate trajectories of discs with varying initial conditions, while also allowing the physical model to be changed. This is useful for analyzing the mechanics of a disc in terms of its design, as well as creating simulated throws for things like disc launchers or other helpful tools.

This is a pure Python rebuild of the old FrisPy code, which included a version of the integrator written in C for speed. To obtain a fast version of the modeling code, either roll back to an old version or check out the [Frisbee_Simulator](https://github.com/tmcclintock/Frisbee_Simulator) repository.

## Running

Check out `example.py` for an example of how to run and view results. It boils down to doing:
```python
disc = FrisPy.create_disc(filename = "initial_conditions_filename.txt")
times, trajectory = FrisPy.get_trajectory(disc)
```
and that's it.

## Initial Conditions

FrisPy takes 12 inital conditions values, defined in a tab-separated values file. They are ordered as:
```
#x  y   z   vx  vy  vz  phi theta   gamma   phidot  thetadot    gammadot
```

* [__x, y, z__] are Cartesian coordinates.
* [__vx, vy, vz__] are the velocities in each of the previous directions.
* [__phi, theta, gamma__] are the Euler angles rotated through x, y', and z'' in successive order.
* [__phidot, thetadot, gammadot__] are the angular velocities in each of the previous directions.

Rigorious definitions of phi, theta, and gamma can be found in Elizabeth Hannah's paper __[Constraining frisbee tracking methods through Bayesian analysis of flying disc models](https://repository.arizona.edu/handle/10150/626742)__.