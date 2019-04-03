# simulated-data
Code for running simulated trajectories with the Motion Calibration Method for AER 1514

Note that for code to work properly, set your home directory to "...path.../simulated-data"

To run the algorithm, run the "calibration.m" file. This file simply calls the following functions:
1. "test.m" - generates a simulated trajectory
2. "DataPro.m" converts the data into a format usable by the algorithm
3. "Veldy_to_IMU.m" - the algorithm that calculates the transformation between the sensors
4. "Plot_1_Two_Sensors_sim.m" - plots the errors of the solutions for the motion calibration method as well as a WLS method.

Notes:
1. The noise of the simulated trajectories can be changed in the "test.m" file. Simply change the values for:
- sigma_tra (translational covariance matrix)
- sigma_rot (rotational covariance matrix)

Additionally, to change the form of the randomly generated trajectory we can change a couple of the parameters used in the code:

In "Create3DSimulatedData.m":
- N_interp -- the fidelity of the surface generated.
- N_modes -- This controls the amount of sinusoids used to generate the surface which the trajectory is based on.
- rad -- the radius of the trajectory.
- amp_range -- the range in amplitude of the surface.
- freq_range -- the frequecy range of the sinusoids used to generate the sruface.

In "random_smooth_trj.m":
- x and y -- these are set to create a circular trajectory in x and y by default. Feel free to play around.
