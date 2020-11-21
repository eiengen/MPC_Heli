# MPC_Heli
Laboratory helicopter project (simulator)

Group project

USN subject: Model Predictive Control (IIA4117)
Main teacher: Roshan Sharma

Tasks:

Simulator development
For developing simulator(s), you can work on your computer(laptops) with Simulink/MATLAB. You do
not need access to the real process. The following tasks should be completed:

(1) Describe the 2 DOF helicopter process. E.g. How does the process operate? What are the
inputs, outputs and states of the system? You can re-use the figures/pictures that is available
to you or you can take/make your own figures.

(2) List all the equations involved with the mathematical model of the process. Describe a little
about the mathematical model. E.g. What kind, type? What is its order? linear or nonlinear etc.

(3) Using Simulink, perform the openloop simulation of the nonlinear model of the process.
Manually excite your system with different values of the control inputs and see the response
of the system. Plot the necessary variables of interests (e.g. angles, velocities, control inputs
etc.). Discuss your simulation results.

(4) Choose a suitable operating point and linearize the model of the process about this operating
point. Show all the necessary calculations involved during the linearization of the mathematical
model.
To linearize your model, first choose a suitable operating point for the four states (for e.g.
(−10𝜋/180,𝜋/2,0,0) for pitch angle, yaw angle, pitch velocity and yaw velocity respectively). To find
the corresponding operating point for the control inputs, solve the nonlinear process model at
the steady state for 𝑉𝑚𝑝 and 𝑉𝑚𝑦 using the operating point for the states, i.e. set the left hand
side of equations (1) to (4) to zero solve these equations (by using the operating point of the
states) for calculating operating point for control input voltages.

(5) Using Simulink, simulate the linear model alongside the nonlinear model. Compare the
behavior of the process from these two models. Comment and discuss about your simulation
results.

(6) Assume that you can measure only the two angles (the pitch angle (𝜃) and the yaw angle (𝜓)).
Design a suitable state estimator in Simulink for estimating the remaining two states (the pitch
velocity (𝜔𝜃) and the yaw velocity (𝜔𝜓)). Here you are not limited to using steady state Kalman
filter but you can use any other state estimator that you know if you want. Change the control 
inputs manually and observe the behavior of your state estimator. Discuss and comment in
detail about your simulation results.

(7) Design an output feedback linear MPC for controlling the pitch angle (𝜃) and the yaw angle (𝜓)
to their respective setpoints. At this point, do not worry about integral action, however, please
do consider the following:

    a. Your controller should not supply more than 3[V] as the control inputs. They cannot be less than 0 [V] either.  sing signal limiter block in Simulink is NOT ENOUGH. You should consider this while formulating your optimal control problem.

    b. You should show the MPC problem formulation very clearly. Show the objective function, constraints, bounds etc. Show your choice of the unknown vectors to optimize. 

    c. You should show every detailed step of how you calculated/formulated all the necessary matrices and vectors. How did you handle the bounds (if any?)

    d. How did you implement the receeding horizon strategy in Simulink? Show clearly. 

    e. The linear MPC should be applied to the helicopter process. For the simulator, consider the nonlinear model of the helicopter as your process. Please DO NOT use the linear model as your process. You can of course make use of  hings that you already did in task (iii) for the nonlinear model of the process.

    f. Provide step changes to the setpoint values and observe the response of your controller. Change your setpoint as ramp (for e.g. using slider or knob in Simulink) and observe the response of your controller.

    g. Plot all the necessary variables (outputs, estimated states, inputs, references etc.)

    h. Discuss in details all your simulation results.

    i. Among many other things, did you observe any steady-state offsets with your linear MPC?

(8) Design an output feedback linear MPC with integral action. Consider the points (vii) a, e, f and
g while designing the controller. How can you ensure integral action with your linear MPC? If
you know many ways to achieve integral action, just describe the one that you have used in
your project. Discuss in detail your simulation results.