%% System Identification

% A state Space model is defined by the following equations:

% x_dot = Ax + Bu
% y_dot = Cx + Du

% A and B are the ones that matter. A has size [states, states] and is called the system matrix.
% This describes how the system changes with no control input. B has size [states, inputs], and describes how the input affects the system state.

% The input to the system is the thing that you can control. Let’s say there’s just one input - throttle.
% The states are everything that’s relevant, including the output that you’re trying to control. And the states should be considered errors.
% In other words, the controller will drive the state to [0,0,0,...], so define states such that 0 is the target.

% Let’s start by assuming that the pitch of the vehicle is fixed at 0. Makes the problem a lot simpler.
% Now imagine the vehicle in straight-and-level ground effect flight with some constant throttle.
% Increasing the throttle will cause us to accelerate, and decreasing it will cause us to decelerate.
% Let’s say the ratio of throttle change to acceleration change is k1. We are assuming that this relationship is linear even if it isn’t,
% ie we are linearizing. We also know that forward acceleration is the derivative of airspeed. Increasing our airspeed would increase our lift.
% Lift is proportional to climb rate. And climb rate is the derivative of altitude.

% To find those magic numbers, we fly the vehicle in some particular ways and look at the logs.

k1 = 123; % TODO What is the ratio of change in airspeed to change in climb rate? Plot CTUN.Arspd and NKF1.dPD. Pitch must remain zero, so maybe collect data in fbwa.
k2 = 123; % TODO What is the ratio of change in throttle to change in acceleration? Plot CTUN.ThrOut and IMU.AccX. Climb rate must be zero, so maybe collect data in fbwb.

% So here's our A and B from this description:

A = [0 0  0 0;
     1 0  0 0;
     0 k1 0 0;
     0 0  1 0];

B = [k2;
     0;
     0;
     0];

% We also gotta choose a reference state. Since we want straight-and-level flight in ground effect, we want to acceleration and no vertical velocity.
% Looking at some logs from manual flight in ground effect, it looks like the vehicle was already pretty stable around 12m/s velocity at 0.6m alt with 65% throttle.

x_ref = [0;
         12;
         0;
         0.6];

u_ref = 65;

% Now define the state-space system!

C = [0 0 0 1] % Select the output of the system as the last state, altitude

D = [0] % no feedforward

system = ss(A,B,C,D)

% TODO discretize the system and explain why

% Sanity check: start with zero error and a constant input of u_ref. The system should not diverge from steady state

% TODO

% Plot what happens if we start at an altitude of 0.5m above the reference and give a constant input of u_ref

% TODO define a Q and R and then run dlqr. plot it

% Then read this
% https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlDigital

% am I confused about the whole ref thing?
