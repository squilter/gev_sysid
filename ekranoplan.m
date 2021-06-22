%% System Identification

% A state Space model is defined by the following equations:

% x_dot = Ax + Bu
% y_dot = Cx + Du

% A and B are the ones that really matter. A has size [states, states] and is called the system matrix.
% This describes how the system changes with no control input. B has size [states, inputs], and describes how the input affects the system state.
% The input to the system is the thing that you can control. Let’s say there’s just one input - throttle.
% The states are everything that’s relevant, including the output(s) that you’re trying to control.

states = {'airspeed', 'climbRate', 'altitude'};
inputs = {'throttle'};

% Let’s start by assuming that the pitch of the vehicle is fixed at 0. Makes the problem a lot simpler.
% To make this true, we can have ardupilot use a simple pid controller to keep pitch at zero.
% First up, let's find the reference state. After manually keeping the vehicle in ground effect in fbwa mode,
% it seemed that the vehicle was pretty stable at 0.6m altitude, at 12m/s using around 65% throttle.

x_ref = [12;
         0;
         0.6];

u_ref = 65;

% altitude_dot = climb_rate (by definiton: acceleration is the derivative of velocity)
%    -> This results in a 1 in the (3,2) position in the A matrix

% F_vertical = lift - mass * gravity (sum of forces)
% Acc_vertical = lift/m - gravity (divide previous equation by mass)
% Lift = (k_density * k_wing_area * k_lift_coeff)*airspeed^2 (from textbook)
% Acc_vertical = k1*airspeed^2 - gravity (combine previous 2 equations, fold all constants into k1)
% Acc_vertical = k1*airspeed (Linearize and remove gravity. This simplification is mathematically wrong, but necessary.
%                             We'll make sure to find a k1 that is close to correct near 12m/s.)
% climbRate_dot = Acc_vertical (by definition: acceleration is the derivative of velocity)
% climbRate_dot = k1*airspeed
%    -> This results in a k1 in the (2,1) position of the A matrix (TODO think about this one again)

% throttle * k_propeller_coefficient  = mass * forward_acceleration (from f=ma. Throttle creates a force.)
% airspeed_dot = forward_acceleration (by definition: acceleration is the derivative of velocity)
% airspeed_dot = k2*throttle (propellor coefficient and mass were folded into one constant, k2)
%    -> This results in a k2 in the (1,1) position of the B matrix

% Now let's find those 2 magic constants by flying the vehicle and looking at the logs.

k1 = 123; % TODO What is the ratio of change in airspeed to change in climb rate? Plot CTUN.Arspd and NKF1.dPD. Pitch must remain zero, so maybe collect data in fbwa.
k2 = 123; % TODO What is the ratio of change in throttle to change in acceleration? Plot CTUN.ThrOut and IMU.AccX. Climb rate must be zero, so maybe collect data in fbwb.

% So here's our A and B from this description:

A = [0  0  0;
     k1 0  0;
     0  1  0];

B = [k2;
     0;
     0];

% Now define the state-space system!
C = [0 0 1]; % Select the output of the system as the last state, altitude
D = [0]; % no feedforward
sys = ss(A,B,C,D); % define the system model!

% Verify that the system is controllable
assert(rank(obsv(sys))==size(A)(1), "System is not controllable!");


% Show the altitude of the vehicle if 100% throttle is applied for 5 seconds
time = 0:0.01:5; % time samples
r = 100*ones(size(time)); % system input
x0 = [0 0 0]; % initial state
[y, t, x] = lsim(sys, r, time);
plot(t, y(:,1));

% Calculate the controller gains
lqr(sys, eye(3), 1)

pause

% TODO

% Plot what happens if we start at an altitude of 0.5m above the reference and give a constant input of u_ref

% TODO define a Q and R and then run dlqr. plot it

% Then read this
% https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlDigital
