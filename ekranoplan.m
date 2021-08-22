%% System Identification

% A state Space model is defined by the following equations:

% x_dot = Ax + Bu
% y_dot = Cx + Du

% A and B are the ones that really matter. A has size [states, states] and is called the system matrix.
% This describes how the system changes with no control input. B has size [states, inputs], and describes how the input affects the system state.
% The input to the system is the thing that you can control. Let’s say there’s just one input - throttle.
% The states are everything that’s relevant, including the output(s) that you’re trying to control.

states = {'airspeed', 'climbRate', 'altitude'};
inputs = {'desiredPosition'};

% Let’s start by assuming that the pitch of the vehicle is fixed at 0. Makes the problem a lot simpler.
% To make this true, we can have ardupilot use a simple pid controller to keep pitch at zero.
% First up, let's find the reference state. After manually keeping the vehicle in ground effect in fbwa mode,
% it seemed that the vehicle was pretty stable at 0.46m altitude, at 13m/s using around 30% throttle.
% See manual_ground_effect_flight.png

x_ref = [13;    % airspeed in m/s
         0;     % climb rate in m/s
         0.46]; % altitude in m

u_ref = 30; % throttle in percent (between 0 and 100). I'm actually going to find this number in a different way below.

% Now let's describe the physics of the system to derive the A and B matrices:

% altitude_dot = climb_rate (by definiton: acceleration is the derivative of velocity)
%    -> This results in a 1 in the (3,2) position in the A matrix

% F_vertical = lift - mass * gravity (sum of forces)
% Acc_vertical = lift/m - gravity (divide previous equation by mass)
% Lift = (k_density * k_wing_area * k_lift_coeff)*airspeed^2 (from textbook)
% Acc_vertical = (k_density * k_wing_area * k_lift_coeff)*airspeed^2 - gravity (combine previous 2 equations)
% Acc_vertical = k1*airspeed (Linearize, remove gravity and fold constants into k1. This simplification is mathematically wrong, but necessary.
%                             We'll make sure to find a k1 that is close to correct near 13m/s.)
% climbRate_dot = Acc_vertical (by definition: acceleration is the derivative of velocity)
% climbRate_dot = k1*airspeed
%    -> This results in a k1 in the (2,1) position of the A matrix (TODO think about this one again)

% throttle * k_propeller_coefficient  = mass * forward_acceleration (from f=ma. Throttle creates a force.)
% airspeed_dot = forward_acceleration (by definition: acceleration is the derivative of velocity)
% airspeed_dot = k2*throttle (propellor coefficient and mass were folded into one constant, k2)
%    -> This results in a k2 in the (1,1) position of the B matrix

% Now let's find those 2 magic constants by flying the vehicle and looking at the logs:

% k1 ----
% Let's start with this formula: Acc_vertical = (k_density * k_wing_area * k_lift_coeff)*airspeed^2 - gravity
% Since we know that we are in straight-and-level flight at 13m/s, let's write 0 = (k_density * k_wing_area * k_lift_coeff)*13^2 - 9.8
% solve: (k_density * k_wing_area * k_lift_coeff) = 0.058
% now we have the formula f(x)=0.058x^2-9.8
% k1 is the slope of this formula at x=13: k1 = f'(13)
% k1 = 0.058*2*13 = 1.5

k1 = 1.5;

% k2 ----
% Here is the formula we are using: airspeed_dot = k2*throttle - u_ref
% We collected some data in ground effect mode and then added more throttle to watch the acceleration. Looked at CTUN.ThrOut vs IMU.AccX in the logs when climb rate and pitch were zero.
% The data points look like "36% throttle causes 1.0m/s acceleration". Now just running linear regression to solve for k2 (and also u_ref because we can)
experiment2_acceleration = [0.0 0.0 0.8 1.0 3.4 3.0 5.0];
experiment2_throttle =     [27  33  33  36  60  62  65];
linreg = polyfit(experiment2_acceleration, experiment2_throttle, 1);
k2 = linreg(1)
u_ref = linreg(2)

% So here's our A and B from this description:

A = [0  0  0;
     k1 0  0;
     0  1  0];

B = [k2;
     0;
     0];

% Now define the state-space system!
C = [0 0 0; 1 0 0; 0 1 0; 0 0 1]; % System outputs ?, airspeed, climbRate, altitude
D = [0;0;0;0]; % no feedforward
sys = ss(A,B,C,D); % define the system model

% Verify that the system is controllable
assert(rank(obsv(sys))==size(A)(1), "System is not controllable!");

% Calculate the controller gains
Q = [1 0 0;
     0 1 0;
     0 0 10];
R=10
K=lqr(sys, Q, R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Cc(1,:)=-K; % System output 1 is throttle
Dc = [D];

outputs = {'throttle', 'airspeed', 'climbRate', 'altitude'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r = 0.0*ones(size(t));
x0 = [3 0 0.5]; % initial state. A bit fast and a bit high
lsim(sys_cl,r,t,x0);
pause
