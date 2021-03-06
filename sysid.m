% Use mission planner to convert dataflash .bin to mat files and then import here
log = load('00000095.BIN-1308477.mat');

start_time = 511000000;
end_time   = 523000000;

% get the data and select the range we want
time_rfnd_select = start_time < log.RFND(:,2) & log.RFND(:,2) < end_time;
times_rfnd = log.RFND(:,2)(time_rfnd_select);
rangefinder = log.RFND(:,4)(time_rfnd_select);

times_ctun_select = start_time < log.CTUN(:,2) & log.CTUN(:,2) < end_time;
times_thr = log.CTUN(:,2)(times_ctun_select);
throttle = log.CTUN(:,7)(times_ctun_select);
airspeed_sensor = log.CTUN(:,10)(times_ctun_select);
airspeed_synthetic = log.CTUN(:,11)(times_ctun_select);

times_nkf1_select = start_time < log.NKF1(:,2) & log.NKF1(:,2) < end_time;
times_nkf = log.NKF1(:,2)(times_nkf1_select);
climb_rate = log.NKF1(:,10)(times_nkf1_select);

%rangefinder gets logged at double the rate, so stretch out the others by duplicating
times_thr = repelem(times_thr, 2);
throttle = repelem(throttle, 2);
airspeed_sensor = repelem(airspeed_sensor, 2);
airspeed_synthetic = repelem(airspeed_synthetic, 2);

h = plot(times_rfnd, [rangefinder, throttle/50, airspeed_sensor/3-3.5, climb_rate+0.7]);
set(h,'LineWidth',3);
set(h,{'DisplayName'},{'altitude';'throttle';'airspeed';'climbRate'});
legend show;


data = iddata([airspeed_sensor, climb_rate, rangefinder], throttle);

[sysd, x0, info] = moen4(data, 3);
sysd.a
sysd.b

sysc = d2c(sysd)

sysc.a
sysc.b

pause;

% % simulate 5 seconds of 100% throttle
% time = 0:0.01:5; % time samples
% r = 100*ones(size(time)); % system input
% x0 = [0.5 40]; % initial state
% [y, t, x] = lsim(sys, r, time);
% % plotyy(t, y(:,1), t, y(:,2), 'plot')
% plot(t, y(:,1));



% % q = [100 0; 0 100]
% % r = [0.01 0; 0 0.01]

% % [K, x, l] = lqr(sys, q, r)

% % K

% % simulate 10 seconds
% % t = 0:0.01:10 % time samples
% % r = zeroes(size(t)) % system input
% % x0 = [0 0 0] % initial state
% % [y, t, x] = lsim(sys, r, t)
% % plotyy(t, y(:,1), t, y(:,2), 'plot')

% pause;