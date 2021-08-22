log = load('Efficency test.BIN-2134217.mat');

##alt = -log.NKF1(:,13);
##alt_times = log.NKF1(:,2);
##
##yaw = log.AHR2(:,5);
##yaw_times = log.AHR2(:,2);
##
##south_mask = 180 < yaw & yaw < 192;
##south_times = yaw_times(south_mask);
##
##plot(alt_times, alt, yaw_times, yaw/30);

aspds = log.ARSP(:,[2 3]);
currents = log.BAT(:,[2 6]);

ground_effect_times = [1.041e9 1.086e9; 1.237e9 1.281e9; 1.435e9 1.481e9; 1.635e9 1.682e9; 1.839e9 1.889e9; 2.042e9 2.092e9];
normal_times = [1.14e9 1.175e9; 1.336e9 1.377e9; 1.535e9 1.572e9; 1.74e9 1.777e9; 1.991e9 1.983e9; 2.153e9 2.19e9];

ground_effect_currents = [];
ground_effect_airspeeds = [];
normal_currents = [];
normal_airspeeds = [];

ground_effect_current_mask = zeros(size(currents(:,1)),1);
ground_effect_airspeed_mask = zeros(size(aspds(:,1)),1);
for idx = 1:size(ground_effect_times)(1)
   getimes = ground_effect_times(idx,:);
   ground_effect_current_mask |= getimes(1) < currents(:,1) & currents(:,1) < getimes(2);
   ground_effect_airspeed_mask |= getimes(1) < aspds(:,1) & aspds(:,1) < getimes(2);
end

ground_effect_currents = currents(ground_effect_current_mask,2);
ground_effect_airspeeds = aspds(ground_effect_current_mask,2);

normal_current_mask = zeros(size(currents(:,1)),1);
normal_airspeed_mask = zeros(size(aspds(:,1)),1);
for idx = 1:size(normal_times)(1)
   ntimes = normal_times(idx,:);
   normal_current_mask |= ntimes(1) < currents(:,1) & currents(:,1) < ntimes(2);
   normal_airspeed_mask |= ntimes(1) < aspds(:,1) & aspds(:,1) < ntimes(2);
end

normal_currents = currents(normal_current_mask,2);
normal_airspeeds = aspds(normal_airspeed_mask,2);

##normal_current_mask = normal_times(1,1) < currents(:,1) & currents(:,1) < normal_times(1,2);
##normal_airspeed_mask = normal_times(1,1) < aspds(:,1) & aspds(:,1) < normal_times(1,2);
##
##normal_currents = currents(normal_current_mask,2);
##normal_airspeeds = aspds(normal_current_mask,2);

graphics_toolkit("gnuplot")
hold on;
xlim([0 20])
scatter(ground_effect_currents, ground_effect_airspeeds);
scatter(normal_currents, normal_airspeeds);