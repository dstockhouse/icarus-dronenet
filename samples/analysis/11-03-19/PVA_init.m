function [P_init, V_init, A_init] = PVA_init(quiescent_gyro, quiescent_accel, quiescent_compass)
% INPUT: Quiescent accel, gyro, compass data
% Assumes quescent measurements are completely stationary 

% Perform coarse self-alignment
fx = mean(quiescent_accel(1,:));
fy = mean(quiescent_accel(2,:));
fz = mean(quiescent_accel(3,:));

roll = atan2(-fy, -fz);
pitch = atan(fx/sqrt(fy^2 + fz^2));


% Compute gyro bias
b_gFB = 


end
