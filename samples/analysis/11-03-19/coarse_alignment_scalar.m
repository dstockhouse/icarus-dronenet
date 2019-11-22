function [roll, pitch, yaw, C_n__b] = coarse_alignment_scalar(gyro, accel)

fx = accel(1);
fy = accel(2);
fz = accel(3);

wx = gyro(1);
wy = gyro(2);
wz = gyro(3);

% From Dr. Bruder's EE 440 Lecture 21
roll = atan2(-fy, -fz);
pitch = atan(fx / sqrt(fy^2 + fz^2));
yaw = atan2(-wy*cos(roll) + wz*sin(roll), wx*cos(pitch) + (wy*sin(roll) + wz*cos(roll))*sin(pitch));

% Dr. Bruder's provided function
C_n__b = ypr2dcm(yaw, pitch, roll);

end