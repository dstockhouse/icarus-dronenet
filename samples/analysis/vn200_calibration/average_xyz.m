
load_constants;

x_vals = readmatrix('point17_x.log');
y_vals = readmatrix('point17_y.log');
z_vals = readmatrix('point17_z.log');

x = mean(x_vals);
y = mean(y_vals);
z = mean(z_vals);

[L_b, lambda_b, h_b] = xyz2llh([x, y, z], constants);

fprintf('Lat:    %.6f\n', L_b*180/pi);
fprintf('Lon:    %.6f\n', lambda_b*180/pi);
fprintf('Height: %d\n', h_b);
