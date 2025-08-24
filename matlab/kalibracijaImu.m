% kalibracijaImu.m
close all; clear; clc;

%% Učitaj podatke
acc_file = 'imu9_log.csv';      % mirovanje
motion_file = 'imu9_log2.csv';  % gibanje

raw_rest = readmatrix(acc_file);
raw_motion = readmatrix(motion_file);

assert(size(raw_rest,2) == 10, 'CSV mora imati 10 stupaca.');

acc_data = raw_rest(:,2:4);   % ax, ay, az
mag_data = raw_rest(:,8:10);  % mx, my, mz

%% Kalibracija akcelerometra
% Optimiziramo parametre: A (3x3 gornji trokut), b (3x1 offset)
x0 = [1 0 0 1 0 1  0 0 0];  % A=[1 0 0; 0 1 0; 0 0 1]; b=[0 0 0]
lb = [0.9 -0.5 -0.5 0.9 -0.5 0.9 -1 -1 -1];
ub = [1.1  0.5  0.5 1.1  0.5 1.1  1  1  1];

options = optimoptions('fmincon','Display','iter','MaxIterations', 300, ...
    'MaxFunctionEvaluations', 6000, 'PlotFcn','optimplotfval');

[res, fval] = fmincon(@(x)accError(x, acc_data), x0, [],[],[],[], lb, ub, [], options);

x = res;
A_acc = [x(1) x(2) x(3);
         x(2) x(4) x(5);
         x(3) x(5) x(6)];
b_acc = [x(7) x(8) x(9)];

fprintf('\n=== Akcelerometar kalibracija ===\n');
disp('Matrica A:'); disp(A_acc);
disp('Offset b:'); disp(b_acc);

%% Kalibracija magnetometra
mag_data_swizzled = [mag_data(:,2), mag_data(:,1), -mag_data(:,3)]; % zamjena za NED
[A_mag, b_mag, expMFS] = magcal(mag_data_swizzled);
fprintf('\n=== Magnetometar kalibracija ===\n');
disp('Matrica A:'); disp(A_mag);
disp('Offset b:'); disp(b_mag);

%% Prikaz elipsoida 
xc = (mag_data_swizzled - b_mag) * A_mag';

de = HelperDrawEllipsoid;
de.plotCalibrated(A_mag, b_mag, expMFS, mag_data_swizzled, xc, 'Auto');

%% Yaw računanje na osnovi akcelerometra i magnetometra
% Koristi podatke u kretanju
acc_mot = raw_motion(:,2:4);
mag_mot = raw_motion(:,8:10);
mag_mot_swizzled = [mag_mot(:,2), mag_mot(:,1), -mag_mot(:,3)];

% Kalibracija
acc_corr = (acc_mot - b_acc) * A_acc';
mag_corr = (mag_mot_swizzled - b_mag) * A_mag';

N = size(acc_corr,1);
yaw = zeros(N,1);

for i = 1:N
    fx = acc_corr(i,1);
    fy = acc_corr(i,2);
    fz = acc_corr(i,3);

    roll = atan2(-fy, -fz);
    pitch = atan2(fx, sqrt(fy^2 + fz^2));

    mx = mag_corr(i,1);
    my = mag_corr(i,2);
    mz = mag_corr(i,3);

    % Zadana formula yaw (psi_m)
    nom = -my*cos(pitch) + mz*sin(pitch);
    denom = mx*cos(roll) + my*sin(roll)*sin(pitch) + mz*cos(pitch)*sin(roll);
    yaw(i) = atan2(nom, denom);
end

%% Prikaz yaw kuta
figure;
plot(rad2deg(yaw));
title('Yaw (kompas) nakon kalibracije');
ylabel('Kut [deg]');
xlabel('Vrijeme [uzorci]');
grid on;

