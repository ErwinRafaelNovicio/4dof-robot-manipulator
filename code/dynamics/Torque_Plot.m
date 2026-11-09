%% Load torque data
T1 = load('Torque_1_T1.mat');
T2 = load('Torque_2_T1.mat');
T3 = load('Torque_3_T1.mat');
T4 = load('Torque_4_T1.mat');

T12 = load('Torque_1_T2.mat');
T22 = load('Torque_2_T2.mat');
T32 = load('Torque_3_T2.mat');
T42 = load('Torque_4_T2.mat');

%% Estraggo tempo e torque
time = T1.Torque_i.time;   % tempo originale T1
TJ1  = T1.Torque_i.values;
TJ2  = T2.Torque_i.values;
TJ3  = T3.Torque_i.values;
TJ4  = T4.Torque_i.values;

time2 = T12.Torque_i.time;   % tempo originale T2
TJ12  = T12.Torque_i.values;
TJ22  = T22.Torque_i.values;
TJ32  = T32.Torque_i.values;
TJ42  = T42.Torque_i.values;

%% Creazione tempo esteso (raddoppiato)
dt = time(2) - time(1);
time_ext = [time; time(end) + dt + time];   % seconda metà traslata

dt2 = time2(2) - time2(1);
time_ext2 = [time2; time2(end) + dt2 + time2];   % seconda metà traslata

%% Creazione torque specchiato (ritorno)
TJ1_ext = [TJ1; TJ1(end:-1:1)];
TJ2_ext = [TJ2; TJ2(end:-1:1)];
TJ3_ext = [TJ3; TJ3(end:-1:1)];
TJ4_ext = [TJ4; TJ4(end:-1:1)];

TJ12_ext = [TJ12; TJ12(end:-1:1)];
TJ22_ext = [TJ22; TJ22(end:-1:1)];
TJ32_ext = [TJ32; TJ32(end:-1:1)];
TJ42_ext = [TJ42; TJ42(end:-1:1)];
%% Plot T1
figure;
hold on;
grid on;
grid minor;   % griglia più fitta

colors = lines(4);
labels = {'Joint 1','Joint 2','Joint 3','Joint 4'};

plot(time_ext, TJ1_ext, 'Color', colors(1,:), 'DisplayName', labels{1});
plot(time_ext, TJ2_ext, 'Color', colors(2,:), 'DisplayName', labels{2});
plot(time_ext, TJ3_ext, 'Color', colors(3,:), 'DisplayName', labels{3});
plot(time_ext, TJ4_ext, 'Color', colors(4,:), 'DisplayName', labels{4});

xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Joint Torques - Trajectory 1 (Lines and Parabolas)');
legend show;

% Imposto yticks più fitti
yl = ylim;                  % prendo limiti attuali
step_y = 2;                 % modifica lo step come vuoi 
yticks(yl(1):step_y:yl(2)); % set personalizzato

hold off;

%% Plot T2
figure;
hold on;
grid on;
grid minor;   % griglia più fitta

colors = lines(4);
labels = {'Joint 1','Joint 2','Joint 3','Joint 4'};

plot(time_ext2, TJ12_ext, 'Color', colors(1,:), 'DisplayName', labels{1});
plot(time_ext2, TJ22_ext, 'Color', colors(2,:), 'DisplayName', labels{2});
plot(time_ext2, TJ32_ext, 'Color', colors(3,:), 'DisplayName', labels{3});
plot(time_ext2, TJ42_ext, 'Color', colors(4,:), 'DisplayName', labels{4});

xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Joint Torques - Trajectory 2 (Cubic Spline)');
legend show;

% Imposto yticks più fitti
yl = ylim;                  % prendo limiti attuali
step_y = 2;                 % modifica lo step come vuoi 
yticks(yl(1):step_y:yl(2)); % set personalizzato

hold off;

