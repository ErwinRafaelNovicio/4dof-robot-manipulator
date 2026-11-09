clear
close all
clc

%% Data 
load('JointAngles_1.mat');


motor(1).A=3;
motor(1).D=3;
motor(1).V=100;
motor(2).A=4;
motor(2).D=4;
motor(2).V=100;
motor(3).A=5;
motor(3).D=5;
motor(3).V=200;

% q1=deg2rad([0,45,100,50,30]);
% q2=deg2rad([0,40,100,180,140]);
% q3=deg2rad([0,30,80,50,5]);
q1=deg2rad(Q(2,:));
q2=deg2rad(Q(3,:));
q3=deg2rad(Q(4,:));

Q=[q1;q2;q3];

dq1=diff(q1);
dq2=diff(q2);
dq3=diff(q3);


dQ=[dq1;dq2;dq3];
dQ_abs=abs(dQ);

%% 0) Minimum actuating time for each displacement
rise_time=zeros(size(dQ_abs,1),size(dQ_abs,2)); % three joints

for i=1:size(dQ_abs,1) %3
    for j=1:size(dQ_abs,2) %number of dots
    rise_time(i,j)=min_rise_time(motor(i),dQ_abs(i,j));
    end
end

T_min_act=max(rise_time);

%% 1) Adding of axiliary points and transition time
t1=zeros(3,1);
tend=zeros(3,1);
for i=1:3
 t1(i,1)=T_min_act(1)-sqrt(T_min_act(1)^2-2*dQ_abs(i,1)/motor(i).A);
 tend(i,1)=T_min_act(end)-sqrt(T_min_act(end)^2-2*dQ_abs(i,end)/motor(i).A);
end

%% 2) Find time for each internal parabolic transition (t_j)
min_act_time_joints=[T_min_act;T_min_act;T_min_act];
min_act_time_joints(:,1)=T_min_act(:,1)-0.5*t1;
min_act_time_joints(:,end)=T_min_act(:,end)-0.5*tend;

q_dot=dQ./min_act_time_joints;
t_j=[];
t_j1=[];
for i=1:3
  t_j1(i,:)=abs(diff(q_dot(i,:),[],2)/motor(i).A);
  t_j(i,:)=[t1(i),t_j1(i,:),tend(i)];
end
%% Time vector
dt_j=zeros(3,length(min_act_time_joints));
for i=1:length(min_act_time_joints)
    dt_j(:,i)=min_act_time_joints(:,i)-0.5*(t_j(:,i)+t_j(:,i+1));
end

dt_j=[[0;0;0],dt_j];



t_vec=zeros(3,length(t_j)+length(dt_j));
for i=1:length(t_j)
    t_vec(:,i*2-1)=dt_j(:,i);
    t_vec(:,i*2)=t_j(:,i);
end

t_vec=cumsum(t_vec,2); % Cumulative sum along the rows


%% 4) Lines coefficients
t_line_vec=cumsum([t1/2,min_act_time_joints],2);

al=q_dot;
bl=Q(:,2:end)-al.*t_line_vec(:,2:end);

%% 5) Parabolas coefficient
q_dot_ext=[[0;0;0],q_dot,[0;0;0]];
Q_end_parabola=[al,al(:,end)].*t_vec(:,2:2:end)+[bl,bl(:,end)];
Q_end_parabola(:,end)=Q(:,end);

ap=diff(q_dot_ext,[],2)./(2*t_j);
bp=[q_dot,[0;0;0]]-ap*2.*t_vec(:,2:2:end);
cp=(Q_end_parabola-ap.*t_vec(:,2:2:end).^2-bp.*t_vec(:,2:2:end));

%% 6) Plot rotation
for j = 1:3
    t_tot.(['t' num2str(j)]) = [];
    y_tot.(['y' num2str(j)]) = [];

    for i = 1:(length(t_vec) - 1)
        t_start = t_vec(j, i);
        t_end = t_vec(j, i + 1);
        t.(['t' num2str(j)]) = linspace(t_start, t_end, 100);

        if mod(i, 2) == 0  % pari
            y.(['y' num2str(j)]) = al(j, i/2) * t.(['t' num2str(j)]) + bl(j, i/2);
        else
            y.(['y' num2str(j)]) = ap(j, floor(i/2 + 1)) .* t.(['t' num2str(j)]).^2 + ...
                                   bp(j, floor(i/2 + 1)) .* t.(['t' num2str(j)]) + ...
                                   cp(j, floor(i/2 + 1));
        end

        if i < length(t_vec) - 1
            t_tot.(['t' num2str(j)]) = [t_tot.(['t' num2str(j)]), t.(['t' num2str(j)])(1:end-1)];
            y_tot.(['y' num2str(j)]) = [y_tot.(['y' num2str(j)]), y.(['y' num2str(j)])(1:end-1)];
        else
            % Ultimo tratto: includi anche il punto finale
            t_tot.(['t' num2str(j)]) = [t_tot.(['t' num2str(j)]), t.(['t' num2str(j)])];
            y_tot.(['y' num2str(j)]) = [y_tot.(['y' num2str(j)]), y.(['y' num2str(j)])];
        end
    end
end

figure('Name', 'Joint_Rotation');
hold on;
for j = 1:3
    plot(t_tot.(['t' num2str(j)]), y_tot.(['y' num2str(j)]), 'LineWidth', 1);
end
title('Joint Rotation');
xlabel('time [sec]');
ylabel('q [rad]');
grid on;
legend({'q1','q2','q3'});

%% Plot spezzata
X1 = [0, t_line_vec(1,:), t_vec(1,end)];
Y1 = [0, q1, q1(1,end)];
X2 = [0, t_line_vec(2,:), t_vec(2,end)];
Y2 = [0, q2, q2(1,end)];
X3 = [0, t_line_vec(3,:), t_vec(3,end)];
Y3 = [0, q3, q3(1,end)];



plot(X1, Y1, 'b-o', 'LineWidth', 0.5, 'MarkerSize', 1, 'MarkerFaceColor', 'b');
plot(X2, Y2, 'b-o', 'LineWidth', 0.5, 'MarkerSize', 1, 'MarkerFaceColor', 'b');
plot(X3, Y3, 'b-o', 'LineWidth', 0.5, 'MarkerSize', 1, 'MarkerFaceColor', 'b');

%% 7) Plot velocity of theta
for j = 1:3
    td_tot.(['t' num2str(j)]) = [];
    yd_tot.(['y' num2str(j)]) = [];

    for i = 1:(length(t_vec) - 1)
        t_start = t_vec(j, i);
        t_end = t_vec(j, i + 1);
        td.(['t' num2str(j)]) = linspace(t_start, t_end, 100);

        if mod(i, 2) == 0  % pari
            yd.(['y' num2str(j)]) = al(j, i/2).*ones(1,length(td.(['t' num2str(j)])));
        else
            yd.(['y' num2str(j)]) = 2*ap(j, floor(i/2 + 1)) .* td.(['t' num2str(j)]) + ...
                                   bp(j, floor(i/2 + 1));
        end

        if i < length(t_vec) - 1
            td_tot.(['t' num2str(j)]) = [td_tot.(['t' num2str(j)]), td.(['t' num2str(j)])(1:end-1)];
            yd_tot.(['y' num2str(j)]) = [yd_tot.(['y' num2str(j)]), yd.(['y' num2str(j)])(1:end-1)];
        else
            % Ultimo tratto: includi anche il punto finale
            td_tot.(['t' num2str(j)]) = [td_tot.(['t' num2str(j)]), td.(['t' num2str(j)])];
            yd_tot.(['y' num2str(j)]) = [yd_tot.(['y' num2str(j)]), yd.(['y' num2str(j)])];
        end
    end
end

figure('Name', 'Joint Velocity');
hold on;
for j = 1:3
    plot(td_tot.(['t' num2str(j)]), yd_tot.(['y' num2str(j)]), 'LineWidth', 1);
end
title('Joint_Velocity');
xlabel('time [sec]');
ylabel('qd [rad/s]');
grid on;
legend({'q1_d','q2_d','q3_d'});


%% 7) Plot acceleration of theta
for j = 1:3
    tdd_tot.(['t' num2str(j)]) = [];
    ydd_tot.(['y' num2str(j)]) = [];

    for i = 1:(length(t_vec) - 1)
        t_start = t_vec(j, i);
        t_end = t_vec(j, i + 1);
        tdd.(['t' num2str(j)]) = linspace(t_start, t_end, 100);

        if mod(i, 2) == 0  % pari
            ydd.(['y' num2str(j)]) = zeros(1,length(tdd.(['t' num2str(j)])));
        else
            ydd.(['y' num2str(j)]) = 2*ap(j, floor(i/2 + 1)).*ones(1,length(tdd.(['t' num2str(j)])));
        end

        if i < length(t_vec) - 1
            tdd_tot.(['t' num2str(j)]) = [tdd_tot.(['t' num2str(j)]), tdd.(['t' num2str(j)])(1:end-1)];
            ydd_tot.(['y' num2str(j)]) = [ydd_tot.(['y' num2str(j)]), ydd.(['y' num2str(j)])(1:end-1)];
        else
            % Ultimo tratto: includi anche il punto finale
            tdd_tot.(['t' num2str(j)]) = [tdd_tot.(['t' num2str(j)]), tdd.(['t' num2str(j)])];
            ydd_tot.(['y' num2str(j)]) = [ydd_tot.(['y' num2str(j)]), ydd.(['y' num2str(j)])];
        end
    end
end

figure('Name', 'Joint Acceleration');
hold on;
for j = 1:3
    plot(tdd_tot.(['t' num2str(j)]), ydd_tot.(['y' num2str(j)]), 'LineWidth', 1);
end
title('Joint Acceleration');
xlabel('time [sec]');
ylabel('qdd [rad/s^2]');
grid on;
legend({'q1_dd','q2_dd','q3_dd'});

%% === Uniform sampling for all joints ===
% Choose a uniform sampling time
Ts = 0.001;  % 1 ms

% Determine total simulation time
T_end = max([t_tot.t1(end), t_tot.t2(end), t_tot.t3(end)]);

% Time vector with uniform sampling
t_uniform = 0:Ts:T_end;

%% Interpolate positions
q1_uniform = interp1(t_tot.t1, y_tot.y1, t_uniform, 'linear');
q2_uniform = interp1(t_tot.t2, y_tot.y2, t_uniform, 'linear');
q3_uniform = interp1(t_tot.t3, y_tot.y3, t_uniform, 'linear');

%% Interpolate velocities
qd1_uniform = interp1(td_tot.t1, yd_tot.y1, t_uniform, 'linear');
qd2_uniform = interp1(td_tot.t2, yd_tot.y2, t_uniform, 'linear');
qd3_uniform = interp1(td_tot.t3, yd_tot.y3, t_uniform, 'linear');

%% Interpolate accelerations
qdd1_uniform = interp1(tdd_tot.t1, ydd_tot.y1, t_uniform, 'linear');
qdd2_uniform = interp1(tdd_tot.t2, ydd_tot.y2, t_uniform, 'linear');
qdd3_uniform = interp1(tdd_tot.t3, ydd_tot.y3, t_uniform, 'linear');

%% Build [time value] matrices
q1_T1 = [t_uniform' q1_uniform'];
q2_T1 = [t_uniform' q2_uniform'];
q3_T1 = [t_uniform' q3_uniform'];

qd1_T1 = [t_uniform' qd1_uniform'];
qd2_T1 = [t_uniform' qd2_uniform'];
qd3_T1 = [t_uniform' qd3_uniform'];

qdd1_T1 = [t_uniform' qdd1_uniform'];
qdd2_T1 = [t_uniform' qdd2_uniform'];
qdd3_T1 = [t_uniform' qdd3_uniform'];

%% Save results
% SIMSCAPE
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\SIMSCAPE\Joint_Positions_T1_ts.mat','q1_T1','q2_T1','q3_T1');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\SIMSCAPE\Joint_Velocities_T1_ts.mat','qd1_T1','qd2_T1','qd3_T1');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\SIMSCAPE\Joint_Accelerations_T1_ts.mat','qdd1_T1','qdd2_T1','qdd3_T1');

% Dynamical Model
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Dynamics\Joint_Positions_T1_ts.mat','q1_T1','q2_T1','q3_T1');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Dynamics\Joint_Velocities_T1_ts.mat','qd1_T1','qd2_T1','qd3_T1');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Dynamics\Joint_Accelerations_T1_ts.mat','qdd1_T1','qdd2_T1','qdd3_T1');

% Direct Method Test
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Motion Planning Test_DK\Joint_Positions_T1_ts.mat','q1_T1','q2_T1','q3_T1');

% Motors adn trasmissions sizing
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Motor_Identification\Joint_Accelerations_T1_ts.mat','qdd1_T1','qdd2_T1','qdd3_T1');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Motor_Identification\Joint_Velocities_T1_ts.mat','qd1_T1','qd2_T1','qd3_T1');

%% === Plot confronto con campionamento uniforme ===
colors = lines(4);
figure('Name','Joint Positions');

% subplot(3,1,1);
hold on; grid on;
plot(t_uniform, q1_uniform, 'Color', colors(2,:), 'LineWidth', 2);
plot(t_uniform, q2_uniform, 'Color', colors(3,:), 'LineWidth', 2);
plot(t_uniform, q3_uniform, 'Color', colors(4,:), 'LineWidth', 2);
title('Joint Positions');
xlabel('Time [s]');
ylabel('q [rad]');
legend('joint 2','joint 3','joint 4');

figure('Name','Joint Velocities');
% subplot(3,1,2); 
hold on; grid on;
plot(t_uniform, qd1_uniform, 'Color', colors(2,:), 'LineWidth', 2);
plot(t_uniform, qd2_uniform, 'Color', colors(3,:), 'LineWidth', 2);
plot(t_uniform, qd3_uniform, 'Color', colors(4,:), 'LineWidth', 2);
title('Joint Velocities');
xlabel('Time [s]');
ylabel('qd [rad/s]');
legend('joint 2','joint 3','joint 4');

figure('Name','Joint Acc');
hold on; grid on;
plot(t_uniform, qdd1_uniform, 'Color', colors(2,:), 'LineWidth', 1.5);
plot(t_uniform, qdd2_uniform, 'Color', colors(3,:), 'LineWidth', 1.5);
plot(t_uniform, qdd3_uniform, 'Color', colors(4,:), 'LineWidth', 1.5);
title('Joint Accelerations');
xlabel('Time [s]');
ylabel('qdd [rad/s^2]');
legend('joint 2','joint 3','joint 4');


%% Utility: assicurati che rad2deg sia disponibile (MATLAB moderno la ha già)
% (se non la trovi, scommenta la riga sottostante)
% rad2deg = @(r) r*180/pi;

%% --- Joint Positions: scala destra in gradi a step di 10°
figure('Name','Joint Positions'); clf; hold on; grid on;
plot(t_uniform, q1_uniform, 'r', 'LineWidth', 1.2);
plot(t_uniform, q2_uniform, 'g', 'LineWidth', 1.2);
plot(t_uniform, q3_uniform, 'b', 'LineWidth', 1.2);
title('Joint Positions');
xlabel('Time [s]');
ylabel('q [rad]');
legend('joint 2','joint 3','joint 4','Location','best');
set(gca,'FontSize',12);

ax = gca;
yyaxis left
% prendi limiti dati e forzali ad includere zero
limL = ax.YLim;
limL(1) = min(limL(1), 0);
limL(2) = max(limL(2), 0);

% converti in gradi e crea tick ogni 10°
deg_min = rad2deg(limL(1));
deg_max = rad2deg(limL(2));
dlow = floor(deg_min/10)*10;
dhigh = ceil(deg_max/10)*10;
deg_ticks = dlow:10:dhigh;
if numel(deg_ticks) < 2
    dlow = dlow - 10;
    dhigh = dhigh + 10;
    deg_ticks = dlow:10:dhigh;
end

% imposta i limiti facendo corrispondere i due assi
left_lim = [dlow, dhigh]*pi/180;   % limiti in radianti corrispondenti ai deg_lim
yyaxis left
set(ax, 'YLim', left_lim, 'YTick', left_lim(1):(left_lim(2)-left_lim(1))/6:left_lim(2));

% imposta destra: ticks ogni 10°, colore nero, label con °
yyaxis right
ax.YAxis(2).Color = 'k';
set(ax, 'YLim', [dlow dhigh], 'YTick', deg_ticks, ...
        'YTickLabel', arrayfun(@(d) sprintf('%d°',d), deg_ticks, 'UniformOutput', false));
ylabel('q [deg]');

% --- Joint Velocities: deg/s ogni 10°
figure('Name','Joint Velocities'); clf; hold on; grid on;
plot(t_uniform, qd1_uniform, 'r', 'LineWidth', 1.2);
plot(t_uniform, qd2_uniform, 'g', 'LineWidth', 1.2);
plot(t_uniform, qd3_uniform, 'b', 'LineWidth', 1.2);
title('Joint Velocities');
xlabel('Time [s]');
ylabel('qd [rad/s]');
legend('joint 2','joint 3','joint 4','Location','best');
set(gca,'FontSize',12);

ax = gca;
yyaxis left
limL = ax.YLim;
limL(1) = min(limL(1), 0);
limL(2) = max(limL(2), 0);

deg_min = rad2deg(limL(1));
deg_max = rad2deg(limL(2));
dlow = floor(deg_min/10)*10;
dhigh = ceil(deg_max/10)*10;
deg_ticks = dlow:10:dhigh;
if numel(deg_ticks) < 2
    dlow = dlow - 10;
    dhigh = dhigh + 10;
    deg_ticks = dlow:10:dhigh;
end

left_lim = [dlow, dhigh]*pi/180;
yyaxis left
set(ax, 'YLim', left_lim, 'YTick', left_lim(1):(left_lim(2)-left_lim(1))/6:left_lim(2));

yyaxis right
ax.YAxis(2).Color = 'k';
set(ax, 'YLim', [dlow dhigh], 'YTick', deg_ticks, ...
        'YTickLabel', arrayfun(@(d) sprintf('%d°/s',d), deg_ticks, 'UniformOutput', false));
ylabel('qd [deg/s]');

% --- Joint Accelerations: deg/s^2 ogni 10°
figure('Name','Joint Acc'); clf; hold on; grid on;
plot(t_uniform, qdd1_uniform, 'r', 'LineWidth', 1.2);
plot(t_uniform, qdd2_uniform, 'g', 'LineWidth', 1.2);
plot(t_uniform, qdd3_uniform, 'b', 'LineWidth', 1.2);
title('Joint Accelerations');
xlabel('Time [s]');
ylabel('qdd [rad/s^2]');
legend('joint 2','joint 3','joint 4','Location','best');
set(gca,'FontSize',12);

ax = gca;
yyaxis left
limL = ax.YLim;
limL(1) = min(limL(1), 0);
limL(2) = max(limL(2), 0);

deg_min = rad2deg(limL(1));
deg_max = rad2deg(limL(2));
dlow = floor(deg_min/10)*10;
dhigh = ceil(deg_max/10)*10;
deg_ticks = dlow:10:dhigh;
if numel(deg_ticks) < 2
    dlow = dlow - 10;
    dhigh = dhigh + 10;
    deg_ticks = dlow:10:dhigh;
end

left_lim = [dlow, dhigh]*pi/180;
yyaxis left
set(ax, 'YLim', left_lim, 'YTick', left_lim(1):(left_lim(2)-left_lim(1))/6:left_lim(2));

yyaxis right
ax.YAxis(2).Color = 'k';
set(ax, 'YLim', [dlow dhigh], 'YTick', deg_ticks, ...
        'YTickLabel', arrayfun(@(d) sprintf('%d°/s^2',d), deg_ticks, 'UniformOutput', false));
ylabel('qdd [deg/s^2]');
