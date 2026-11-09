close all
clear all
clc

%% Data
motor.A=3; % Acceleration [rad/s^2]
motor.D=3; % Deceleration [rad/s^2]
motor.V=4; % Max Velocity [m/s]

Q = deg2rad([0, 90, 120, 180, 200, 240;
    0, 30, 90, 120, 180, 190; ...
    0, 20, 30, 50, 54, 10]);    % 3 joints

dQ = diff(Q, [], 2);
dQ=abs(dQ);          %Needed to avoid imaginary part in raising time

%% 1) Limit case
[T_lim,dq_lim]=limit_case(motor);

%% 2) Case 1 or case 2?
is_case1 = dQ < dq_lim; % we are in the case 1 (look in the workspace) - Boolean Variable

%% 3) Calculate the rise time
rise_time=zeros(size(dQ,1),size(dQ,2));

for i=1:size(dQ,1)
    for j=1:size(dQ,2)
    rise_time(i,j) = min_rise_time(motor,dQ(i,j));
    end
end

%% 4) Minimum actuating time
T_min_act=zeros(1,size(dQ,2));

for i=1:size(dQ,2)
    T_min_act(i)=max(rise_time(:,i));   % time needed by the robot to perform all the motion
end

% Until now it's correct

%% 5) New max speed
V_hat = zeros(size(dQ,1),size(dQ,2));   %zeros(1,length(dQ));
T_A=V_hat/motor.A;
T_D=V_hat/motor.D;

for i=1:length(dQ)
    [V_hat(i),T_A(i),T_D(i)] = trapezoidal(motor,dQ(i),T_min_act);
end

plot_trapezoidal(V_hat,T_A,T_D,T_min_act,motor);