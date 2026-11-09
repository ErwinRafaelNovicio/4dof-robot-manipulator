close all
clear
clc

%% Loading
load('Joint_Positions_T2_ts.mat');
load('Joint_Velocities_T2_ts.mat');
load('Joint_Accelerations_T2_ts.mat');

%% CAD Parameters
L = [0.5; 0.4; 0.3; 0.17];                    
G = [L(1)/2; 0.115; 0.0664; 0.0097];            
m = 0.5; m1 = 13.63; m2 = 4.48; m3 = 2.127; m4 = 0.4194; 

J_1 = 0.21;
J_4 = 0.0007;
J_3 = 0.0313;
J_2 = 0.1585;

M_vec = [m,m,m,m4,m4,m4,J_4,m3,m3,m3,J_3,m2,m2,m2,J_2,J_1];
M = diag(M_vec);

F_ext = zeros(16,1);
F_ext([3 6 10 14]) = -[m m4 m3 m2]*9.81;

%% === Preallocazione coppie ===
n_points = length(q1_T2);
Torque = zeros(4,n_points); 


%% Rotation, velocity and acceleration definitions

Q   = [q1_T2(:,2), q2_T2(:,2), q3_T2(:,2), q4_T2(:,2)]'; 
Qd  = [qd1_T2(:,2), qd2_T2(:,2), qd3_T2(:,2), qd4_T2(:,2)]';
Qdd = [qdd1_T2(:,2), qdd2_T2(:,2), qdd3_T2(:,2), qdd4_T2(:,2)]';


%% Compute Torque
Se_dd=zeros(16,n_points);
Fsi=zeros(16,1);
Fs=zeros(16,1);
Fcq=zeros(4,1);


%% J and Jd
J=zeros(16,4,n_points);
J_d=zeros(16,4,n_points);

for k = 1:n_points
    J(:,:,k) = Jacobian_dyn(Q(:,k), L, G);
    J_d(:,:,k) = Jacobian_d_dyn(Q(:,k),Qd(:,k),L,G);
    fprintf('Iterazione: %d\n', k)

end


for k = 1:n_points

    % J   = Jacobian_dyn(Q(:,k),L,G);
    % J_d = Jacobian_d_dyn(Q(:,k),Qd(:,k),L,G);

    Se_dd(:,k) = J_d(:,:,k)*Qd(:,k) + J(:,:,k)*Qdd(:,k);
    Fsi(:,k)   = -M * Se_dd(:,k);
    Fs(:,k)    = F_ext + Fsi(:,k);
    Fcq(:,k)   = -J(:,:,k)' * Fs(:,k);

    Torque(:,k) = Fcq(:,k);
end


%% Torque Plot
figure;
hold on;
grid on;

colors = lines(4);  % 4 colori diversi
labels = {'Joint 1','Joint 2','Joint 3','Joint 4'};

time = q1_T2(:,1);

% Prealloca gli oggetti linea
h = gobjects(4,1);
for j = 1:4
    h(j) = plot(NaN, NaN, 'Color', colors(j,:), 'LineWidth', 1.5);
end

xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Joint Torques');
legend(labels);

for k = 1:5:length(time)
    for j = 1:4
        set(h(j), 'XData', time(1:k), 'YData', Torque(j,1:k));
    end
    drawnow;
end

%% Salvataggio torque T2 per singolo giunto

for i = 1:4
    
    Torque_i.time   = time;
    Torque_i.values = Torque(i,:)';    % vettore colonna
    
    filename = sprintf('Torque_%d_T2.mat', i);
    save(filename, 'Torque_i');
    
end

disp('Salvati: Torque_1_T2.mat ... Torque_4_T2.mat');