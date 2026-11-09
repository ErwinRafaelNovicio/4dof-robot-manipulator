close all
clear
clc

%% Loading
load('Joint_Positions_T2_ts.mat');
load('Joint_Velocities_T2_ts.mat');
load('Joint_Accelerations_T2_ts.mat');

%% CAD Parameters
L = [0.5; 0.4; 0.3; 0.17];                    
G = [L(1)/2; 0.116; 0.0713; 0.0164];            
m = 0.5; m1 = 13.7; m2 = 4.5079; m3 = 2.1758; m4 = 0.4462; 

J_1 = 0.22;
J_4 = 0.0011;
J_3 = 0.0349;
J_2 = 0.1602;

M_vec = [m,m,m,m4,m4,m4,J_4,m3,m3,m3,J_3,m2,m2,m2,J_2,J_1];
M = diag(M_vec);

F_ext = zeros(16,1);
F_ext([3 6 10 14]) = -[m m4 m3 m2]*9.81;


%% === Preallocazione coppie ===
n_points = length(q1_T2);
Torque = zeros(4,n_points); 


%% Rotation, velocity and acceleration definitions
q0=zeros(length(q1_T2),1);
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
    J(:,:,k) = Jacobian_num(Q(:,k), L, G,1e-4);
    J_d(:,:,k) = Jacobian_d_num(Q(:,k),Qd(:,k),L,G,1e-4, 1e-4);
end


for k = 1:n_points

    % J   = Jacobian_dyn(Q(:,k),L,G);
    % J_d = Jacobian_d_dyn(Q(:,k),Qd(:,k),L,G);

    Se_dd(:,k) = J_d(:,:,k)*Qd(:,k) + J(:,:,k)*Qdd(:,k);
    Fsi(:,k)   = -M * Se_dd(:,k);
    Fs(:,k)    = F_ext + Fsi(:,k);
    Fcq(:,k)   = -J(:,:,k)' * Fs(:,k);

    Torque(:,k) = Fcq(:,k);
    fprintf('Iterazione: %d\n', k)
end


%% Torque Plot
figure;
hold on;
grid on;
grid minor;

colors = lines(4);  % 4 colori diversi
labels = {'Joint 1','Joint 2','Joint 3','Joint 4'};

time = q1_T2(:,1);

% Prealloca gli oggetti linea
h = gobjects(4,1);
for j = 1:4
    h(j) = plot(NaN, NaN, 'Color', colors(j,:), 'LineWidth', 2);
end

xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Joint Torques');
legend(labels);

for k = 1:10:length(time)
    for j = 1:4
        set(h(j), 'XData', time(1:k), 'YData', Torque(j,1:k));
    end
    % drawnow;
end


%% Salvataggio torque T1 per singolo giunto

for i = 1:4

    Torque_i.time   = time;
    Torque_i.values = Torque(i,:)';    % vettore colonna

    filename = sprintf('Torque_%d_T2.mat', i);
    save(filename, 'Torque_i');

end

disp('Salvati: Torque_1_T2.mat ... Torque_4_T2.mat');

%% Plot confronto simulazione

load("Torque_SIM_T2.mat");

T1S=C_S_T2(:,1);
T2S=C_S_T2(:,2);
T3S=C_S_T2(:,3);
T4S=C_S_T2(:,4);

T1N=Torque(1,:)';
T2N=Torque(2,:)';
T3N=Torque(3,:)';
T4N=Torque(4,:)';

time = q1_T2(:,1);

% --- Assumo che tu abbia già caricato i dati come nel tuo snippet ---
% T1S=T1S; ... T4N=T4N; time = q1_T1(:,1);

% Metti i dati in matrici (colonne = giunti 1..4)
Tsim = [T1S, T2S, T3S, T4S];   % tratteggiati
Tnom = [T1N, T2N, T3N, T4N];   % continui

% Assicuriamoci che time e vettori abbiano la stessa lunghezza (trim se necessario)
nT = min([length(time), size(Tsim,1), size(Tnom,1)]);
time = time(1:nT);
Tsim = Tsim(1:nT, :);
Tnom = Tnom(1:nT, :);

% Preparazione plot
figure;
hold on;
grid on;
grid minor;
colors = lines(4);        % 4 colori distinti
lw_nom = 1;             % linewidth per le curve continue
lw_sim = 2;             % linewidth per le curve tratteggiate

legendEntries = cell(1, 8);

% Plot: per ogni giunto prima la curva nominale (continua) poi la simulata (tratteggiata)
for i = 1:4
    % curva continua (TiN)
    plot(time, Tnom(:,i), '-', 'Color', colors(i,:), 'LineWidth', lw_nom);
    legendEntries{2*i-1} = sprintf('T%d MATLAB', i);
    
    % curva tratteggiata (TiS) con lo stesso colore
    plot(time, Tsim(:,i), '--', 'Color', colors(i,:), 'LineWidth', lw_sim);
    legendEntries{2*i} = sprintf('T%d SIMULINK', i);
end

% Etiquette del grafico
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Joint Torques: MATLAB(continuous) vs SIMSCAPE (dashed)');
legend(legendEntries, 'Location', 'best');
axis tight;
hold off;
