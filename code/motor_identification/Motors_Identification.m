clear
clc
close all

%% Import data
load("Torque_SIM_T1.mat")
load("Torque_SIM_T2.mat")
load("Joint_Accelerations_T1_ts.mat")
load("Joint_Accelerations_T2_ts.mat")
load("Joint_Velocities_T1_ts.mat")
load("Joint_Velocities_T2_ts.mat")
 
Jmas=1.445*1e-2;                                               %kg·m²
Jm=[0.109 0.109 0.109 0.0362]*1e-4;                            %kg·m²
C_n=[0.64 0.64 0.64 0.32];                                     %Nm
Jr=[0.34 0.24+Jmas 0.041+Jmas 0.0011+Jmas];               %kg·m² 
Qpm_max=628;

alpha=zeros(4,2);
beta=zeros(4,2);

%Ts_SIM 0.005
%Ts_Dyn 0.01

T_opt=zeros(4,2);
T_max=zeros(4,2);
T_min=zeros(4,2);
T_vel=zeros(4,2);
Qpp_rms=zeros(4,2);
Cr_rms=zeros(4,2);


for i=1:4           %N° motors T1
    Cr_rms(i,1) = sqrt(mean(C_S_T1(:,i).^2));
end

for i=1:4           %N° motors T2
    Cr_rms(i,2) = sqrt(mean(C_S_T2(:,i).^2));
end
Cr_rms(1,1)=NaN;
% Cr_rms=Cr_rms./2;

% FOR SPRING USE THIS
% Cr_rms(2,1)=6.17;
% Cr_rms(2,2)=9.3;

Qpp_rms(1,1)=NaN;
Qpp_rms(2,1) = sqrt(mean(qdd1_T1(:,2).^2));
Qpp_rms(3,1) = sqrt(mean(qdd2_T1(:,2).^2));
Qpp_rms(4,1) = sqrt(mean(qdd3_T1(:,2).^2));

Qpp_rms(1,2) = sqrt(mean(qdd1_T2(:,2).^2));
Qpp_rms(2,2) = sqrt(mean(qdd2_T2(:,2).^2));
Qpp_rms(3,2) = sqrt(mean(qdd3_T2(:,2).^2));
Qpp_rms(4,2) = sqrt(mean(qdd4_T2(:,2).^2));

% Max velocity for each joint
Qp_max=zeros(4,2);
Qp_max(1,1)=NaN;
Qp_max(2,1) = max(abs(qd1_T1(:,2)));
Qp_max(3,1) = max(abs(qd2_T1(:,2)));
Qp_max(4,1) = max(abs(qd3_T1(:,2)));

Qp_max(1,2) = max(abs(qd1_T2(:,2)));
Qp_max(2,2) = max(abs(qd2_T2(:,2)));
Qp_max(3,2) = max(abs(qd3_T2(:,2)));
Qp_max(4,2) = max(abs(qd4_T2(:,2)));

%% Find trasmission ratios

for j=1:2 %N° trajectories 
   for i=1:4 %N° motors
       alpha(i,j)=C_n(i)^2/Jm(i); 
       beta(i,j)=4*Jr(i)*Qpp_rms(i,j)^2;

       T_opt(i,j)=sqrt(Jm(i)*Qpp_rms(i,j)/Cr_rms(i,j));
       T_max(i,j)=sqrt(Jm(i))*(sqrt(alpha(i,j))+sqrt(alpha(i,j)-beta(i,j)))/(2*Jr(i)*Qpp_rms(i,j));
       T_min(i,j)=sqrt(Jm(i))*(sqrt(alpha(i,j))-sqrt(alpha(i,j)-beta(i,j)))/(2*Jr(i)*Qpp_rms(i,j)); 
       T_vel(i,j)=Qp_max(i,j)/Qpm_max;
   end 
end


%% Choose tau value

tau=[1/2 1/48 1/9 1/6];
Cm_max=[1.9 1.9 1.9 0.95]; 

Cr_rms_max=zeros(1,4);
C_max=zeros(1,4);

for i=1:4
Cr_rms_max(i)=max(Cr_rms(i,:));
end

% q_max(1)=max(qdd2_T2(:,2));
% q_max(2)=max([max(qdd1_T1(:,2)) max(qdd2_T2(:,2))]);
% q_max(3)=max([max(qdd2_T1(:,2)) max(qdd3_T2(:,2))]);
% q_max(4)=max([max(qdd3_T1(:,2)) max(qdd4_T2(:,2))]);

C_max(1)=max([max(abs(C_S_T1(:,1))) max(abs(C_S_T2(:,1)))]);
C_max(2)=max([max(abs(C_S_T1(:,2))) max(abs(C_S_T2(:,2)))]);
C_max(3)=max([max(abs(C_S_T1(:,3))) max(abs(C_S_T2(:,3)))]);
C_max(4)=max([max(abs(C_S_T1(:,4))) max(abs(C_S_T2(:,4)))]);

%% Condition that must be satisfied
for i = 1:4
    cond1 = C_n(i) > Cr_rms_max(i) * tau(i);
    cond2 = Cm_max(i) > tau(i) * C_max(i);
    
    fprintf('Giunto %d:\n', i);
    
    % Stampa valori della condizione 1
    fprintf('  Condizione 1: Cm_n = %.4f > Cr_rms_max*tau = %.4f -> %s\n', ...
            C_n(i), Cr_rms_max(i)*tau(i), ternary(cond1,'Superato','Non superato'));
    
    % Stampa valori della condizione 2
    fprintf('  Condizione 2: Cm_max = %.4f > tau*C_max = %.4f -> %s\n\n', ...
            Cm_max(i), tau(i)*C_max(i), ternary(cond2,'Superato','Non superato'));
end

% Funzione ausiliaria inline per sostituire il ternario (MATLAB non ha ?:)
function out = ternary(cond, valTrue, valFalse)
    if cond
        out = valTrue;
    else
        out = valFalse;
    end
end

%% Plot
figure;

giunti = 1:4;
trajectories = 0:1;

for i = 1:4
    subplot(2,2,i)
    hold on
    
    % Plot limiti
    plot(trajectories, T_min(i,:), 'v','MarkerSize',8,'MarkerFaceColor','r','DisplayName','T_{min}');
    plot(trajectories, T_max(i,:), '^','MarkerSize',8,'MarkerFaceColor','g','DisplayName','T_{max}');
    plot(trajectories, T_opt(i,:), 'o','MarkerSize',8,'MarkerFaceColor','b','DisplayName','T_{opt}');
    
    % Plot tau scelto
    plot([0 1], [tau(i) tau(i)], 'x-', 'Color', [0.5 0 0.5], ...
        'MarkerFaceColor',[0.5 0 0.5], 'MarkerSize', 6, 'DisplayName','Chosen \tau');
    
    % Plot tau max velocity
plot(trajectories, T_vel(i,:), 'o', 'MarkerSize', 6, ...
     'MarkerFaceColor','w','Color','k', 'DisplayName','\tau max vel');

    xlabel('Trajectory j')
    ylabel('\tau')
    title(['Joint ' num2str(i)])
    legend show
    grid on
    set(gca,'YScale','log')
    xticks(0:1)
end

%% Final plot: alpha (red) and beta (blue) for each Trajectory & Joint
% Order: T1J1, T2J1, T1J2, T2J2, T1J3, T2J3, T1J4, T2J4

% Build vectors and labels
labels = cell(1,8);
alpha_vec = zeros(1,8);
beta_vec  = zeros(1,8);
idx = 1;
for i = 1:4       % joint
    for j = 1:2   % trajectory
        labels{idx} = ['T' num2str(j) 'J' num2str(i)];
        alpha_vec(idx) = alpha(i,j);
        beta_vec(idx)  = beta(i,j);
        idx = idx + 1;
    end
end

% Plot
figure;
hold on;
hA = plot(1:8, alpha_vec, 'o-','LineWidth',1.5,'MarkerSize',8, ...
          'LineStyle','none','MarkerFaceColor','r','Color','r','DisplayName','alpha (C_n^2 / J_m)');
hB = plot(1:8, beta_vec,  's-','LineWidth',1.5,'MarkerSize',8, ...
          'LineStyle','none','MarkerFaceColor','b','Color','b','DisplayName','beta (4 J_r q̈_{rms}^2)');

set(gca,'YScale','log')           % logarithmic y-axis
xticks(1:8)
xticklabels(labels)
xtickangle(45)
xlabel('Trajectory & Joint')
ylabel('Value (log scale)')
title('Alpha and Beta values for each Trajectory & Joint')
legend('Location','best')
grid on
box on
hold off;
