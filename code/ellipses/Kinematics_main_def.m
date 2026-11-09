clear; close all; clc;

%% --- Parametri del robot ---
L = [0.5; 0.4; 0.3; 0.17]; % Lunghezze link [m]

%% --- Caricamento traiettorie desiderate ---
%basePath = 'C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Kinematics\';

%load(fullfile(basePath, 'Trj_1.mat'));  % → variabile Trj_1
%load(fullfile(basePath, 'Trj_2.mat'));  % → variabile Trj_2
load('Trj_2.mat');
load('Trj_1.mat');

trajectories = {Trj_1, Trj_2};
trajNames = {'1', '2'};

%% --- Ciclo su entrambe le traiettorie ---
for t = 1:2
    S_d = trajectories{t};
    Q = zeros(4, length(S_d)); % Matrice angoli giunti

    % --- Inverse Kinematics ---
    for i = 1:length(S_d)
        Q(:, i) = rad2deg(Inverse_kin(S_d(:, i), L));
    end

    % --- Salvataggio angoli in gradi ---
    %savePath = fullfile(basePath, '..\Motion Planning\', ['JointAngles_' trajNames{t} '.mat']);
    %save(savePath, 'Q');

    %% --- Animazione 3D ---
    Q = deg2rad(Q);
    nSteps = size(Q, 2);

    figure('Color', 'w');
    axis equal; grid on;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(['Animazione Robot 4DOF - Traiettoria ' trajNames{t} ' (θ₁ su Z, θ₂–θ₄ su Y)']);
    view(45, 25);
    axis([-1 1 -1 1 -0.5 1]); hold on;

    linkColor = [0.2 0.6 1];
    jointColor = [0.9 0.2 0.2];
    traj = [];
    
    conditioning_inv=zeros(1,nSteps); %we generate the vector of the conditioning index
    for i = 1:nSteps
        q = Q(:, i);

        % Posizioni giunti (cinematica diretta semplificata)
        P0 = [0; 0; 0];
        P1 = [0; 0; L(1)];
        P2 = Direct_kin([q(1); q(2); 0; 0], [L(1); L(2); 0; 0]);
        P3 = Direct_kin([q(1); q(2); q(3); 0], [L(1); L(2); L(3); 0]);
        P4 = Direct_kin(q, L);

        traj = [traj, P4];

        % --- Disegno ---
        cla;
        plot3([P0(1) P1(1)], [P0(2) P1(2)], [P0(3) P1(3)], 'Color', linkColor, 'LineWidth', 3); hold on;
        plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], 'Color', linkColor, 'LineWidth', 3);
        plot3([P2(1) P3(1)], [P2(2) P3(2)], [P2(3) P3(3)], 'Color', linkColor, 'LineWidth', 3);
        plot3([P3(1) P4(1)], [P3(2) P4(2)], [P3(3) P4(3)], 'Color', linkColor, 'LineWidth', 3);

        % Giunti
        plot3(P0(1), P0(2), P0(3), 'o', 'MarkerFaceColor', jointColor, 'MarkerEdgeColor', 'k');
        plot3(P1(1), P1(2), P1(3), 'o', 'MarkerFaceColor', jointColor, 'MarkerEdgeColor', 'k');
        plot3(P2(1), P2(2), P2(3), 'o', 'MarkerFaceColor', jointColor, 'MarkerEdgeColor', 'k');
        plot3(P3(1), P3(2), P3(3), 'o', 'MarkerFaceColor', jointColor, 'MarkerEdgeColor', 'k');
        plot3(P4(1), P4(2), P4(3), 's', 'MarkerFaceColor', [0.1 0.9 0.1], 'MarkerEdgeColor', 'k');

        % Traiettoria
        plot3(traj(1, :), traj(2, :), traj(3, :), 'k--', 'LineWidth', 1.2);

        %Elissoidi
        J=Jacobian(q,L); %jacobian step per step
        M=J*J'; %manipulability matrix
        [V,D]=eig(M); %eigenvector(direction) and eigenvalues(semiaxis of ellipsoid)
        %raggi ellissoide
        rx = sqrt(D(1,1));
        ry = sqrt(D(2,2));
        rz = sqrt(D(3,3));
        
        % Ellissoide unitario
        [xe,ye,ze] = ellipsoid(0,0,0, rx, ry, rz, 15);

        % Rotazione ellissoide
        pts = V * [xe(:)'; ye(:)'; ze(:)'];

        % CENTRO = posizione end-effector = P4
        P = P4;

        xe2 = reshape(pts(1,:) + P(1), size(xe));
        ye2 = reshape(pts(2,:) + P(2), size(ye));
        ze2 = reshape(pts(3,:) + P(3), size(ze));

        % Plot ellissoide
        surf(xe2, ye2, ze2, 'EdgeColor','k', 'FaceAlpha',0.1);

        plot3(P(1), P(2), P(3), 'ro', 'MarkerSize', 6,'MarkerFaceColor', 'r');   % marker rosso pieno
        
        % Evaluation of conditioning index
        eig_min=min(diag(D));
        eig_max=max(diag(D));
        conditioning_inv(i)=sqrt(eig_min/eig_max);
        
        axis([0.2 1.5 -1 1 -1 2]);
        view(0, 0);
        grid on;
        drawnow;
        pause(0.1);
    end

    figure();
    plot(1:nSteps,conditioning_inv,'o','LineStyle','none');
    axis([0 nSteps 0 1]);
    grid on;
end
