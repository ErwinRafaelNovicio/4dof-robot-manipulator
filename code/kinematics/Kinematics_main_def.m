clear; close all; clc;

%% --- Parametri del robot ---
L = [0.5; 0.4; 0.3; 0.17]; % Lunghezze link [m]

%% --- Caricamento traiettorie desiderate ---
basePath = 'C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Kinematics\';

load(fullfile(basePath, 'Trj_1.mat'));  % → variabile Trj_1
load(fullfile(basePath, 'Trj_2.mat'));  % → variabile Trj_2

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
    savePath = fullfile(basePath, '..\Motion Planning\', ['JointAngles_' trajNames{t} '.mat']);
    save(savePath, 'Q');

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

        axis([-1 1 -1 1 -0.5 1]);
        view(45, 25);
        grid on;
        drawnow;
        pause(0.1);
    end
end
