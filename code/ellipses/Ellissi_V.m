% Mostra l'evoluzione dinamica degli ellissoidi di manipulabilità per le
% velocità per le traiettorie Trj_1 e Trj_2.

clear; close all; clc;

%% --- Parametri del robot ---
L = [0.5; 0.4; 0.3; 0.17];

%% --- Caricamento traiettorie ---
load('Trj_1.mat');
load('Trj_2.mat');

trajectories = {Trj_1, Trj_2};
names = {'1','2'};

%% --- Loop su entrambe le traiettorie ---
gifFilePrefix = 'RobotEllissoidi_V_';
for t = 1:2
    % Pre-calcola la traiettoria completa
    S_d = trajectories{t};
    nSteps = size(S_d,2);
    conditioning_inv=zeros(1,nSteps); %we generate the vector of the conditioning index
    traj = zeros(3, nSteps);
    for k = 1:nSteps
        qk = Inverse_kin(S_d(:,k), L);
        traj(:,k) = Direct_kin(qk, L);
    end

    % Figura per ellissoidi
    fig = figure('Color','w'); hold on; grid on; axis equal;
    % xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    % title(['Evoluzione ellissoidi - Traiettoria ', names{t}]);
    % view(45,25);
    % axis([-1.5 1.5 -1.5 1.5 -1 2]);

    gifFile = [gifFilePrefix names{t} '.gif']; %per la generazione delle GIF
    for i = 1:nSteps
        %% --- Inverse + forward kinematics ---
        q = Inverse_kin(S_d(:,i), L);
        P4 = Direct_kin(q, L);

        %% --- Jacobiano e ellissoide ---
        J = Jacobian(q, L);
        M = J * J';
        [V,D] = eig(M);
        D = max(D,0);
        %In questo caso gli autovalori saranno inversi rispetto a quelli
        %delle ellissi di forza, mentre gli autovettori sono coincidenti
        rx = 1/sqrt(D(1,1)); ry = 1/sqrt(D(2,2)); rz = 1/sqrt(D(3,3));
        [xe,ye,ze] = ellipsoid(0,0,0, rx, ry, rz, 15);
        pts = V * [xe(:)'; ye(:)'; ze(:)'];
        xe2 = reshape(pts(1,:) + P4(1), size(xe));
        ye2 = reshape(pts(2,:) + P4(2), size(ye));
        ze2 = reshape(pts(3,:) + P4(3), size(ze));

        %% --- Conditioning number ---
        eig_min=min([rx ry rz]);
        eig_max=max([rx ry rz]);
        conditioning_inv(i)=sqrt(eig_min/eig_max);

        %% --- Pulizia e nuovo ellissoide ---
        cla; grid on; hold on;
        % --- traiettoria fissa ---
        plot3(traj(1,:), traj(2,:), traj(3,:), 'k--', 'LineWidth', 1.2);
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        title(['Ellipsoid-Velocity. Step ', num2str(i), ' / ', num2str(nSteps), ' - Trj ', names{t}]);
        %axis([-1 2 -1 1 -0.5 1.8]);
        view(-15,15);
        %view(15,10);
        %view(0,0);

        surf(xe2, ye2, ze2, 'FaceAlpha',0.12, 'EdgeColor','k');
        plot3(P4(1), P4(2), P4(3), 'ro', 'MarkerSize',4, 'MarkerFaceColor','r');

        % %% --- Cinematica diretta per animazione link ---
        % P0 = [0;0;0];
        % P1 = [0;0;L(1)];
        % P2 = Direct_kin([q(1); q(2); 0; 0], [L(1); L(2); 0; 0]);
        % P3 = Direct_kin([q(1); q(2); q(3); 0], [L(1); L(2); L(3); 0]);
        % P4 = Direct_kin(q, L);
        % 
        % % --- Robot links ---
        % plot3([P0(1) P1(1)], [P0(2) P1(2)], [P0(3) P1(3)], 'LineWidth', 3, 'Color',[0.2 0.6 1]);
        % plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], 'LineWidth', 3, 'Color',[0.2 0.6 1]);
        % plot3([P2(1) P3(1)], [P2(2) P3(2)], [P2(3) P3(3)], 'LineWidth', 3, 'Color',[0.2 0.6 1]);
        % plot3([P3(1) P4(1)], [P3(2) P4(2)], [P3(3) P4(3)], 'LineWidth', 3, 'Color',[0.2 0.6 1]);
        % 
        % 
        % % --- Giunti ---
        % plot3(P0(1), P0(2), P0(3), 'ko', 'MarkerFaceColor',[0.9 0.2 0.2]);
        % plot3(P1(1), P1(2), P1(3), 'ko', 'MarkerFaceColor',[0.9 0.2 0.2]);
        % plot3(P2(1), P2(2), P2(3), 'ko', 'MarkerFaceColor',[0.9 0.2 0.2]);
        % plot3(P3(1), P3(2), P3(3), 'ko', 'MarkerFaceColor',[0.9 0.2 0.2]);
        % plot3(P4(1), P4(2), P4(3), 'r*', 'MarkerSize',4, 'MarkerFaceColor',[0.9 0.2 0.2]);
        % plot3(P4(1), P4(2), P4(3), 'r*', 'MarkerSize',4, 'MarkerFaceColor',[0.9 0.2 0.2]);

        drawnow;
        pause(0.2);

        % --- Salvataggio frame in GIF ---
        frame = getframe(fig);
        im = frame2im(frame);
        frameImgName = sprintf('frame_%s_%04d.png', names{t}, i);
        imwrite(im, frameImgName);
        [A,map] = rgb2ind(im,256);
        if i == 1
            imwrite(A,map,gifFile,'gif','LoopCount',inf,'DelayTime',0.5);
        else
            imwrite(A,map,gifFile,'gif','WriteMode','append','DelayTime',0.5);
        end
    end
    %Plot of the conditioning index for each ellipse considered
    figure();
    plot(1:nSteps,conditioning_inv,'o','LineStyle','none','MarkerSize',4,'MarkerEdgeColor','r','MarkerFaceColor',[1 0.5 0]);

    limit=0.2;
    yline(limit,'b','LineStyle','-.','LineWidth',1.2);
    text(1, limit+0.02,'Reference limit','FontSize',8,'FontName','Calibri','FontAngle','italic','Color','b');

    title(['Conditioning (inverse) number trend - Trj', names{t}]);
    xlabel('Step number'); ylabel('Index value');
    xticks(1:1:nSteps); yticks(0:0.05:1);
    axis([0.8 nSteps+0.2 0 1]);
    grid on;
end