
clear; close all; clc;

%% =======================
%  Data
%  =======================
load('JointAngles_2.mat');   % deve contenere Q in gradi (nxm come nel tuo script precedente)

% motor (non usato direttamente in questa versione, lo lasciamo per compatibilità)
motor(1).A=4; motor(1).D=4; motor(1).V=100;
motor(2).A=3; motor(2).D=3; motor(2).V=100;
motor(3).A=4; motor(3).D=4; motor(3).V=100;
motor(4).A=5; motor(4).D=5; motor(4).V=200;

% Convert to radians and build Q matrix
q1 = deg2rad(Q(1,:));
q2 = deg2rad(Q(2,:));
q3 = deg2rad(Q(3,:));
q4 = deg2rad(Q(4,:));
Q  = [q1; q2; q3; q4];

% Differences (unused for spline construction but kept for compatibility)
dq1 = diff(q1);
dq2 = diff(q2);
dq3 = diff(q3);
dq4 = diff(q4);
dQ  = [dq1; dq2; dq3; dq4];
dQ_abs = abs(dQ);

%% =======================
%  0.1) Minimum actuating time for each displacement (t_vec)
%  =======================
rise_time = zeros(size(dQ_abs,1), size(dQ_abs,2)); % four joints
for i = 1:size(dQ_abs,1)
    for j = 1:size(dQ_abs,2)
        rise_time(i,j) = min_rise_time(motor(i), dQ_abs(i,j));
    end
end
T_min_act = max(rise_time);
t_vec = [0, cumsum(T_min_act, 2)]; % vettore tempi nodali (1 x nP)

%% =======================
%  Spline: cubic internal + quintic first/last intervals (C2 continuity)
%  =======================
nJ = size(Q,1);   % number of joints (4)
nP = size(Q,2);   % number of via points
Spline_Coeffs = cell(nJ,1);

for j = 1:nJ
    q = Q(j,:);
    t = t_vec;
    n = length(q) - 1; % numero intervalli (n = nP-1)
    if n < 1
        error('Servono almeno 2 punti (n>=1).');
    end
    h = diff(t);      % h(i) = t(i+1)-t(i), i=1..n

    % -----------------------
    %  1) Risolvo sistema per M usando condizione clamped (q'(t0)=0, q'(tf)=0)
    % -----------------------
    A = zeros(n+1,1);
    B = zeros(n+1,1);
    C = zeros(n+1,1);
    D = zeros(n+1,1);

    % boundary conditions: clamped q'(t0)=0, q'(tf)=0
    B(1) = 2*h(1);
    C(1) = h(1);
    D(1) = 6*((q(2)-q(1))/h(1) - 0);  % v0 = 0

    A(n+1) = h(n);
    B(n+1) = 2*h(n);
    D(n+1) = 6*(0 - (q(n+1)-q(n))/h(n));  % vf = 0

    for i = 2:n
        A(i) = h(i-1);
        B(i) = 2*(h(i-1)+h(i));
        C(i) = h(i);
        D(i) = 6*((q(i+1)-q(i))/h(i) - (q(i)-q(i-1))/h(i-1));
    end

    % Thomas algorithm (tridiagonal solver)
    for i = 2:n+1
        w = A(i)/B(i-1);
        B(i) = B(i) - w*C(i-1);
        D(i) = D(i) - w*D(i-1);
    end
    M = zeros(n+1,1);
    M(n+1) = D(n+1)/B(n+1);
    for i = n:-1:1
        M(i) = (D(i) - C(i)*M(i+1))/B(i);
    end
    % ora M(1..n+1) trovati

    % -----------------------
    %  2) Calcolo coefficienti cubici per tutti gli intervalli (a,b,c,d)
    % -----------------------
    a = zeros(1,n);
    b = zeros(1,n);
    c = zeros(1,n);
    d = zeros(1,n);
    for i = 1:n
        a(i) = q(i);
        c(i) = M(i)/2;
        d(i) = (M(i+1)-M(i))/(6*h(i));
        b(i) = (q(i+1)-q(i))/h(i) - h(i)*(2*M(i)+M(i+1))/6;
    end

    % -----------------------
    %  3) Costruisco quintica per primo intervallo (i=1)
    % -----------------------
    quint1 = []; % inizializzo a vuoto
    if n >= 1
        h1 = h(1);
        if n >= 2
            b2 = b(2);
            M2 = M(2);
        else
            b2 = 0;
            M2 = 0;
        end

        % coeffinci p0..p5
        p0 = q(1);
        p1 = 0;
        p2 = 0;
        H = h1;
        rhs1 = q(2) - (p0 + p1*H + p2*H^2);
        rhs2 = b2 - (p1 + 2*p2*H);
        rhs3 = M2 - (2*p2);

        A_q = [ H^3     H^4      H^5;
                3*H^2   4*H^3    5*H^4;
                6*H     12*H^2   20*H^3 ];

        sol = A_q \ [rhs1; rhs2; rhs3];
        p3 = sol(1); p4 = sol(2); p5 = sol(3);

        quint1 = struct('p0',p0,'p1',p1,'p2',p2,'p3',p3,'p4',p4,'p5',p5,'h',h1);
    end

    % -----------------------
    %  4) Costruisco quintica per ultimo intervallo (i=n)
    % -----------------------
    quintN = []; % inizializzo a vuoto
    if n >= 1
        hn = h(n);
        if n >= 2
            dx_prev = h(n-1);
            b_left_n = b(n-1) + 2*c(n-1)*dx_prev + 3*d(n-1)*dx_prev^2;
            M_left_n = M(n);
        else
            b_left_n = 0;
            M_left_n = 0;
        end

        H = hn;
        r0 = q(n);
        r1 = b_left_n;
        r2 = M_left_n/2;

        rhs1 = q(n+1) - (r0 + r1*H + r2*H^2);
        rhs2 = 0 - (r1 + 2*r2*H);
        rhs3 = 0 - (2*r2);

        A_qn = [ H^3     H^4      H^5;
                 3*H^2   4*H^3    5*H^4;
                 6*H     12*H^2   20*H^3 ];

        soln = A_qn \ [rhs1; rhs2; rhs3];
        r3 = soln(1); r4 = soln(2); r5 = soln(3);

        quintN = struct('r0',r0,'r1',r1,'r2',r2,'r3',r3,'r4',r4,'r5',r5,'h',hn);
    end

    % -----------------------
    %  Save coefficients (include quintic structs for edge intervals)
    % -----------------------
    Spline_Coeffs{j} = struct('a',a,'b',b,'c',c,'d',d,'M',M, ...
                              'quint1',quint1, 'quintN',quintN, 't', t);
end

%% Sampling Time
% Unifrom Sampling
% ============================
Ts = 0.001;  % 1 ms

% Determine total simulation time
T_end = max(t_vec);

% Time vector with uniform sampling
t_uniform_T2 = 0:Ts:T_end;

q_uniform_T2=[];
qd_uniform_T2=[];
qdd_uniform_T2=[];


%% =======================
%  2) Posizioni con punti da raggiungere (plot)
%  =======================
figure; hold on; grid on;
colors = lines(4);
markers = ['o','o','o','o'];

for j = 1:nJ
    coeff = Spline_Coeffs{j};
    xx = linspace(coeff.t(1), coeff.t(end), 500);
    yy = zeros(size(xx));
    n = length(coeff.a);

    for i = 1:n
        idx = xx >= coeff.t(i) & xx <= coeff.t(i+1);
        dx = xx(idx) - coeff.t(i);
        if i == 1 && ~isempty(coeff.quint1)
            p = coeff.quint1;
            yy(idx) = p.p0 + p.p1.*dx + p.p2.*dx.^2 + p.p3.*dx.^3 + p.p4.*dx.^4 + p.p5.*dx.^5;
        elseif i == n && ~isempty(coeff.quintN)
            r = coeff.quintN;
            yy(idx) = r.r0 + r.r1.*dx + r.r2.*dx.^2 + r.r3.*dx.^3 + r.r4.*dx.^4 + r.r5.*dx.^5;
        else
            yy(idx) = coeff.a(i) + coeff.b(i).*dx + coeff.c(i).*dx.^2 + coeff.d(i).*dx.^3;
        end
    end

    plot(xx, yy, 'Color', colors(j,:), 'LineWidth', 2)
    %plot(coeff.t, Q(j,:), [colors(j) markers(j)], 'MarkerSize', 6, 'LineStyle', 'none')
    q_uniform_T2(j,:) = interp1(xx,yy,t_uniform_T2,'linear');
end

xlabel('t [s]')
ylabel('q [rad]')
title('Joints Position')
legend('joint1','joint 2','joint 3','joint 4')

%% =======================
%  3) Velocità dei giunti (plot)
%  =======================
figure; hold on; grid on;
for j = 1:nJ
    coeff = Spline_Coeffs{j};
    xx = linspace(coeff.t(1), coeff.t(end), 500);
    qd = zeros(size(xx));
    n = length(coeff.a);

    for i = 1:n
        idx = xx >= coeff.t(i) & xx <= coeff.t(i+1);
        dx = xx(idx) - coeff.t(i);
        if i == 1 && ~isempty(coeff.quint1)
            p = coeff.quint1;
            qd(idx) = p.p1 + 2*p.p2.*dx + 3*p.p3.*dx.^2 + 4*p.p4.*dx.^3 + 5*p.p5.*dx.^4;
        elseif i == n && ~isempty(coeff.quintN)
            r = coeff.quintN;
            qd(idx) = r.r1 + 2*r.r2.*dx + 3*r.r3.*dx.^2 + 4*r.r4.*dx.^3 + 5*r.r5.*dx.^4;
        else
            qd(idx) = coeff.b(i) + 2*coeff.c(i).*dx + 3*coeff.d(i).*dx.^2;
        end
    end

    plot(xx, qd, 'Color', colors(j,:), 'LineWidth', 2)
    qd_uniform_T2(j,:) = interp1(xx,qd,t_uniform_T2,'linear');
    % plot(t_uniform_T2,  qd_uniform_T2(j,:),  [colors(j) '--'], 'LineWidth', 2)
end

xlabel('t [s]')
ylabel('q-dot [rad/s]')
title('Joints Velocities')
legend('joint1','joint 2','joint 3','joint 4')

%% =======================
%  4) Accelerazioni dei giunti (plot)
%  =======================
figure; hold on; grid on;
for j = 1:nJ
    coeff = Spline_Coeffs{j};
    xx = linspace(coeff.t(1), coeff.t(end), 500);
    qdd = zeros(size(xx));
    n = length(coeff.a);

    for i = 1:n
        idx = xx >= coeff.t(i) & xx <= coeff.t(i+1);
        dx = xx(idx) - coeff.t(i);
        if i == 1 && ~isempty(coeff.quint1)
            p = coeff.quint1;
            qdd(idx) = 2*p.p2 + 6*p.p3.*dx + 12*p.p4.*dx.^2 + 20*p.p5.*dx.^3;
        elseif i == n && ~isempty(coeff.quintN)
            r = coeff.quintN;
            qdd(idx) = 2*r.r2 + 6*r.r3.*dx + 12*r.r4.*dx.^2 + 20*r.r5.*dx.^3;
        else
            qdd(idx) = 2*coeff.c(i) + 6*coeff.d(i).*dx;
        end
    end

    plot(xx, qdd, 'Color', colors(j,:), 'LineWidth', 2)
    qdd_uniform_T2(j,:) = interp1(xx,qdd,t_uniform_T2,'linear');
end

xlabel('t [s]')
ylabel('q-double-dot [rad/s^2]')
title('Joints Accelerations')
legend('joint1','joint 2','joint 3','joint 4')

%% ===========================

% Build [time value] matrices
q1_T2 = [t_uniform_T2' q_uniform_T2(1,:)'];
q2_T2 = [t_uniform_T2' q_uniform_T2(2,:)'];
q3_T2 = [t_uniform_T2' q_uniform_T2(3,:)'];
q4_T2 = [t_uniform_T2' q_uniform_T2(4,:)'];

qd1_T2 = [t_uniform_T2' qd_uniform_T2(1,:)'];
qd2_T2 = [t_uniform_T2' qd_uniform_T2(2,:)'];
qd3_T2 = [t_uniform_T2' qd_uniform_T2(3,:)'];
qd4_T2 = [t_uniform_T2' qd_uniform_T2(4,:)'];

qdd1_T2 = [t_uniform_T2' qdd_uniform_T2(1,:)'];
qdd2_T2 = [t_uniform_T2' qdd_uniform_T2(2,:)'];
qdd3_T2 = [t_uniform_T2' qdd_uniform_T2(3,:)'];
qdd4_T2 = [t_uniform_T2' qdd_uniform_T2(4,:)'];



%% Save results
% SIMSCAPE
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\SIMSCAPE\Joint_Positions_T2_ts.mat','q1_T2','q2_T2','q3_T2','q4_T2');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\SIMSCAPE\Joint_Velocities_T2_ts.mat','qd1_T2','qd2_T2','qd3_T2','qd4_T2');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\SIMSCAPE\Joint_Accelerations_T2_ts.mat','qdd1_T2','qdd2_T2','qdd3_T2','qdd4_T2');

% Dynamical Model
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Dynamics\Joint_Positions_T2_ts.mat','q1_T2','q2_T2','q3_T2','q4_T2');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Dynamics\Joint_Velocities_T2_ts.mat','qd1_T2','qd2_T2','qd3_T2','qd4_T2');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Dynamics\Joint_Accelerations_T2_ts.mat','qdd1_T2','qdd2_T2','qdd3_T2','qdd4_T2');

% Direct Method Test
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Motion Planning Test_DK\Joint_Positions_T2_ts.mat','q1_T2','q2_T2','q3_T2','q4_T2');

% Motors adn trasmissions sizing
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Motor_Identification\Joint_Accelerations_T2_ts.mat','qdd1_T2','qdd2_T2','qdd3_T2','qdd4_T2');
save('C:\Users\Admin\OneDrive - Politecnico di Milano\File di Erwin Rafael Novicio - Industrial Robotics SY 25-26\Matlab Code\Motor_Identification\Joint_Velocities_T2_ts.mat','qd1_T2','qd2_T2','qd3_T2','qd4_T2');