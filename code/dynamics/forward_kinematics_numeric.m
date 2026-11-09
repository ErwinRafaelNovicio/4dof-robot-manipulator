function Se = forward_kinematics_numeric(Q, L, G)
% Numeric version of your Se symbolic expressions
% Q is 4x1: [th1; th2; th3; th4]
th1 = Q(1); th2 = Q(2); th3 = Q(3); th4 = Q(4);

l1 = L(1); l2 = L(2); l3 = L(3); l4 = L(4);
g1 = G(1); g2 = G(2); g3 = G(3); g4 = G(4);

Se = zeros(16,1);

% replicate exactly your symbolic definitions
Se(1)  = (l2*cos(th2)+l3*cos(th2+th3)+l4*cos(th2+th3+th4))*cos(th1); % xp
Se(2)  = (l2*cos(th2)+l3*cos(th2+th3)+l4*cos(th2+th3+th4))*sin(th1); % yp
Se(3)  = l2*sin(th2)+l3*sin(th2+th3)+l4*sin(th2+th3+th4)+l1;        % zp

Se(4)  = (l2*cos(th2)+l3*cos(th2+th3)+g4*cos(th2+th3+th4))*cos(th1); % xg4
Se(5)  = (l2*cos(th2)+l3*cos(th2+th3)+g4*cos(th2+th3+th4))*sin(th1); % yg4
Se(6)  = l2*sin(th2)+l3*sin(th2+th3)+g4*sin(th2+th3+th4)+l1;        % zg4

Se(7)  = th2 + th3 + th4; % Fi

Se(8)  = (l2*cos(th2)+g3*cos(th2+th3))*cos(th1); % xg3
Se(9)  = (l2*cos(th2)+g3*cos(th2+th3))*sin(th1); % yg3
Se(10) = l2*sin(th2)+g3*sin(th2+th3)+l1;        % zg3

Se(11) = th2 + th3;

Se(12) = g2*cos(th2)*cos(th1); % xg2
Se(13) = g2*cos(th2)*sin(th1); % yg2
Se(14) = g2*sin(th2)+l1;       % zg2

Se(15) = th2;  % Psi
Se(16) = th1;  % th1

end