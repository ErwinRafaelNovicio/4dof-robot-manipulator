function Jd_dyn = Jacobian_d_dyn(Q,Qd,L,G)

l1 = L(1);
l2 = L(2);
l3 = L(3);
l4 = L(4);

% Distance of each Centroid of each link wrt the Joint 2
g1 = G(1);
g2 = G(2);
g3 = G(3);
g4 = G(4);

syms th1 th2 th3 th4
syms qd1 qd2 qd3 qd4

q = [th1; th2; th3; th4];
qd = [qd1; qd2; qd3; qd4];


Se = sym(zeros(16,1));

Se(1,1) = (l2*cos(q(2))+l3*cos(q(2)+q(3))+l4*cos(q(2)+q(3)+q(4)))*cos(q(1)); %xp
Se(2,1) = (l2*cos(q(2))+l3*cos(q(2)+q(3))+l4*cos(q(2)+q(3)+q(4)))*sin(q(1)); %yp
Se(3,1) = l2*sin(q(2))+l3*sin(q(2)+q(3))+l4*sin(q(2)+q(3)+q(4))+l1; %zp
Se(4,1) = (l2*cos(q(2))+l3*cos(q(2)+q(3))+g4*cos(q(2)+q(3)+q(4)))*cos(q(1)); %xg4
Se(5,1) = (l2*cos(q(2))+l3*cos(q(2)+q(3))+g4*cos(q(2)+q(3)+q(4)))*sin(q(1)); %yg4
Se(6,1) = l2*sin(q(2))+l3*sin(q(2)+q(3))+g4*sin(q(2)+q(3)+q(4))+l1; %zg4
Se(7,1) = q(2)+q(3)+q(4); % Fi
Se(8,1) = (l2*cos(q(2))+g3*cos(q(2)+q(3)))*cos(q(1)); %xg3
Se(9,1) = (l2*cos(q(2))+g3*cos(q(2)+q(3)))*sin(q(1)); %yg3
Se(10,1) = l2*sin(q(2))+g3*sin(q(2)+q(3))+l1; %zg3
Se(11,1) = q(2)+q(3);
Se(12,1) = g2*cos(q(2))*cos(q(1)); %xg2
Se(13,1) = g2*cos(q(2))*sin(q(1)); %yg2
Se(14,1) = g2*sin(q(2))+l1; %zg2
Se(15,1) = q(2); %Psi
Se(16,1) = q(1); %th1

J_dyn = sym(zeros(16,4));

for i=1:length(Se)
    for j=1:length(q)
        J_dyn(i,j) = diff(Se(i), q(j)); % Compute the Jacobian elements
    end
end

Jd_dyn = sym(zeros(16,4));

for i=1:length(q)
    Jd_dyn = Jd_dyn+ diff(J_dyn,q(i))*qd(i,1);
end

Jd_dyn = simplify(Jd_dyn);

% Jd = subs(Jd,[q,qdot], [Q,Qd] )
Jd_dyn = subs(Jd_dyn,q,Q);
Jd_dyn = subs(Jd_dyn,qd,Qd);

Jd_dyn = double(Jd_dyn);

end