function Jd=Jacobian_d(Q,Qd,L)

%Length of the links
l1=L(1);
l2=L(2);
l3=L(3);
l4=L(4);

% Definition of the Joint angles
syms theta1 theta2 theta3 theta4 real

% % Call the Jacobian function
% [Jnum, Jsym]=Jacobian(Q,L);
% Jd=zeros(3,4);

% Define the Joint Space
q=[theta1; theta2; theta3; theta4];
qdot = sym('qdot', [4 1]);  % symbolic joint velocities

% % Derivative of the Joint Space
% qdot=diff(q,t);

% Defining the Matrix Fi(qj)
F = [( l2*cos(theta2) + l3*cos(theta2+theta3) + l4*cos(theta2+theta3+theta4) )*cos(theta1);
  ( l2*cos(theta2)+l3*cos(theta2+theta3)+l4*cos(theta2+theta3+theta4) )*sin(theta1);
  l2*sin(theta2)+l3*sin(theta2+theta3)+l4*sin(theta2+theta3+theta4)];


% F = [ (l2*cos(theta2(t)) + l3*cos(theta2(t)+theta3(t)) + l4*cos(theta2(t)+theta3(t)+theta4(t)))*cos(theta1(t));
%         (l2*cos(theta2(t)) + l3*cos(theta2(t)+theta3(t)) + l4*cos(theta2(t)+theta3(t)+theta4(t)))*sin(theta1(t));
%          l2*sin(theta2(t)) + l3*sin(theta2(t)+theta3(t)) + l4*sin(theta2(t)+theta3(t)+theta4(t))];


%Initialization of the Jacobian Matrix
J=sym(zeros(3,4));

% Derive the functions with respect to the joint angle variables
for i=1:3
    for j=1:4
        J(i,j)=diff(F(i),q(j));
    end
end

Jd=zeros(3,4);

for i=1:4
    Jd=Jd+(diff(J,q(i))*qdot(i));
end

Jd = simplify(Jd);

Jd = subs(Jd,q,Q);
Jd = subs(Jd,qdot,Qd)
Jd = double(Jd);






% % Call the Jacobian function
% [Jnum, Jsym]=Jacobian(Q,L);
% Jd=zeros(3,4);


% 
% %Initialize the Jdot matrix
% Jd=zeros(3,4);
% 
% for i=1:4
%     Jd=Jd+(diff(Jsym,q(i))*qdot(i));
% end
% 
% % Jd=diff(Jsym,q(1))*qdot(1)+diff(Jsym,q(2))*qdot(2)+diff(Jsym,q(3))*qdot(3)+diff(Jsym,q(4))*qdot(4)
% 
% 
% Jd = subs(Jd,[q qdot],[Q Qd]);
% % Jd = double(Jd);


end

