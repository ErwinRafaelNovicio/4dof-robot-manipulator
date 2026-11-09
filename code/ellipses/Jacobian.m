function [Jnum, Jsym]=Jacobian(Q,L)

%Jnum - Jacobian with the numerical values
%Jsym - Jacobian with the symbolical values

%Length of the links
l1=L(1);
l2=L(2);
l3=L(3);
l4=L(4);

 % Symbolic time
 syms t
 syms theta1(t) theta2(t) theta3(t) theta4(t)

 q = [theta1(t); theta2(t); theta3(t); theta4(t)];


  % Forward kinematics (end-effector position only)
  F = [ (l2*cos(theta2(t)) + l3*cos(theta2(t)+theta3(t)) + l4*cos(theta2(t)+theta3(t)+theta4(t)))*cos(theta1(t));
        (l2*cos(theta2(t)) + l3*cos(theta2(t)+theta3(t)) + l4*cos(theta2(t)+theta3(t)+theta4(t)))*sin(theta1(t));
         l2*sin(theta2(t)) + l3*sin(theta2(t)+theta3(t)) + l4*sin(theta2(t)+theta3(t)+theta4(t))];

% % Define symbolics variables for the joint angles
% syms theta1 theta2 theta3 theta4
% q = [theta1; theta2; theta3; theta4]; %qj
% 
% 
% % Defining the Matrix Fi(qj)
% F = [( l2*cos(theta2) + l3*cos(theta2+theta3) + l4*cos(theta2+theta3+theta4) )*cos(theta1);
%   ( l2*cos(theta2)+l3*cos(theta2+theta3)+l4*cos(theta2+theta3+theta4) )*sin(theta1);
%   l2*sin(theta2)+l3*sin(theta2+theta3)+l4*sin(theta2+theta3+theta4)];

%Initialization of the Jacobian Matrix
J=sym(zeros(3,4));

% Derive the functions with respect to the joint angle variables
for i=1:3
    for j=1:4
        J(i,j)=diff(F(i),q(j));
    end
end

% Parametric Jacobian Matrix
Jsym=J;

%Substitute the values on the theta parameters symbolically
J=subs(J,q,Q);

%Transform the symbolic values into real numbers
Jnum=double(J);


end