function S=Direct_kin(Q,L)

% Length of the links
l1=L(1);
l2=L(2);
l3=L(3);
l4=L(4);

% Inizialization of the work-space vector
S=zeros(3,1);

% Direct Kinematics analytical model
S(1) = ( l2*cos(Q(2)) + l3*cos(Q(2)+Q(3)) + l4*cos(Q(2)+Q(3)+Q(4)) )*cos(Q(1));
S(2) = ( l2*cos(Q(2))+l3*cos(Q(2)+Q(3))+l4*cos(Q(2)+Q(3)+Q(4)) )*sin(Q(1));
S(3) = l1+l2*sin(Q(2))+l3*sin(Q(2)+Q(3))+l4*sin(Q(2)+Q(3)+Q(4));

end


% If the base link offset is included (l1)
% S(3) = l1+l2*sin(Q(2))+l3*sin(Q(2)+Q(3))+l4*sin(Q(2)+Q(3)+Q(4));

% If the base link offset is not included (l1)
%S(3) = l2*sin(Q(2))+l3*sin(Q(2)+Q(3))+l4*sin(Q(2)+Q(3)+Q(4));