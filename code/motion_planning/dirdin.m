function S = dirdin(Q,L)
%direct kinematics of the extended 2R
l1=L(1); l2=L(2); l3=L(3); l4=L(4);
g1=L(5); g2=L(6); g3=L(7); g4=L(8);

S=zeros(8,1);
S(1)=l1*cos(Q(1))+l2*cos(Q(1)+Q(2));
S(2)=l1*sin(Q(1))+l2*sin(Q(1)+Q(2));
S(3)=l1*cos(Q(1))+g2*cos(Q(1)+Q(2));
S(4)=l1*sin(Q(1))+g2*sin(Q(1)+Q(2));
S(5)=(Q(1)+Q(2));
S(6)=g1*cos(Q(1));
S(7)=g1*sin (Q(1));
S(8)=(Q(1));
end
function