function Q = Inverse_kin(S,L)

% Lunghezze dei link
l1 = L(1);
l2 = L(2);
l3 = L(3);
l4 = L(4);

% Coordinate end-effector
x = S(1);
y = S(2);
z = S(3);

% Inizializzazione vettore giunti
Q = zeros(4,1);

% Angolo base (theta1)
theta1 = atan2(y, x);
Q(1) = theta1;

% Variabili simboliche per i restanti giunti
syms theta2 theta3 theta4

% Posizione del "polso" (wrist)
X = x - l4 * cos(theta1) * cos(theta1); % semplificare se necessario
Y = y - l4 * cos(theta1) * sin(theta1);
Z = z - l4 * sin(theta1);

% Equazioni cinematiche inverse
Phi = atan2(z, sqrt(x^2 + y^2));
eq1 = theta2 == Phi - atan2(l4*sin(theta3+theta4) + l3*sin(theta3), l2 + l3*cos(theta3) + l4*cos(theta3+theta4));
%eq2 = theta3 == -acos((X^2 + Y^2 + Z^2 - l2^2 - l3^2)/(2*l2*l3)); %elbow up             
eq2 = theta3 == acos((X^2 + Y^2 + Z^2 - l2^2 - l3^2)/(2*l2*l3)); %elbow down
eq3 = theta4 == Phi - theta2 - theta3;

% Vincoli in radianti
theta2_min = deg2rad(-45); theta2_max = deg2rad(90);
theta3_min = deg2rad(-110); theta3_max = deg2rad(110);
theta4_min = deg2rad(-120); theta4_max = deg2rad(120);

% Risolvo considerando possibili soluzioni multiple
sol = vpasolve([eq1, eq2, eq3], [theta2, theta3, theta4], ...
    [theta2_min theta2_max; theta3_min theta3_max; theta4_min theta4_max]);

% Estraggo tutte le soluzioni
theta2_sols = double(sol.theta2);
theta3_sols = double(sol.theta3);
theta4_sols = double(sol.theta4);

% Filtro solo soluzioni reali e nei vincoli
valid_idx = find(...
    imag(theta2_sols) == 0 & theta2_sols >= theta2_min & theta2_sols <= theta2_max & ...
    imag(theta3_sols) == 0 & theta3_sols >= theta3_min & theta3_sols <= theta3_max & ...
    imag(theta4_sols) == 0 & theta4_sols >= theta4_min & theta4_sols <= theta4_max);

if isempty(valid_idx)
    error('Nessuna soluzione valida nei limiti dati');
end

% Seleziono la prima soluzione valida
Q(2) = theta2_sols(valid_idx(1));
Q(3) = theta3_sols(valid_idx(1));
Q(4) = theta4_sols(valid_idx(1));

end


%% OLD CODE
% function Q = Inverse_kin(S,L)
% 
% 
% % Length of the links
% l1=L(1);
% l2=L(2);
% l3=L(3);
% l4=L(4);
% 
% % End Effector coordinates
% x=S(1);
% y=S(2);
% z=S(3);
% 
% %With base links offset (l1)
% % z=z-l1;
% 
% % Initialization of the joint-space vector
% Q=zeros(4,1);
% 
% % Orientation of the End Effector position vector
% Phi=atan2(z,sqrt(x^2+y^2));
% 
% % Joint angles computation 
% 
% theta1=atan2(y,x);
% Q(1)=theta1;
% 
% % Defining the symbolic joint-space variables
% syms theta2 theta3 theta4
% 
% X=x-l4*cos(Phi)*cos(theta1);
% Y=y-l4*cos(Phi)*sin(theta1);
% Z=z-l4*sin(Phi);
% 
% % Defining the symbolic system of equations (unknowns: theta2, theta3 and theta4)
% eq1= theta2 == Phi-atan2( [l4*sin(theta3+theta4)+l3*sin(theta3)],[l2+l3*cos(theta3)+l4*cos(theta3+theta4)] );
% eq2= theta3 == acos( (X^2+Y^2+Z^2-l2^2-l3^2)/(2*l2*l3));
% eq3= theta4 == Phi-theta2-theta3;
% 
% % Numerically solving the symbolic non-linear equations 
% [Q(2),Q(3),Q(4)]=vpasolve([eq1,eq2,eq3], [theta2,theta3,theta4]);
% 
% 
% end
% 