function [T_lim,dq_lim] = limit_case(motor)
T_lim(1) = motor.V*(1/motor.A + 1/motor.D);
dq_lim = 1/2*motor.V^2*(1/motor.A+1/motor.D);
end