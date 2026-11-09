function J_num = Jacobian_num(Q, L, G, delta)
% Numeric Jacobian by central finite differences
% Q: 4x1 vector of joint angles
% L, G: geometry (same as your code)
% delta: optional scalar or 4x1 vector of perturbations (recommended)
%
% Returns J_num (16x4)

if nargin < 4 || isempty(delta)
    % relative perturbation per joint to avoid cancellation
    delta = 1e-6*(1 + abs(Q));
end

nq = length(Q);
Se0 = forward_kinematics_numeric(Q, L, G);
J_num = zeros(length(Se0), nq);

for j = 1:nq
    d = delta(min(j,end)); % in case delta is scalar or vector
    Qp = Q; Qm = Q;
    Qp(j) = Qp(j) + d;
    Qm(j) = Qm(j) - d;
    Sep = forward_kinematics_numeric(Qp, L, G);
    Sem = forward_kinematics_numeric(Qm, L, G);
    J_num(:, j) = (Sep - Sem) / (2*d);
end
end