function Jd_num = Jacobian_d_num(Q, Qd, L, G, h, delta)
% Numeric time derivative of Jacobian using directional derivative
% Q: 4x1 joint angles
% Qd: 4x1 joint velocities
% h: small scalar step along direction Qd (optional)
% delta: passed to Jacobian_num (optional)
%
% Returns Jd_num (16x4)

if nargin < 5 || isempty(h)
    % Choose h relative to expected scale of Qd to keep finite diff stable.
    % If Qd is small, h must not be too small; this heuristic works well in practice.
    h = 1e-6;
end

% If Qd is exactly zero, Jd is zero
if all(abs(Qd) < 1e-14)
    Jd_num = zeros(16, length(Q));
    return
end

% Two Jacobian evaluations displaced along +/- Qd direction
Qp = Q + h * Qd;
Qm = Q - h * Qd;

Jp = Jacobian_num(Qp, L, G, delta);
Jm = Jacobian_num(Qm, L, G, delta);

Jd_num = (Jp - Jm) / (2*h);

end