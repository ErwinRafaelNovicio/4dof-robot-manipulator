%% --- Parametri robot ---
L = [0.5, 0.4, 0.3, 0.17]; % link lengths
qmin = deg2rad([-40, -52, -112, -120]);
qmax = deg2rad([ 40,  52,  112,  120]);

% sampling
nq2 = 20; nq3 = 20; nq4 = 20; ntheta = 20;

% compute workspace (full)
P_full = workspace_symmetry(qmin, qmax, L, nq2, nq3, nq4, ntheta);

%% --- remove points with x < 0 (not accessible) ---
P = P_full(P_full(:,1) >= 0, :);   % keep only x >= 0

%% --- Mesh esterna con alphaShape sulla mesh troncata ---
shp = alphaShape(P(:,1), P(:,2), P(:,3), 0.05);
[tri, verts] = boundaryFacets(shp);

%% === Vista 3D: workspace (solo x>=0) ===
figure;
p = patch('Vertices', verts, 'Faces', tri);
p.FaceColor = [0 0.7 1];
p.EdgeColor = 'none';
p.FaceAlpha = 0.35;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Workspace 3D - External Mesh');
axis equal; grid on;
camlight HEADLIGHT; lighting GOURAUD;
view(45,30);

hold on;

%% --- Correct spherical-shell sector (improved visibility) ---
center = [0, 0, 0.5];   % [xc, yc, zc]
r_in  = 0.522;
r_out = 0.870;

alpha_deg = linspace(-23, 60, 120);
alpha = deg2rad(alpha_deg);

phi_deg = linspace(-40, 40, 120);
phi = deg2rad(phi_deg);

[PHI, ALPHA] = meshgrid(phi, alpha);

X_out = center(1) + r_out * cos(ALPHA) .* cos(PHI);
Y_out = center(2) + r_out * cos(ALPHA) .* sin(PHI);
Z_out = center(3) + r_out * sin(ALPHA);

X_in  = center(1) + r_in  * cos(ALPHA) .* cos(PHI);
Y_in  = center(2) + r_in  * cos(ALPHA) .* sin(PHI);
Z_in  = center(3) + r_in  * sin(ALPHA);

% Better visibility: stronger color, higher alpha, slight darker edges
facecolor = [1 0.4 0.4]; % slightly more saturated red

hOut = surf(X_out, Y_out, Z_out, ...
    'FaceColor', facecolor, 'FaceAlpha', 0.55, ...
    'EdgeColor', [0.3 0.1 0.1]);

hold on;

hIn  = surf(X_in, Y_in, Z_in, ...
    'FaceColor', facecolor, 'FaceAlpha', 0.55, ...
    'EdgeColor', [0.3 0.1 0.1]);

% close caps on elevation edges
for idx = [1, size(ALPHA,1)]
    xo = X_out(idx, :); yo = Y_out(idx, :); zo = Z_out(idx, :);
    xi = X_in(idx,  :); yi = Y_in(idx,  :); zi = Z_in(idx,  :);
    Xcap = [xo, fliplr(xi)];
    Ycap = [yo, fliplr(yi)];
    Zcap = [zo, fliplr(zi)];
    patch(Xcap, Ycap, Zcap, facecolor, ...
        'FaceAlpha', 0.55, 'EdgeColor', [0.3 0.1 0.1]);
end

% close caps on azimuth edges
for jdx = [1, size(PHI,2)]
    xo = X_out(:, jdx)'; yo = Y_out(:, jdx)'; zo = Z_out(:, jdx)';
    xi = X_in(:, jdx)';  yi = Y_in(:, jdx)';  zi = Z_in(:, jdx)';
    Xcap = [xo, fliplr(xi)];
    Ycap = [yo, fliplr(yi)];
    Zcap = [zo, fliplr(zi)];
    patch(Xcap, Ycap, Zcap, facecolor, ...
        'FaceAlpha', 0.55, 'EdgeColor', [0.3 0.1 0.1]);
end

% removed red line marker (plot3)

camlight HEADLIGHT;
lighting GOURAUD;
axis equal;


%% --- workspace_symmetry function (unchanged) ---
function P = workspace_symmetry(qmin, qmax, L, nq2, nq3, nq4, ntheta)
q2vec = linspace(qmin(2), qmax(2), nq2);
q3vec = linspace(qmin(3), qmax(3), nq3);
q4vec = linspace(qmin(4), qmax(4), nq4);
theta = linspace(qmin(1), qmax(1), ntheta); % samples for q1

N = nq2 * nq3 * nq4 * ntheta;
P = zeros(N,3);
idx = 1;

for i2 = 1:nq2
    q2 = q2vec(i2);
    for i3 = 1:nq3
        q3 = q3vec(i3);
        for i4 = 1:nq4
            q4 = q4vec(i4);
            for it = 1:ntheta
                q1 = theta(it);
                Q = [q1, q2, q3, q4];
                S = Direct_kin(Q, L);
                P(idx, :) = S';
                idx = idx + 1;
            end
        end
    end
end
P = P(1:idx-1, :);
end
