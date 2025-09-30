function u = car(state, R_fuzzy, Q_fuzzy,state_ref)
% Adaptive Q/R LQR controller
% Inputs:
%   x         - 5x1 state vector [x; y; theta; phi; v]
%   R_fuzzy   - scalar weight from fuzzy for R
%   Q_fuzzy   - scalar weight from fuzzy for Q
% Output:
%   u         - 2x1 control vector [a_cmd; phi_dot_cmd]

    %#codegen

    %% Parameters
    L = 2.5;       % wheelbase [m]
    v0 = 1;        % operating velocity
    theta0 = .2;    % operating orientation
    phi0=0;

    %% Linearized system around (theta0, v0)
    A = [0 0 -v0*sin(theta0) 0 cos(theta0);
         0 0  v0*cos(theta0) 0 sin(theta0);
         0 0  0 (v0/L)* (1/cos(phi0)^2) tan(phi0)/L;
         0 0  0 0 0;
         0 0  0 0 0];

    B = [0 0;
         0 0;
         0 0;
         0 1;
         1 0];

    %% Base Q and R
    Q_default = eye(5);   % 5x5, for 5 states
    R_default = eye(2);   % 2x2, for 2 inputs

    %% Adaptive scaling from fuzzy outputs
    Q_new = Q_fuzzy * Q_default;
    R_new = R_fuzzy * R_default;

   %% Solve LQR
    K = lqr(A, B, Q_new, R_new);

    % Error state
    e = state - state_ref;

    % Control input
    u = -K * e;
end
