function u = car(state,state_ref)
    %% Parameters
    L = 2.5;       % wheelbase [m]
     v0 = state(5);
     theta0 = state(3);
     phi0 = state(4);
phi0 = max(min(phi0, pi/6), -pi/6);

  %% Linearized system around (theta, v)
  

A = [  0,  0, -v0*sin(theta0),           0,             cos(theta0);
       0,  0,  v0*cos(theta0),           0,             sin(theta0);
       0,  0,          0,   (v0/L)*(1/cos(phi0)^2),     (1/L)*tan(phi0);
       0,  0,          0,               0,                     0;
       0,  0,          0,               0,                     0 ];

B = [  0,  0;
       0,  0;
       0,  0;
       0,  1;
       1,  0 ];

    %% Base Q and R
    Q_default =[1 0 0 0 0
                0 50 0 0 0 
                0 0 100 0 0 
                0 0 0 1 0 
                0 0 0 0 1];   %  for 5 states
    R_default = eye(2);   %  for 2 inputs

   

   %% Solve LQR
    K = lqr(A, B, Q_default, R_default);

    % Error state
    e = state - state_ref;

    % Control input
    u = -K * e;
    u(1) = max(min(u(1), .53), -.53); % فرمان
u(2) = max(min(u(2), 1), 0);     % شتاب
end