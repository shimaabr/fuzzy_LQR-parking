# fuzzy_LQR-parking

This project implements a hybrid Fuzzy-LQR controller for motorcycle parking .
The system uses a bicycle model to represent vehicle dynamics with the state vector:
[x, y, theta, phi, v], where x and y are the global position coordinates, theta is the vehicle heading, phi is the steering angle, and v is speed.

The key feature of this controller is that the LQR weighting matrices Q and R are adaptively tuned using a fuzzy logic system.
This allows the controller to handle nonlinearities and different parking scenarios more effectively compared to a standard LQR.

## Dynamic System Model

In this project I use a simplified model like a motorcycle. The dynamics of the system are as follows:

[Reference: Meijaard et al., “Linearized Dynamics Equations for the Balance and Steer of a Bicycle,” 2007].

## System Dynamics

$\dot{x} = v \cos(\theta)$,  
$\dot{y} = v \sin(\theta)$,  
$\dot{\theta} = \frac{v}{L} \tan(\phi)$,  
$\dot{\phi} = \dot{\phi}_{\text{control}}$,  
$\dot{v} = a$ 

where `L` is the wheelbase of the motorcycle.

and x y theta and phi are the states of system which represent 

- `x` : longitudinal position  
- `y` : lateral position  
- `theta` : vehicle heading (yaw angle)  
- `phi` : steering angle  
- `v` : longitudinal speed  

The dynamics of this system are **nonlinear**.

Control Inputs

- `phi_dot` : steering rate  
- `a` : longitudinal acceleration  

 Outputs

Since the goal is to control the position and angle of the vehicle (for example, for car parking), the outputs are:

- `x` : longitudinal position  
- `y` : lateral position  
- `θ` : heading (yaw) angle

## Fuzzy_LQR controller

In LQR design, the performance of the controller heavily depends on the **weighting matrices** `Q` (for states) and `R` (for inputs).  

Choosing fixed Q and R for a nonlinear system is difficult, because the “best” values change depending on the system's state (e.g., speed, steering angle).  
  Fuzzy logic can **adaptively adjust Q and R** based on the current state of the system.
  This way, the LQR controller performs well across all operating.

  - **Fuzzy**: finding best Q and R acording to the error of x and theta

  - **LQR**: Optimal control for linear systems with specified weights on states and inputs.

### fuzzy 
A**Fuzzy Controller** is a smart control system that works like human thinking.  
It does not need exact math equations.  
Instead, it uses simple rules, for example: "If speed is high, slow down."

I used a **Fuzzy Controller** to find the best  scolar values for `Q` and `R`, which are the weights for the state error and control input in the LQR.  
This allows the controller to adapt based on the current position and heading errors of the vehicle.  

The controller uses **two inputs**:  
1. `error_x` – the longitudinal position error, range: [-3, 3]  
2. `error_theta` – the heading (yaw) error, range: [-0.7854 0.7854] 

Each input has **5 membership functions**:  
- `PL` = positive larege
- `ps` = positive small
- `z`  = zero  
- `NS` = negetive small   
- `NL` = negetive large

And outputs are the LQR scalor weight Q0 and R0:  
-`R` – weight for control input, range: [0.1, 10]  
-`Q` – weight for state error, range: [1,30]  

Each output also has **5 membership functions**: VL,L,M,S,VS.( very large, large, medium, small, very small)


And it has 25 rules as follow :

Rules Table

## Fuzzy Rules for Adaptive LQR

The controller uses 25 fuzzy rules to map the inputs (`error_x` and `error_theta`) to outputs (`R` and `Q`).  
Below is a detailed table with meanings of each input and output:

| Rule | error_x (position) | error_theta (heading) | R (control weight) | Q (state weight) |
|------|------------------|---------------------|------------------|----------------|
| 1    | PL               | PL                  | VS               | VL             |
| 2    | PL               | PS                  | L                | L              |
| 3    | PL               | Z                   | M                | M              |
| 4    | PL               | NS                  | M                | L              |
| 5    | PL               | NL                  | L                | VL             |
| 6    | PS               | PL                  | L                | L              |
| 7    | PS               | PS                  | M                | M              |
| 8    | PS               | Z                   | S                | L              |
| 9    | PS               | NS                  | VS               | VL             |
| 10   | PS               | NL                  | VS               | VL             |
| 11   | Z                | PL                  | M                | M              |
| 12   | Z                | PS                  | S                | L              |
| 13   | Z                | Z                   | S                | M              |
| 14   | Z                | NS                  | VS               | L              |
| 15   | Z                | NL                  | VS               | VL             |
| 16   | NS               | PL                  | S                | L              |
| 17   | NS               | PS                  | S                | M              |
| 18   | NS               | Z                   | VS               | L              |
| 19   | NS               | NS                  | VS               | VL             |
| 20   | NS               | NL                  | VS               | VL             |
| 21   | NL               | PL                  | VS               | VL             |
| 22   | NL               | PS                  | VS               | VL             |
| 23   | NL               | Z                   | VS               | VL             |
| 24   | NL               | NS                  | VS               | VL             |
| 25   | NL               | NL                  | VS               | VL             |


Q represents the system’s sensitivity to errors.

The bigger the error, the larger Q becomes, so the system tries to correct the error faster.

R

R represents the strength or intensity of the control action.

When the error is small, R is large → system gives smooth and gentle control.

### LQR Controller
**LQR (Linear Quadratic Regulator)** computes optimal control inputs for linear or linearized systems.
- Minimizes **tracking error** and **control effort** simultaneously

and cause motor bicycle has Nonlinear kinematics so we should linerize it

\[
\dot{x} = A x + B u
\]


I computed the **Jacobian matrices** with respect to the states and inputs:  

- **State Jacobian** → gives matrix \(A\)  
- **Input Jacobian** → gives matrix \(B\)

A matrix:
\[
A = 
A = [ 
  0, 0, -v0*sin(theta0), 0, cos(theta0); 
  
  0, 0,  v0*cos(theta0), 0, sin(theta0); 
  
  0, 0, 0, (v0/L)*(1/cos(phi0)^2), tan(phi0)/L; 
  
  0, 0, 0, 0, 0; 
  
  0, 0, 0, 0, 0 
]

 B = [
         0 0;
         0 0;
         0 0;
         0 1;
         1 0];

and get v0 and theta0 and phi0 as follow
   v0 = state(5);
     theta0 = state(3);
     phi0 = state(4);
    phi0 = max(min(phi0, pi/6), -pi/6);

``matlab

 function u = car(state, R_fuzzy, Q_fuzzy,state_ref)
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


    %%  Q and R default
    Q_default = eye(5);   %  for 5 states
    R_default = eye(2);   %  2 inputs

    %% Adaptive scaling from fuzzy outputs
    Q_new = Q_fuzzy * Q_default;
    R_new = R_fuzzy * R_default;

     %% Solve LQR safely
        K = lqr(A, B, Q_new, R_new);
   


    % Error state
    e = state - state_ref;

    % Control input
     u = -K * e;
    u(1) = max(min(u(1), .53), -.53); % فرمان
    u(2) = max(min(u(2), 1), 0);     % شتاب
    
     end

 


 


## result 

In this example, the target position for parking is set as:

x_target = 2, y_target = 2 theta=0

The vehicle’s current position (x, y,theta,v,phi) starts from(0,0,.3,.5,0) cause i set initial integrated block for them as followes.


## result of fuzzy_LQR

final_x=2.651  final_y=   final_theta=

## Comparison: Fuzzy-LQR vs. Pure LQR

We compare the performance of **Fuzzy-LQR** with a **pure LQR controller**  
(using `Q = 10*eye(5)` and `R = 0.1*eye(2)`).

The reference target is:

Final Results

| Controller  | x_final | y_final | theta_final (rad) | theta_final (deg) |
|-------------|---------|---------|-------------------|-------------------|
| Fuzzy-LQR   | 2.06   | 1.704   | 0.4276            | 24.5°             |11
| Pure LQR    | 2.053   | 1.738   | 0.3898            | 22.3°             |



 Position Error
Fuzzy-LQR:
error_x = 0.028 , error_y = -0.137
error_pos ≈ sqrt(0.028^2 + 0.137^2) = 0.14 m

Pure LQR:
error_x = 0.053 , error_y = -0.262
error_pos ≈ sqrt(0.053^2 + 0.262^2) = 0.27 m


Fuzzy-LQR reduces the position error by almost half compared to Pure LQR.


Fuzzy-LQR  theta_error: 0.4276 rad ≈ 24.5°
Pure LQR   theta_error: 0.3898 rad ≈ 22.3°

Pure LQR has slightly better orientation accuracy (≈ 2° improvement).

Since precise position is usually more crucial in vehicle parking , **Fuzzy-LQR is the better choice overall.

# How to Run
1. Open `parking.slx` in Simulink.  
2. Run the simulation with `car.m` for Fuzzy-LQR or `car_LQR.m` for standard LQR.
3. The fuzzy logic system is saved in **`parking.fis`**. Make sure this file is in the same folder as `parking.slx`  before running the simulations. This ensures that MATLAB/Simulink can correctly load the fuzzy rules for adaptive Q/R tuning.
















