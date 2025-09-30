# fuzzy_LQR-parking

This project implements a hybrid Fuzzy-LQR controller for motorcycle parking .
The system uses a bicycle model to represent vehicle dynamics with the state vector:
[x, y, theta, phi, v], where x and y are the global position coordinates, theta is the vehicle heading, phi is the steering angle, and v is speed.

The key feature of this controller is that the LQR weighting matrices Q and R are adaptively tuned using a fuzzy logic system.
This allows the controller to handle nonlinearities and different parking scenarios more effectively compared to a standard LQR.

# Dynamic System Model

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

I used a **Fuzzy Controller** to find the best values for `Q` and `R`, which are the weights for the state error and control input in the LQR.  
This allows the controller to adapt based on the current position and heading errors of the vehicle.  

The controller uses **two inputs**:  
1. `error_x` – the longitudinal position error, range: [-10, 10]  
2. `error_theta` – the heading (yaw) error, range: [-0.5236, 0.5236]  

Each input has **5 membership functions**:  
- `NL` = Negative Large  
- `NS` = Negative Small  
- `Z`  = Zero  
- `PS` = Positive Small  
- `PL` = Positive Large  

And outputs are the LQR weights:  
-`R` – weight for control input, range: [0.1, 10]  
-`Q` – weight for state error, range: [10, 100]  

Each output also has **5 membership functions**: NL, NS, Z, PS, PL.


And it has 25 rules as follow :

Rules Table

| Rule | errorx | error_theta | R  | Q  |
|------|--------|-------------|----|----|
| 1    | VS     | VS          | VL | VS |
| 2    | VS     | S           | L  | S  |
| 3    | VS     | M           | M  | M  |
| 4    | VS     | L           | S  | L  |
| 5    | VS     | VL          | VS | VL |
| 6    | S      | VS          | L  | S  |
| 7    | S      | S           | M  | M  |
| 8    | S      | M           | S  | L  |
| 9    | S      | L           | VS | VL |
| 10   | S      | VL          | VS | VL |
| 11   | M      | VS          | M  | M  |
| 12   | M      | S           | S  | L  |
| 13   | M      | M           | S  | M  |
| 14   | M      | L           | VS | L  |
| 15   | M      | VL          | VS | VL |
| 16   | L      | VS          | S  | L  |
| 17   | L      | S           | S  | M  |
| 18   | L      | M           | VS | L  |
| 19   | L      | L           | VS | VL |
| 20   | L      | VL          | VS | VL |
| 21   | VL     | VS          | VS | VL |
| 22   | VL     | S           | VS | VL |
| 23   | VL     | M           | VS | VL |
| 24   | VL     | L           | VS | VL |
| 25   | VL     | VL          | VS | VL |

Q

Q represents the system’s sensitivity to errors.

The bigger the error, the larger Q becomes, so the system tries to correct the error faster.

R

R represents the strength or intensity of the control action.

When the error is small, R is large → system gives smooth and gentle control.

### LQR Controller
**LQR (Linear Quadratic Regulator)** computes optimal control inputs for linear or linearized systems.
- Minimizes **tracking error** and **control effort** simultaneously

and cause motor bicycle has Nonlinear kinematics so Linearized  it around reference point

\[
\dot{x} = A x + B u
\]


We computed the **Jacobian matrices** with respect to the states and inputs:  

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

 B = [0 0;
         0 0;
         0 0;
         0 1;
         1 0];

the refrence point is 

 v0 = 1;       
 theta0 = 1  
  phi0=0;

## simulink 
I implement it in simulink matlab use fuzzy toolbox                  and           code for LQR controller


## result 

In this example, the target position for parking is set as:

x_target = 2, y_target = 2

The vehicle’s current position (x, y) starts from(0,0) cause i set initial x and y in integrated block to 0 and moves toward the target.

Using the Fuzzy Logic Controller and/or LQR, the vehicle gradually approaches the target.

Over time, both x and y converge to 2, showing that the parking control works correctly.

<img width="481" height="393" alt="image" src="https://github.com/user-attachments/assets/4cf3034f-14f4-4a53-a286-e57589ac0f3d" />

<img width="481" height="393" alt="image" src="https://github.com/user-attachments/assets/6167b7fa-29df-4521-a614-9f86f1269801" />


and its the out put of fuzzy 

<img width="875" height="619" alt="image" src="https://github.com/user-attachments/assets/a89ea88c-7525-4ff9-8885-0d487d7be877" />









