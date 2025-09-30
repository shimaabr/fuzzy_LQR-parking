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






