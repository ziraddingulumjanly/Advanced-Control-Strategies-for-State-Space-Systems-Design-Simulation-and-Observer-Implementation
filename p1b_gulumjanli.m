
% Define the original state-space matrices
A = [  0         1       0;
       0    -Cf/M   Ac/M;
       0  -2*Ac*beta/V  -2*Cl*beta/V];

B = [0;
     0;
     2*k*beta/V];

C = [1 0 0]; 

r =0; 

% Augment the system with an integrator
A_aug = [A, zeros(3,1);
        -C, 0];

B_aug = [B; 0];

% Define the desired closed-loop poles
desired_poles = [-20, -16, -12 + 12j, -12 - 12j];

% Compute the full-state feedback gain (K, Ki)
K_aug = place(A_aug, B_aug, desired_poles);

% Extract the gains
K = K_aug(1:3);
Ki = K_aug(4);

% External Load Force (Disturbance f = 500N)
B_f = [0; 1/M; 0; 0]; 
f_load = 500; 

% Define the closed-loop system with disturbance
A_cl = A_aug - B_aug * K_aug;

% Define simulation parameters
TSPAN = [0 3]; 
X0 = [0; 0; 0; 0]; 

% Define the system dynamics for ODE45
state_derivative = @(t, x) (A_cl * x + B_f * f_load);

% Solve the system using ODE45
[t, x] = ode45(state_derivative, TSPAN, X0);

% Compute system output y = C*x
y = C * x(:, 1:3)'; % Extracting only x1, x2, x3

% Plot response to disturbance
figure;
plot(t, y, 'r-', 'LineWidth', 2);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('System Output $y(t)$', 'Interpreter', 'latex', 'FontSize', 14);
title('Response to Disturbance: Full-State Feedback with Integration', 'Interpreter', 'latex', 'FontSize', 16);
grid off;

e = r-y(end);
disp(e)