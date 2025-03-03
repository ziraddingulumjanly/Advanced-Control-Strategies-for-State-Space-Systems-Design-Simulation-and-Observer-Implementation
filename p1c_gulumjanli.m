
% Define the original state-space matrices
A = [  0         1       0;
       0    -Cf/M   Ac/M;
       0  -2*Ac*beta/V  -2*Cl*beta/V];

B = [0;
     0;
     2*k*beta/V];

C = [1 0 0];  % Output matrix

% Define the external disturbance input 
B_f = [0; 1/M; 0];

%%Case 1: Full-State Feedback WITHOUT Integration


% Define the desired closed-loop poles (without -16)
desired_poles_noInt = [-20, -12 + 12j, -12 - 12j];

% Compute full-state feedback gain (K) without integration
K_noInt = place(A, B, desired_poles_noInt);

% Closed-loop system without integration
A_cl_noInt = A - B * K_noInt;

%% Case 2: Full-State Feedback WITH Integration
% Augment the system with an integrator
A_aug = [A, zeros(3,1);
        -C, 0];

B_aug = [B; 0];

% Define the desired closed-loop poles (with integration)
desired_poles_Int = [-20, -12 + 12j, -12 - 12j, -16]; % Removed -16 from previous case

% Compute full-state feedback gain (K, Ki)
K_aug = place(A_aug, B_aug, desired_poles_Int);

% Extract the gains
K_Int = K_aug(1:3);
Ki_Int = K_aug(4);

% Closed-loop system with integration
A_cl_Int = A_aug - B_aug * K_aug;

%% Simulation Setup
TSPAN = [0 2]; 
X0 = [0; 0; 0]; 
X0_aug = [0; 0; 0; 0]; 

r = 1; 
f_load = 500;

%% Simulating Step Response for Reference Input (r = 1)
% Define step response function
step_response_noInt = @(t, x) (A_cl_noInt * x + B * r);
step_response_Int = @(t, x) (A_cl_Int * x + [B; 0] * r);

% Solve using ODE45
[t_noInt, x_noInt] = ode45(step_response_noInt, TSPAN, X0);
[t_Int, x_Int] = ode45(step_response_Int, TSPAN, X0_aug);

% Compute system output y = C*x
y_noInt = C * x_noInt';
y_Int = C * x_Int(:, 1:3)';

% Plot Step Response Comparison
figure;
plot(t_noInt, y_noInt, 'r-', 'LineWidth', 2);
hold on;
plot(t_Int, y_Int, 'b-', 'LineWidth', 2);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('System Output $y(t)$', 'Interpreter', 'latex', 'FontSize', 14);
title('Step Response: Full-State Feedback with and without Integration', 'Interpreter', 'latex', 'FontSize', 16);
legend('Without Integration', 'With Integration', 'Interpreter', 'latex', 'FontSize', 12);
grid off

%% Simulating Response to Disturbance (f = 500N)
% Define disturbance response function
disturbance_response_noInt = @(t, x) (A_cl_noInt * x + B_f * f_load);
disturbance_response_Int = @(t, x) (A_cl_Int * x + [B_f; 0] * f_load);

% Solve using ODE45
[t_noInt_f, x_noInt_f] = ode45(disturbance_response_noInt, TSPAN, X0);
[t_Int_f, x_Int_f] = ode45(disturbance_response_Int, TSPAN, X0_aug);

% Compute system output y = C*x
y_noInt_f = C * x_noInt_f';
y_Int_f = C * x_Int_f(:, 1:3)';

% Plot Disturbance Response Comparison
figure;
plot(t_noInt_f, y_noInt_f, 'r-', 'LineWidth', 2);
hold on;
plot(t_Int_f, y_Int_f, 'b-', 'LineWidth', 2);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('System Output $y(t)$', 'Interpreter', 'latex', 'FontSize', 14);
title('Response to Disturbance: Full-State Feedback with and without Integration', 'Interpreter', 'latex', 'FontSize', 16);
legend('Without Integration', 'With Integration', 'Interpreter', 'latex', 'FontSize', 12);
grid off;
