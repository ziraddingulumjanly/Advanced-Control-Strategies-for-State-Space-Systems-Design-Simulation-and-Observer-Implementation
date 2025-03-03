
close all; clear; clc

% Simulation options
X0 = zeros(6,1); 
tfinal = 200;
T = linspace(0, tfinal, 10000)';

% Reference signal 
triangle = @(t) abs(mod((t+pi)/pi, 2)-1);
R = min(triangle(T/15), 0.75);

% System matrices
A = [-0.14 0.33 -0.33;
     0.1 -0.28 0;
     0 1.7 -0.77];
B = [0; 0; -0.025];
C = [2 0 0];

nx = size(A,1);
nr = size(C,1);

% Adjusted Observer gain L (same as previous)
L = [6.0; -40.0; -70.0];  % Ensures fast convergence

K = [1137.30 -318.69 -152.40];
Ki = -817.50 * 0.75;  
Ki2 = -817.50 * 0.75; 

% Augmented System Matrices for Observer
A_obsv = [A -B*K; L*C A-L*C-B*K];
B_obsv = [B*Ki, B*Ki2; B*Ki, B*Ki2]; 
C_obsv = [C, zeros(nr,nx)];

U = [R, R]; % Applying reference to both integrator channels

% Create Observer-Based System
sys_obsv = ss(A_obsv, B_obsv, C_obsv, []);

% Simulate system response
Y_obsv = lsim(sys_obsv, U, T, X0);

% Plot results
figure; hold on;
plot(T, R, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'Reference r(t)');
plot(T, Y_obsv, 'b', 'LineWidth', 1.5, 'DisplayName', 'System Output y(t)');
xlabel('Time [s]');
ylabel('Signals');
title('Optimized Observer-Based Full-State Feedback with Integral Control');
legend;
grid off;
