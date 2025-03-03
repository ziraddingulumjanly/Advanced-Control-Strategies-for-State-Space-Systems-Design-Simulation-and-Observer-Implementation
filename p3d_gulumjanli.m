
close all; clear; clc

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

% Observer gain L 
L = [6.0; -40.0; -70.0]; 

% Integral Control Gains 
K = [1137.30 -318.69 -152.40];
Ki = -817.50 * 0.75;  
Ki2 = -817.50 * 0.75; 

% Observer-Based System Matrices
A_obsv = [A -B*K; L*C A-L*C-B*K];
B_obsv = [B*Ki, B*Ki2; B*Ki, B*Ki2]; 
C_obsv = [C, zeros(nr,nx)];

% Ensure correct input dimensions for lsim
U = [R, R]; 

% Create Observer-Based System
sys_obsv = ss(A_obsv, B_obsv, C_obsv, []);

% Simulate system response
Y_obsv = lsim(sys_obsv, U, T, X0);

% Plot the results
figure; hold on;
plot(T, R, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'Reference r(t)');
plot(T, Y_obsv, 'b', 'LineWidth', 1.5, 'DisplayName', 'System Output y(t)');
xlabel('Time [s]');
ylabel('Signals');
title('Simulation of Observer-Based Full-State Feedback System');
legend;
grid on;
