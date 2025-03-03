% Bode Plot Analysis: Comparing Systems from Part (a) and (c)

close all; clear; clc

A = [-0.14 0.33 -0.33;
     0.1 -0.28 0;
     0 1.7 -0.77];
B = [0; 0; -0.025];
Bv = [1; 0; 0]; 
C = [2 0 0];

nx = size(A,1);
nr = size(C,1);

% Observer gain 
L = [6.0; -40.0; -70.0];

% Full-State Feedback Gains 
K = [1137.30 -318.69 -152.40];
Ki = -817.50 * 0.75;  
Ki2 = -817.50 * 0.75;

% System from Part (a) (No Observer)
A_a = [A, -B*Ki, -B*Ki2; -C, zeros(nr,nr), eye(nr); zeros(nr,nx), eye(nr), zeros(nr,nr)];
Bv_a = [Bv; zeros(2,1)]; 
C_a = [C, zeros(nr,nr), zeros(nr,nr)];
sys_a = ss(A_a, Bv_a, C_a, []);

A_c = [A -B*K; L*C A-L*C-B*K];
Bv_c = [Bv; zeros(nx,1)]; 
C_c = [C, zeros(nr,nx)];
sys_c = ss(A_c, Bv_c, C_c, []);

% Bode Plot Comparison
figure;
bode(sys_a, 'r', sys_c, 'b--'); % Red for Part (a), Blue Dashed for Part (c)
grid on;
legend('Full-State Feedback (No Observer)', 'Observer-Based Feedback');
title('Bode Plot: Disturbance \( v \) to Output \( y \)');
