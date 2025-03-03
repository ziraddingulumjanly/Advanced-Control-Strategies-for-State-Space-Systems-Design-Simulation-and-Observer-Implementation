clc; clear; close all;

Ac = 150  ; 
V = 3000 ;   
M =0.5;           
beta = 7000 ; 
Cl = 1;    
Cf = 0.1 ;     
k = 20 ;    

% Define the original state-space matrices
A = [  0         1       0;
       0    -Cf/M   Ac/M;
       0  -2*Ac*beta/V  -2*Cl*beta/V];

B = [0;
     0;
     2*k*beta/V];

C = [1 0 0];  
D = 0;        

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

% Display results
disp('State Feedback Gain Matrix (K):');
disp(K);
disp('Integral Gain (Ki):');
disp(Ki);
