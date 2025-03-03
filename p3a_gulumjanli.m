clear; clc; close all;

% Original system matrices
A = [-0.14 0.33 -0.33;
     0.1 -0.28 0;
     0 1.7 -0.77];

B = [0; 0; -0.025];
Bv = [1; 0; 0];
C = [2 0 0];

% Define additional integrator states
nr = size(C,1);  % Number of outputs
nx = size(A,1);  % Number of states

% Construct augmented state matrices
A1 = [A, zeros(nx,nr), zeros(nx,nr);
     -C, zeros(nr,nr), eye(nr);
     zeros(nr,nx), eye(nr), zeros(nr,nr)];

B1 = [B; zeros(nr,1); zeros(nr,1)];
Br = [zeros(nx,nr); eye(nr); zeros(nr)];
Bv1 = [Bv; zeros(nr,nr); zeros(nr,nr)];
C1 = [C, zeros(nr,nr), zeros(nr,nr)];

% Check controllability
Mc = ctrb(A1, B1);
if rank(Mc) == size(A1,1)
    disp('System is controllable');
else
    disp('System is NOT controllable');
end

% Desired closed-loop eigenvalues
Lambda = [-1, -1+1j*0.25, -1-1j*0.25, -1+1j*0.5, -1-1j*0.5];

% Compute the state feedback gain
K1 = place(A1, B1, Lambda) ; 


% Extract gains
K = K1(:,1:nx);
Ki = K1(:, nx+1:nx+nr);
Ki2 = K1(:, nx+nr+1:end);

% Display the gain values
disp('K = '), disp(K);
disp('Ki = '), disp(Ki);
disp('Ki2 = '), disp(Ki2);

% Time settings
tfinal = 200;
T = linspace(0, tfinal, 10000)';  % Time vector

% Reference signal: Triangular wave
triangle = @(t) abs(mod((t+pi)/pi, 2)-1);
r = min(triangle(T/15), 0.75);

% Initial states 
X0 = zeros(5,1); 

% Closed-loop system
sys_cl = ss(A1 - B1*K1, Br, C1, []);

% Simulate response
Y = lsim(sys_cl, r, T, X0);

% Plot results
figure; hold on;
plot(T, r, '--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(T, Y, 'LineWidth', 1.5, 'DisplayName', 'System Response');
xlabel('Time [s]'); ylabel('Output');
title('System Response with Integral Control');
legend('Location', 'best'); grid on;
