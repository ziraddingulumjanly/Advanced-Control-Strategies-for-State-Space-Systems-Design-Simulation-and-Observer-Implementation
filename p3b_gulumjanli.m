close all; clear; clc

% Simulation options
X0 = zeros(5,1); 
tfinal = 200;
T = linspace(0, tfinal, 10000)';

% Reference signal (triangular wave)
triangle = @(t) abs(mod((t+pi)/pi, 2)-1);
R = min(triangle(T/15), 0.75); % Reference signal

% System matrices
A = [-0.14 0.33 -0.33;
     0.1 -0.28 0;
     0 1.7 -0.77];
B = [0; 0; -0.025];
Bv = [1; 0; 0];
C = [2 0 0];

nx = size(A,1);
nr = size(C,1);

% Augmented system matrices
A1 = [[A, zeros(nx,nr), zeros(nx,nr)];
      [-C, zeros(nr,nr), eye(nr)];
      [zeros(nr,nx), eye(nr), zeros(nr,nr)]];
B1 = [B; zeros(nr,1); zeros(nr,1)];
Br1 = [zeros(nx,nr); eye(nr); zeros(nr,nr)];
Bv1 = [Bv; zeros(nr,1); zeros(nr,1)];
C1 = [C, zeros(nr,nr), zeros(nr,nr)];

% Desired eigenvalues
desired_poles = [-1, -1+0.25j, -1-0.25j, -1+0.5j, -1-0.5j];

% Compute state feedback gains
K1 = place(A1, B1, desired_poles);
K = K1(:, 1:nx);
Ki = K1(:, nx:nx+nr);
Ki2 = K1(:, nx+nr:end);

% Create closed-loop system
sys1 = ss(A1 - B1*K1, Br1, C1, []);

% Simulate system response
Y1 = lsim(sys1, R, T, X0);

% Plot results
figure; hold on;
plot(T, R, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'Reference r(t)');
plot(T, Y1, 'b', 'LineWidth', 1.5, 'DisplayName', 'System Output y(t)');
xlabel('Time [s]');
ylabel('Signals');
title('System Response with Integral Control');
legend;
grid on;
