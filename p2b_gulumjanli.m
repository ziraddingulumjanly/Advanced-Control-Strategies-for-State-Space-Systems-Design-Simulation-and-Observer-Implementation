
%% Define Transformed System Matrices
A = [-0.0667   0        0.0499   0;
      0       -0.0335   0        0;
      0.0499   0       -0.0499   0;
      0        0        0       -0.0251];

B = [0     0;
     0     0;
     0.0051  0.0051;
     0.0377 -0.0377];

C = [2.0000  0  0  0;
     0  0.1000  0  0];

%% Partition the Matrices
A11 = A(1:2, 1:2);
A12 = A(1:2, 3:4);
A21 = A(3:4, 1:2);
A22 = A(3:4, 3:4);

B1 = B(1:2, :); 
B2 = B(3:4, :);

C1 = C(:, 1:2);  

%% Check Observability
Obs_matrix = obsv(A22, C1 * A12);
rank_Obs = rank(Obs_matrix);
disp(['Observability Rank: ', num2str(rank_Obs)]);

if rank_Obs < size(A22,1)
    disp('Warning: System is not observable. Pole placement may fail.');
end

%% Compute Observer Gain L
desired_poles = [-4 + 0.5*1i, -4 - 0.5*1*i]; 

try
    L = place(A22', (C1 * A12)', desired_poles)'; 
catch
    disp('Warning: place() failed, switching to LQR observer gain.');
    Q = eye(size(A22));
    R = eye(size(C1 * A12,1));
    L = lqr(A22', (C1 * A12)', Q, R)'; % Alternative observer gain
end

%% Compute Reduced-Order Observer Matrices
M = A22 - L * C1 * A12;
N = B2 - L * C1 * B1;
P_matrix = ((A21 - L * C1 * A11) / C1) +M*L;

%% Display Results
disp('Reduced-Order Observer Matrices:');
disp('L = '); disp(L);
disp('M = '); disp(M);
disp('N = '); disp(N);
disp('P = '); disp(P_matrix);
