clc; clear; close all;

%% Define Given System Matrices (From Example 2.9)
A = [-0.0499  0.0499  0       0;
      0.0499 -0.0667  0       0;
      0       0      -0.0251  0;
      0       0       0       -0.0335];

B = [0.00510  0.00510;
     0        0;
     0.0377  -0.0377;
     0        0];

C = [0 2 0 0;
     0 0 0 0.1];

%% Define the Permutation Matrix (Reordering States)

P = [ 0 1 0 0;
      0 0 0 1;
      1 0 0 0;
      0 0 1 0];

% Apply Similarity Transformation
A_transformed = P * A * inv(P);
B_transformed = P * B;
C_transformed = C * inv(P);

%% Display Results
disp('Transformed A matrix:');
disp(A_transformed);
disp('Transformed B matrix:');
disp(B_transformed);
disp('Transformed C matrix:');
disp(C_transformed);

