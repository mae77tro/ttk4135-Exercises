% Problem data
A = [-1 0.4 0.8; 1 0 0; 0 1 0]; nx = size(A,2);
B = [1; 0; 3]; nu = size(B,2);

x0 = [1;0;0];       % Initial state
xdes = [7;2;-6];    % Desired end state

N = 30;             % Horizon

% Define inputs to quadprog:
H = eye(nu*N);
f = zeros(nu*N,1);
Aeq = zeros(nx,nu*N);
for k = 1:N
    ind = N+1-k;
   Aeq(:,ind) = (A^(k-1))*B;
end

beq = xdes-(A^N)*x0;



...
    
% Solve QP. 
% Here the output from quadprog is corresponding to 'Batch approach 2 (reduced space)'. If you choose 'Batch approach 1 (full space)', 
% then you must extract the optimal inputs from the output from quadprog, and save in a variable U.
U = quadprog(H,[],[],[],Aeq,beq)

% Function that MAY be used if you use the reduced space approach.
function [Sx, Su] = genMPCprob(A,B,N)
% Generate reduced space matrices for MPC QP problem
% Inputs: 
%   A, B        System matrices in discrete time system: x+ = A x + B u
%   N           Control horizon length (degrees of freedom)
% 
% Outputs:
%   Sx          State predictions: 
%   Su              [x_1 x_2 ... x_N] = Sx*x_0 + Su*[u_0 u_1 ... u_N-1]

% 12/02/2018 Lars Imsland

% Define predictions:
% [x_1 x_2 ... x_N] = Sx*x_0 + Su*[u_0 u_1 ... u_N-1]

nx = size(A,2);
nu = size(B,2);

Sx = [eye(nx);A];
Su = zeros(N*nx,N*nu);
Su(1:nx,1:nu) = B;
for i = 2:N,
    Sx = [Sx;Sx((i-1)*nx+1:i*nx,:)*A];
    for j=1:i,
        Su((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu) = ...
            Sx((i-j)*nx+1:(i-j+1)*nx,1:nx)*B;
    end
end
Sx = Sx(nx+1:end,:); % remove first I
end

x = zeros(nx,N+1);
x(:,1) = x0;
for i = 2:N+1
    x(:,i) = A*x(:,i-1) + B*U(i-1)
end
figure(2)
subplot(2,1,1)
plot(x(1,:))
subplot(2,1,2)
plot(U)