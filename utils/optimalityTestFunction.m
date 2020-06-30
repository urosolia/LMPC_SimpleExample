function [lambdaMult] = optimalityTestFunction( x, u, N, X, U, A, B, Q, R )
% Chech LICQ

% Initialize QP Matrices (Notice that here different convention z = [x_0, \ldots, x_{N-1}, u_0, \ldots, u_{N-1}])
I = eye(size(A,2));
A_big = kron(eye(N+1), I) + kron(diag(ones(N,1),-1), -A);
B_big = kron(eye(N), -B);
B_big = [zeros(size(A,1), size(B_big,2)); 
         B_big];
GeqNotOrdered = [A_big(:,1:end-2), B_big]; % here remove one column of x_N because not needed. In this formulation x_t = [x_t, \ldots, x_{t+N-1}];

% Reorder Columns to match paper
n = size(A,1);
d = size(B,2);
Geq = zeros(size(GeqNotOrdered,1));
for i = 1:N
    Geq(:,(i-1)*(n+d)+[1:n])     = GeqNotOrdered(:, (i-1)*n+[1:n]);
    Geq(:,(i-1)*(n+d)+n+[1:d])   = GeqNotOrdered(:, n*N+(i-1)*d+[1:d]);
end
constrMat   = blkdiag(kron(eye(N),blkdiag(X.A,U.A)));
constrVec   = [kron(ones(N,1),[X.b;U.b])];

% Initialize Vector used to compute the gradient
gradientMat = blkdiag(kron(eye(N),blkdiag(Q,R)));

% Initialize loop quantities
lambdaMult{1} = [];
gradCost{1}   = [];
testPassed    = 1;
counter       = [];
% Notice that the loop start from t = 2 as the condition does not have to
% be satified at time t = 0 (and matlab index start from 1)
for t = 2:(size(u,2)-(N-1))
    % Select N-1 step solutiob
    xt = x(:, t:t+N-1);
    ut = u(t:t+N-1);
    zt = [];
    for i = 1:N
        zt = [zt, xt((i-1)*n+[1:n]), ut((i-1)*d+[1:d])];
    end
    zt = zt';
    
    % Select active constraints
    tol = 0.0001; 
    active = constrMat*zt>= constrVec-tol;
    
    % Compute Matric of equality and active inequalities
    eqAndActIneq = [Geq; constrMat(active,:)];    
    
    % Check if matrix of equality constraints and active inequality
    if (rank(eqAndActIneq') < size(eqAndActIneq',2))
        testPassed = 0; 
        counter = [counter; t];
    end
end

%% Print to screen results
if testPassed == 1
    disp('LICQ satified. The LMPC has converges to the optimal solution')
else
    disp('Sufficient conditions not satisfied: try a longer horizon')
    disp(['Not satysfied at time steps: ', num2str(counter')])
    
end

%% Now Set N = N+1 and compute the LMPC horizon
% Initialize QP Matrices (Notice that here different convention z = [x_0, \ldots, x_{N-1}, u_0, \ldots, u_{N-1}])
N = N + 1;
I = eye(size(A,2));
A_big = kron(eye(N+1), I) + kron(diag(ones(N,1),-1), -A);
B_big = kron(eye(N), -B);
B_big = [zeros(size(A,1), size(B_big,2)); 
         B_big];
GeqNotOrdered = [A_big(:,1:end-2), B_big]; % here remove one column of x_N because not needed. In this formulation x_t = [x_t, \ldots, x_{t+N-1}];

% Reorder Columns to match paper
n = size(A,1);
d = size(B,2);
Geq = zeros(size(GeqNotOrdered,1));
for i = 1:N
    Geq(:,(i-1)*(n+d)+[1:n])     = GeqNotOrdered(:, (i-1)*n+[1:n]);
    Geq(:,(i-1)*(n+d)+n+[1:d])   = GeqNotOrdered(:, n*N+(i-1)*d+[1:d]);
end
constrMat   = blkdiag(kron(eye(N),blkdiag(X.A,U.A)));
constrVec   = [kron(ones(N,1),[X.b;U.b])];

% Initialize Vector used to compute the gradient
gradientMat = blkdiag(kron(eye(N),blkdiag(Q,R)));

for t = 1:(size(u,2)-(N-1))
    % Select N-1 step solutiob
    xt = x(:, t:t+N-1);
    ut = u(t:t+N-1);
    zt = [];
    for i = 1:N
        zt = [zt, xt((i-1)*n+[1:n]), ut((i-1)*d+[1:d])];
    end
    zt = zt';
    
    % Select active constraints
    tol = 0.0001; 
    active = constrMat*zt>= constrVec-tol;
    
    % Compute Matric of equality and active inequalities
    eqAndActIneq = [Geq; constrMat(active,:)];  
    
    % Compute gradient
    gradCost{t} = -2*gradientMat*zt;
    
    % Compute multipliers
    lambdaMult{t} = mldivide(eqAndActIneq',gradCost{t});

end

