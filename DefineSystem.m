function [A, B, U, X, Q, R] = DefineSystem(example)
% Set cost function
Q = [1, 0;
     0, 1];
  
R = 1;
% Define the system matrices
A = [1  1;
	 0  1];

B = [0;
     1]; 


% Define the input constraint set U
if example == 1
    Val_U = 2.0; 
else
    Val_U = 1.5;
end
U = Polyhedron([Val_U; -Val_U]);

% Define the Recovery input constraint set R
% Define the state constraint set X
x_max = 15.0;
v_max = 15.0;
X = Polyhedron([x_max v_max; x_max -v_max; -x_max -v_max; -x_max v_max]);
end