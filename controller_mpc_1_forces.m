% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_1_forces(Q, R, T, N, ~)
% controller variables
persistent param forces_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, forces_optimizer] = init(Q, R, N);
end

% evaluate control action by solving MPC problem
[u_mpc, errorcode] = forces_optimizer(T);
if (errorcode ~= 1)
    warning('MPC1 infeasible');
end
p = u_mpc + param.p_sp;
end

function [param, forces_optimizer] = init(Q, R, N)
% get basic controller parameters
param = compute_controller_base_parameters;
% get terminal cost
[~, S, ~] = dlqr(param.A, param.B, Q, R);
% implement your MPC using Yalmip2Forces interface here
nx = size(param.A, 1);
nu = size(param.B, 2);
U = sdpvar(repmat(nu, 1, N - 1), ones(1, N - 1), 'full');
X = sdpvar(repmat(nx, 1, N), ones(1, N), 'full');
T0 = sdpvar(nx, 1, 'full');
X{1} = T0 - param.T_sp;
objective = 0;
constraints = [];
for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
    constraints = [constraints, param.Xcons(:, 1) <= X{k+1} <= param.Xcons(:, 2)];
    constraints = [constraints, param.Ucons(:, 1) <= U{k} <= param.Ucons(:, 2)];
    objective = objective + X{k}' * Q * X{k} + U{k}' * R * U{k};
end
objective = objective + X{N}' * S * X{N};
codeoptions = getOptions('solvername');
forces_optimizer = optimizerFORCES(constraints, objective, codeoptions, T0, U{1});
end
