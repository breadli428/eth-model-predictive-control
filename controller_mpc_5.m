% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
%   d: Disturbance matrix, dimension (3,N)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_5(Q, R, T, N, d)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q, R, N);
end

% evaluate control action by solving MPC problem
[u_mpc, errorcode] = yalmip_optimizer([T, d]);
if (errorcode ~= 0)
    warning('MPC5 infeasible');
end
p = u_mpc + param.p_sp;
end

function [param, yalmip_optimizer] = init(Q, R, N)
% get basic controller parameters
param = compute_controller_base_parameters;
% get terminal cost
[~, S, ~] = dlqr(param.A, param.B, Q, R);
% get terminal set
[A_x, b_x] = compute_X_LQR(Q, R);
% implement your MPC using Yalmip here
nx = size(param.A, 1);
nu = size(param.B, 2);
U = sdpvar(repmat(nu, 1, N - 1), ones(1, N - 1), 'full');
X = sdpvar(repmat(nx, 1, N), ones(1, N), 'full');
v = 1000;
epsilon = sdpvar(repmat(nx, 1, N), ones(1, N), 'full');
T0 = sdpvar(nx, 1, 'full');
X{1} = T0 - param.T_sp;
d = sdpvar(repmat(nx, 1, N), ones(1, N), 'full');
objective = 0;
constraints = [];
constraints = [constraints, param.Xcons(:, 1) - epsilon{1} <= X{1} <= param.Xcons(:, 2) + epsilon{1}];
constraints = [constraints, epsilon{1} >= 0];
for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k} + param.Bd * d{k}];
    constraints = [constraints, param.Xcons(:, 1) - epsilon{k+1} <= X{k+1} <= param.Xcons(:, 2) + epsilon{k+1}];
    constraints = [constraints, epsilon{k+1} >= 0];
    constraints = [constraints, param.Ucons(:, 1) <= U{k} <= param.Ucons(:, 2)];
    objective = objective + X{k}' * Q * X{k} + U{k}' * R * U{k} + v * norm(epsilon{k}, 1);
end
constraints = [constraints, A_x * X{N} <= b_x];
objective = objective + X{N}' * S * X{N} + v * norm(epsilon{N}, 1);
ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
yalmip_optimizer = optimizer(constraints, objective, ops, [T0, d{:}], U{1});
end