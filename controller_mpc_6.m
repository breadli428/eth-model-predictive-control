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

function p = controller_mpc_6(Q, R, T, N, ~)
% controller variables
persistent param yalmip_optimizer est

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q, R, N);
end

if isempty(est)
    est = [T; zeros(3,1)];
end
d_hat = est(4:6);
H = eye(size(param.C_ref));
Cd = zeros(size(param.C_ref));
T_sp = (H * param.C_ref) \ (param.b_ref - H * Cd * d_hat);
p_sp = param.B \ ((eye(size(param.A)) - param.A) * T_sp - param.Bd * d_hat);

% evaluate control action by solving MPC problem
[u_mpc, errorcode] = yalmip_optimizer([T, T_sp, p_sp]);
if (errorcode ~= 0)
    warning('MPC6 infeasible');
end
p = u_mpc + p_sp;

% observer update
A_aug = [param.A, param.Bd; zeros(size(param.A)), eye(size(param.Bd))];
B_aug = [param.B; zeros(size(param.B))];
Cd = zeros(size(param.C_ref));
C_aug = [param.C_ref, Cd];
P = [0.0, 0.0, 0.0, 0.8, 0.8, 0.8];
L = place(A_aug', C_aug', P)';
y = T;
est = A_aug * est + B_aug * p + L * (y - C_aug * est);
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
T0 = sdpvar(nx, 1, 'full');
T_sp = sdpvar(nx, 1, 'full');
p_sp = sdpvar(nu, 1, 'full');
Ucons = param.Pcons - repmat(p_sp, 1, 2);
Xcons = param.Tcons - repmat(T_sp, 1, 2);
X{1} = T0 - T_sp;
objective = 0;
constraints = [];
for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
    constraints = [constraints, Xcons(:, 1) <= X{k+1} <= Xcons(:, 2)];
    constraints = [constraints, Ucons(:, 1) <= U{k} <= Ucons(:, 2)];
    objective = objective + X{k}' * Q * X{k} + U{k}' * R * U{k};
end
constraints = [constraints, A_x * X{N} <= b_x];
objective = objective + X{N}' * S * X{N};
ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
yalmip_optimizer = optimizer(constraints, objective, ops, [T0, T_sp, p_sp], U{1});
end