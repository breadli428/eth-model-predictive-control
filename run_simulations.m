%% Init
clear; clc; close all;
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')
T_sp = [25, -42, -18.5]';
T0_1 = T_sp + [-2.25, 1.75, 0.75]';
T0_2 = T_sp + [1.5, 2.75, -0.25]';
% dT0_exmaple = ...
% T0_example = ...


%% Example
% figure; set(gcf, 'WindowStyle' ,'docked');
% clear persisten variables of function controller_example
% clear controller_example
% execute simulation
% [T,~,~,t] = simulate_building(T0_example, @controller_example);


%% Unconstrained optimal control
disp('Unconstraint optimal control');

% Task 5: Uncontrolled system
figure('Position', [0, 0, 1000, 1000])
simulate_building(T_sp);

% pause;


%%
% Task 6: Tuning of LQR on first initial condition
[Q, R] = heuristic_LQR_tuning(2500, T0_1, T_sp, scen1);

% pause;


%%
% Task 7
T0 = T0_1;
figure('Position', [0, 0, 1000, 1000])
clear controller_lqr
simulate_building(T0, @controller_lqr, Q, R, scen1);

influence_inspection = true; % Inspect influence of Q and R

if influence_inspection
    figure('Position', [0, 0, 1000, 1000])
    scale = [0.1, 1, 10];
    for s = scale
        clear controller_lqr
        simulate_building(T0, @controller_lqr, s * Q, R, scen1);
    end
    figure('Position', [0, 0, 1000, 1000])
    for s = scale
        clear controller_lqr
        simulate_building(T0, @controller_lqr, Q, s * R, scen1);
    end
end

% pause;


%%
% Task 8
T0 = T0_2;
figure('Position', [0, 0, 1000, 1000])
clear controller_lqr
simulate_building(T0, @controller_lqr, Q, R, scen1);

% pause;

%% LQR feasible set
disp('LQR feasible set');

% Task 9
[A_x, b_x] = compute_X_LQR(Q, R);
S = Polyhedron('A', A_x, 'b', b_x);
figure('Position', [0, 0, 800, 600])
hold on

x0_1 = T0_1 - T_sp;
x0_2 = T0_2 - T_sp;
scatter3(x0_1(1), x0_1(2), x0_1(3), 'filled');
text(x0_1(1) + 0.1, x0_1(2) + 0.1, x0_1(3) + 0.1, "x_0^{(1)}", 'FontSize', 14);
scatter3(x0_2(1), x0_2(2), x0_2(3), 'filled');
text(x0_2(1) + 0.1, x0_2(2) + 0.1, x0_2(3) + 0.1, "x_0^{(2)}", 'FontSize', 14);
S.plot('color', 'lightgreen', 'Alpha', 0.1);
xlabel('x_{VC}', 'FontSize', 14)
ylabel('x_{F1}', 'FontSize', 14)
zlabel('x_{F2}', 'FontSize', 14)
view(-70, 50)

% pause;


%% LQR optimal cost
disp('LQR optimal cost');

% Task 10
[A_x, b_x] = compute_X_LQR(Q, R);
S = Polyhedron('A', A_x, 'b', b_x);
figure('Position', [0, 0, 800, 600])
S.plot('color', 'lightgreen', 'Alpha', 0.1);
hold on

x_VC = -3:3;
x_F1 = -3:3;
x_F2 = -0.5:0.5:1.5;
[X_VC, X_F1, X_F2] = meshgrid(x_VC, x_F1, x_F2);
X_VC = reshape(X_VC, [numel(X_VC), 1]);
X_F1 = reshape(X_F1, [numel(X_F1), 1]);
X_F2 = reshape(X_F2, [numel(X_F2), 1]);
J = [];
X_VC_selected = [];
X_F1_selected = [];
X_F2_selected = [];
for i = 1 : length(X_VC)
    if S.isInside([X_VC(i); X_F1(i); X_F2(i)])
        clear controller_lqr
        [~, ~, J_opt] = simulate_building([X_VC(i); X_F1(i); X_F2(i)] + T_sp, @controller_lqr, Q, R, scen1, 0);
        J = [J; sum(J_opt)];
        X_VC_selected = [X_VC_selected; X_VC(i)];
        X_F1_selected = [X_F1_selected; X_F1(i)];
        X_F2_selected = [X_F2_selected; X_F2(i)];
    end
end
scatter3(X_VC_selected, X_F1_selected, X_F2_selected, [], J, 'filled');
xlabel('x_{VC}', 'FontSize', 14)
ylabel('x_{F1}', 'FontSize', 14)
zlabel('x_{F2}', 'FontSize', 14)
cb = colorbar;
cb.Label.String = 'Infinite horizon cost under the LQR control law';
cb.FontSize = 14;
view(-20, 60)

% pause;


%% From LQR to MPC
disp('First MPC'); 

% Task 11
T0 = T0_1;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_1
simulate_building(T0, @controller_mpc_1, Q, R, scen1);
T0 = T0_2;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_1
simulate_building(T0, @controller_mpc_1, Q, R, scen1);

% pause;


%% MPC with guarantees
disp('MPC with guarantees');

% Task 13
T0 = T0_1;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_2
simulate_building(T0, @controller_mpc_2, Q, R, scen1);
T0 = T0_2;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_2
simulate_building(T0, @controller_mpc_2, Q, R, scen1);


% Task 14
T0 = T0_1;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_3
simulate_building(T0, @controller_mpc_3, Q, R, scen1);
T0 = T0_2;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_3
simulate_building(T0, @controller_mpc_3, Q, R, scen1);


% Task 15
T0 = T0_1;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_1
[~, ~, J_mpc_1] = simulate_building(T0, @controller_mpc_1, Q, R, scen1);
fprintf('The optimization cost for MPC 1 is %f \n', sum(J_mpc_1));
% figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_2
[~, ~, J_mpc_2] = simulate_building(T0, @controller_mpc_2, Q, R, scen1);
fprintf('The optimization cost for MPC 2 is %f \n', sum(J_mpc_2));
% figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_3
[~, ~, J_mpc_3] = simulate_building(T0, @controller_mpc_3, Q, R, scen1);
fprintf('The optimization cost for MPC 3 is %f \n', sum(J_mpc_3));

T0 = T0_2;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_1
[~, ~, J_mpc_1] = simulate_building(T0, @controller_mpc_1, Q, R, scen1);
fprintf('The optimization cost for MPC 1 is %f \n', sum(J_mpc_1));
% figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_2
[~, ~, J_mpc_2] = simulate_building(T0, @controller_mpc_2, Q, R, scen1);
fprintf('The optimization cost for MPC 2 is %f \n', sum(J_mpc_2));
% figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_3
[~, ~, J_mpc_3] = simulate_building(T0, @controller_mpc_3, Q, R, scen1);
fprintf('The optimization cost for MPC 3 is %f \n', sum(J_mpc_3));

% pause;


%% Soft-constrained MPC
disp('Soft-constrained MPC');

% Task 17
T0 = T0_1;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_3
simulate_building(T0, @controller_mpc_3, Q, R, scen2);


% Task 18
T0 = T0_1;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_4
simulate_building(T0, @controller_mpc_4, Q, R, scen2);


% Task 19
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_3
simulate_building(T0, @controller_mpc_3, Q, R, scen1);
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_4
simulate_building(T0, @controller_mpc_4, Q, R, scen1);


% Task 20
T0 = T0_1;
d = zeros(3, scen2.Nbar + 30);
d(1, 36:50) = -1.2e4;
d(2, 37:43) = 5.5e3;
d(3, 45:49) = 1e3;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_5
simulate_building(T0, @controller_mpc_5, Q, R, scen2, 1, 30, d);
% compare with controller_mpc_4
clear controller_mpc_4
simulate_building(T0, @controller_mpc_4, Q, R, scen2);

% pause;


%% Offset-free MPC
disp('Offset-free MPC');

% % Task 21
% param = compute_controller_base_parameters;
% A_aug = [param.A, param.Bd; zeros(size(param.A)), eye(size(param.Bd))];
% B_aug = [param.B; zeros(size(param.B))];
% Cd = zeros(size(param.C_ref));
% C_aug = [param.C_ref, Cd];
% D_aug = [zeros(size(param.B)); zeros(size(param.B))];


% % Task 22
% P = [0.0, 0.0, 0.0, 0.5, 0.5, 0.5];
% L = place(A_aug', C_aug', P)';


% % Task 23
T0 = T0_1;
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_6
simulate_building(T0, @controller_mpc_6, Q, R, scen3);
figure('Position', [0, 0, 1000, 1000])
clear controller_mpc_3
simulate_building(T0, @controller_mpc_3, Q, R, scen3);

% pause;


%% Comparison using forces
disp('MPC Implementation with FORCES Pro');

% % Task 24
% T0 = T0_2;
% figure('Position', [0, 0, 1000, 1000])
% clear controller_mpc_1_forces
% controller_mpc_1_forces(Q, R, T0, 30); % initialization
% [~, ~, ~, t_sim_forces] = simulate_building(T0, @controller_mpc_1_forces, Q, R, scen1);
% fprintf('The running time for MPC 1 using FORCES Pro is %f s\n', t_sim_forces);
% figure('Position', [0, 0, 1000, 1000])
% clear controller_mpc_1
% [~, ~, ~, t_sim] = simulate_building(T0, @controller_mpc_1, Q, R, scen1);
% fprintf('The running time for MPC 1 is %f s\n', t_sim);

% pause;
