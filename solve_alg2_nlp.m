function sol = solve_alg2_nlp(r0, v0, rf, vf, obstacles, r_init, params)
%SOLVE_ALG2_NLP  用非线性规划（fmincon）求解带椭圆柱障碍的最短时间轨迹。
%   直接优化物理变量 tf, r, v, a，障碍约束采用原始非凸形式
%   g(r) = 1 - (r-c)'P(r-c) ≤ 0。输出结构体与 solve_alg2_cvx 兼容。

N = params.N;

% ---------- 构造初始猜测 ----------
tf0 = norm(rf - r0) / (0.55 * params.vmax);
v_init = zeros(3, N);
v_const = (rf - r0) / tf0;
for d = 1:3
    v_init(d, :) = v_const(d);
end
a_init = zeros(3, N - 1);
a_init(3, :) = -params.g_vec(3);   % 初始猜测补偿重力

X0 = pack_vars(tf0, r_init, v_init, a_init, N);

% ---------- 边界 ----------
lb = zeros(size(X0));
lb(2:end) = -inf;
ub = inf(size(X0));

% ---------- 求解 ----------
fmincon_opts = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'MaxIterations', params.max_outer_iter * 100, ...
    'MaxFunctionEvaluations', 1e5, ...
    'OptimalityTolerance', 1e-6, ...
    'ConstraintTolerance', 1e-4, ...
    'StepTolerance', 1e-8, ...
    'Display', 'iter-detailed');

solve_clock = tic;
[X_opt, ~, exitflag, fmincon_output] = fmincon( ...
    @(X) objective(X, N), X0, ...
    [], [], [], [], lb, ub, ...
    @(X) nonlcon(X, r0, v0, rf, vf, obstacles, params, N), ...
    fmincon_opts);
solve_time = toc(solve_clock);

% ---------- 解包 ----------
[tf_opt, r_opt, v_opt, a_opt] = unpack_vars(X_opt, N);

% ---------- 计算障碍安全裕度 ----------
[obstacle_margin, min_margin_each] = compute_obstacle_margins(r_opt, obstacles);
min_margin = min(obstacle_margin(:));

% ---------- 构造与 solve_alg2_cvx 兼容的输出 ----------
sol.tf    = tf_opt;
sol.ts    = tf_opt^2;
sol.r     = r_opt;
sol.vbar  = tf_opt * v_opt;
sol.abar  = (tf_opt^2) * a_opt;
sol.sigma = 0;
sol.gap   = 0;

% 合成外层/内层历史以兼容 plot_alg2_results
sol.outer_history = struct('iteration', 1, 'sigma', 0, ...
    'tf', tf_opt, 'ts', tf_opt^2, 'gap', 0, ...
    'inner_iterations', 1, ...
    'cvx_status', fmincon_exit_msg(exitflag), ...
    'solve_time', solve_time);

inner_entry = struct('iteration', 1, 'trajectory_change', 0, ...
    'cvx_status', fmincon_exit_msg(exitflag), ...
    'cvx_optval', tf_opt, 'solve_time', solve_time, 'r', r_opt);
sol.inner_history = {inner_entry};

sol.successive_ground_tracks = {{r_init(1:2, :), r_opt(1:2, :)}};

sol.history = sol.outer_history;

sol.obstacle_margin        = obstacle_margin;
sol.min_obstacle_margin    = min_margin;
sol.min_margin_each_obstacle = min_margin_each;

sol.cvx_status       = fmincon_exit_msg(exitflag);
sol.solve_time_total = solve_time;
sol.fmincon_exitflag = exitflag;
sol.fmincon_output   = fmincon_output;

% 物理约束违反量验证
sol.validation = nlp_validation(sol, params, v_opt, a_opt, obstacles);
end

% ==================== 局部函数 ====================

function X = pack_vars(tf, r, v, a, N)
X = [tf; r(:); v(:); a(:)];
end

function [tf, r, v, a] = unpack_vars(X, N)
tf = X(1);
offset = 1;
r = reshape(X(offset+1 : offset+3*N), [3, N]);
offset = offset + 3*N;
v = reshape(X(offset+1 : offset+3*N), [3, N]);
offset = offset + 3*N;
a = reshape(X(offset+1 : offset+3*(N-1)), [3, N-1]);
end

function f = objective(X, N)
[~, ~, ~, ~] = unpack_vars(X, N);  %#ok<ASGLU>
f = X(1);
end

function [c, ceq] = nonlcon(X, r0, v0, rf, vf, obstacles, params, N)
[tf, r, v, a] = unpack_vars(X, N);
h = tf / (N - 1);

% --- 不等式约束 c ≤ 0 ---
n_obs = numel(obstacles);

c_speed   = zeros(N, 1);
c_thrust  = zeros(N - 1, 1);
c_tilt    = zeros(N - 1, 1);
c_az      = zeros(N - 1, 1);
c_obstacle = zeros(N * n_obs, 1);   % 障碍约束

for n = 1:N
    c_speed(n) = norm(v(:, n)) - params.vmax;
    % 障碍约束：g(r) = 1 - (r-c)'P(r-c) ≤ 0
    for i = 1:n_obs
        d = r(:, n) - obstacles(i).center;
        c_obstacle((n-1)*n_obs + i) = 1 - d' * obstacles(i).P * d;
    end
end
for n = 1:N-1
    c_thrust(n) = norm(a(:, n)) - params.amax;
    c_tilt(n)   = norm(a(1:2, n)) - tan(params.phi_max) * a(3, n);
    c_az(n)     = -a(3, n);
end
c = [c_speed; c_thrust; c_tilt; c_az; c_obstacle];

% --- 等式约束 ceq = 0 ---
ceq_dyn_r = zeros(3*(N-1), 1);
ceq_dyn_v = zeros(3*(N-1), 1);

idx = 1;
for n = 1:N-1
    ceq_dyn_r(idx:idx+2) = r(:, n+1) - r(:, n) - h * v(:, n);
    ceq_dyn_v(idx:idx+2) = v(:, n+1) - v(:, n) - h * (a(:, n) + params.g_vec);
    idx = idx + 3;
end

ceq_bc = [r(:,1) - r0;  v(:,1) - v0; ...
          r(:,N) - rf;  v(:,N) - vf];

ceq = [ceq_dyn_r; ceq_dyn_v; ceq_bc];
end

function [margins, min_each] = compute_obstacle_margins(r, obstacles)
n_obs = numel(obstacles);
margins = zeros(n_obs, size(r, 2));
for i = 1:n_obs
    for n = 1:size(r, 2)
        d = r(:, n) - obstacles(i).center;
        margins(i, n) = d' * obstacles(i).P * d - 1;
    end
end
min_each = min(margins, [], 2);
end

function out = nlp_validation(sol, params, v, a, obstacles)
out.max_velocity     = max(vecnorm(v, 2, 1) - params.vmax);
out.max_acceleration = max(vecnorm(a, 2, 1) - params.amax);
tilt = atan2(vecnorm(a(1:2,:), 2, 1), a(3,:));
out.max_tilt_rad     = max(tilt - params.phi_max);
out.gap              = sol.gap;
out.min_obstacle_margin = sol.min_obstacle_margin;
out.min_margin_each_obstacle = sol.min_margin_each_obstacle;

N = size(sol.r, 2);
h = sol.tf / (N - 1);
max_r_residual = 0;
max_v_residual = 0;
for n = 1:N-1
    r_res = sol.r(:,n+1) - sol.r(:,n) - h * sol.vbar(:,n) / sol.tf;
    v_res = sol.vbar(:,n+1)/sol.tf - sol.vbar(:,n)/sol.tf ...
            - h * (sol.abar(:,n)/sol.ts + params.g_vec);
    max_r_residual = max(max_r_residual, norm(r_res, 2));
    max_v_residual = max(max_v_residual, norm(v_res, 2));
end
out.max_r_dynamics_residual  = max_r_residual;
out.max_v_dynamics_residual  = max_v_residual;
end

function msg = fmincon_exit_msg(exitflag)
switch exitflag
    case 1
        msg = 'fmincon: 一阶最优性满足';
    case 2
        msg = 'fmincon: 步长/约束满足';
    case 3
        msg = 'fmincon: 目标函数变化量小于容差';
    case 0
        msg = 'fmincon: 达到最大迭代/函数计算次数';
    case -1
        msg = 'fmincon: 被输出函数终止';
    case -2
        msg = 'fmincon: 未找到可行点';
    otherwise
        msg = sprintf('fmincon exitflag=%d', exitflag);
end
end
