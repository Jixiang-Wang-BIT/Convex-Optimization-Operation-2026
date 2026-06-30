function sol = solve_alg1_nlp(r0, v0, rf, vf, params)
%SOLVE_ALG1_NLP  用非线性规划（fmincon）求解无障碍三维最短时间轨迹。
%   直接优化物理变量 tf, r, v, a，不使用凸松弛。输出结构体与
%   solve_alg1_cvx 兼容，可用 recover_physical_trajectory 和现有绘图函数。

N = params.N;
h_scalar = 1 / (N - 1);

% ---------- 构造初始猜测 ----------
tf0 = norm(rf - r0) / (0.55 * params.vmax);
r_init = zeros(3, N);
v_init = zeros(3, N);
for d = 1:3
    r_init(d, :) = linspace(r0(d), rf(d), N);
end
v_const = (rf - r0) / tf0;
a_init = zeros(3, N - 1);
a_init(3, :) = -params.g_vec(3);   % 初始猜测补偿重力
for d = 1:3
    v_init(d, :) = v_const(d);
end

X0 = pack_vars(tf0, r_init, v_init, a_init, N);

% ---------- 边界 ----------
lb = zeros(size(X0));               % tf ≥ 0，其余无下界限制
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
    @(X) nonlcon(X, r0, v0, rf, vf, params, N), ...
    fmincon_opts);
solve_time = toc(solve_clock);

% ---------- 解包 ----------
[tf_opt, r_opt, v_opt, a_opt] = unpack_vars(X_opt, N);

% ---------- 构造与 solve_alg1_cvx 兼容的输出 ----------
sol.tf    = tf_opt;
sol.ts    = tf_opt^2;
sol.r     = r_opt;
sol.vbar  = tf_opt * v_opt;
sol.abar  = (tf_opt^2) * a_opt;
sol.sigma = 0;
sol.gap   = 0;

% fmincon 不产生 outer iteration 历史，构造单条记录以兼容绘图
sol.history = struct('iteration', 1, 'sigma', 0, ...
    'tf', tf_opt, 'ts', tf_opt^2, 'gap', 0, ...
    'cvx_status', fmincon_exit_msg(exitflag), ...
    'cvx_optval', tf_opt, 'solve_time', solve_time);

sol.cvx_status       = fmincon_exit_msg(exitflag);
sol.solve_time_total = solve_time;
sol.fmincon_exitflag = exitflag;
sol.fmincon_output   = fmincon_output;

% 物理约束违反量验证
sol.validation = nlp_validation(sol, params, v_opt, a_opt);
end

% ==================== 局部函数 ====================

function X = pack_vars(tf, r, v, a, N)
% 将物理变量打包为 fmincon 的列向量。
X = [tf; r(:); v(:); a(:)];
end

function [tf, r, v, a] = unpack_vars(X, N)
% 将 fmincon 的列向量解包为物理变量。
tf = X(1);
offset = 1;
r = reshape(X(offset+1 : offset+3*N), [3, N]);
offset = offset + 3*N;
v = reshape(X(offset+1 : offset+3*N), [3, N]);
offset = offset + 3*N;
a = reshape(X(offset+1 : offset+3*(N-1)), [3, N-1]);
end

function f = objective(X, N)
% 目标函数：最小化飞行时间 tf。
[~, ~, ~, ~] = unpack_vars(X, N);  %#ok<ASGLU>
f = X(1);
end

function [c, ceq] = nonlcon(X, r0, v0, rf, vf, params, N)
% 非线性约束：动力学 + 边界条件 + 状态/控制限制。
[tf, r, v, a] = unpack_vars(X, N);
h = tf / (N - 1);

% --- 不等式约束 c ≤ 0 ---
c_speed  = zeros(N, 1);       % 速度限制
c_thrust = zeros(N-1, 1);     % 推力幅值限制
c_tilt   = zeros(N-1, 1);     % 推力倾角限制
c_az     = zeros(N-1, 1);     % 竖直推力非负

for n = 1:N
    c_speed(n) = norm(v(:,n)) - params.vmax;
end
for n = 1:N-1
    c_thrust(n) = norm(a(:,n)) - params.amax;
    c_tilt(n)   = norm(a(1:2, n)) - tan(params.phi_max) * a(3, n);
    c_az(n)     = -a(3, n);
end
c = [c_speed; c_thrust; c_tilt; c_az];

% --- 等式约束 ceq = 0 ---
ceq_dyn_r = zeros(3*(N-1), 1);   % 位置动力学残差
ceq_dyn_v = zeros(3*(N-1), 1);   % 速度动力学残差

idx = 1;
for n = 1:N-1
    ceq_dyn_r(idx:idx+2) = r(:,n+1) - r(:,n) - h * v(:,n);
    ceq_dyn_v(idx:idx+2) = v(:,n+1) - v(:,n) - h * (a(:,n) + params.g_vec);
    idx = idx + 3;
end

% 端点边界条件
ceq_bc = [r(:,1) - r0;  v(:,1) - v0; ...
          r(:,N) - rf;  v(:,N) - vf];

ceq = [ceq_dyn_r; ceq_dyn_v; ceq_bc];
end

function out = nlp_validation(sol, params, v, a)
% 计算物理约束违反量，与 solve_alg1_cvx 的 physical_violations 格式一致。
out.max_velocity     = max(vecnorm(v, 2, 1) - params.vmax);
out.max_acceleration = max(vecnorm(a, 2, 1) - params.amax);
tilt = atan2(vecnorm(a(1:2,:), 2, 1), a(3,:));
out.max_tilt_rad     = max(tilt - params.phi_max);
out.gap              = sol.gap;

% 动力学残差
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
% 将 fmincon 的 exitflag 映射为可读字符串。
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
