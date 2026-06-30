function sol = solve_alg1_cvx(r0, v0, rf, vf, params)
%SOLVE_ALG1_CVX 用 Algorithm 1 求解无障碍三维最短时间轨迹。
%   每次外层迭代求解一个 SOCP，并更新支撑超平面参数 sigma，
%   直到松弛约束 tf^2<=ts 成为等式（在 eps_t 内）。

check_inputs(r0,v0,rf,vf,params);
N = params.N;
h = 1/(N-1);
sigma = 0;
history = repmat(struct('iteration',0,'sigma',0,'tf',nan,'ts',nan, ...
    'gap',nan,'cvx_status','','cvx_optval',nan,'solve_time',nan), ...
    1, params.max_outer_iter);

for k = 1:params.max_outer_iter
    solve_clock = tic;
    if params.verbose
        fprintf('\nAlgorithm 1 外层迭代 %d，sigma=%.9g\n', k, sigma);
        cvx_begin
    else
        cvx_begin quiet
    end
        cvx_solver(params.cvx_solver_name)
        cvx_precision(params.cvx_precision_name)
        % MATLAB Control System Toolbox 已占用函数名 tf，CVX 不允许同名变量。
        % 因此模型内部使用 tf_cvx/ts_cvx，求解后仍以 tf/ts 对外输出。
        variables r(3,N) vbar(3,N) abar(3,N-1) tf_cvx ts_cvx
        minimize(ts_cvx - sigma*tf_cvx)
        subject to
            tf_cvx >= 0;
            ts_cvx >= 0;
            square_pos(tf_cvx) <= ts_cvx;
            r(:,1) == r0;
            r(:,N) == rf;
            vbar(:,1) == tf_cvx*v0;
            vbar(:,N) == tf_cvx*vf;
            for n = 1:N-1
                r(:,n+1) == r(:,n) + h*vbar(:,n);
                vbar(:,n+1) == vbar(:,n) + h*(abar(:,n) + ts_cvx*params.g_vec);
            end
            for n = 1:N
                norm(vbar(:,n),2) <= params.vmax*tf_cvx;
            end
            for n = 1:N-1
                norm(abar(:,n),2) <= params.amax*ts_cvx;
                norm(abar(1:2,n),2) <= tan(params.phi_max)*abar(3,n);
                abar(3,n) >= 0;
            end
    cvx_end
    solve_time = toc(solve_clock);
    tf = tf_cvx;
    ts = ts_cvx;

    if ~is_cvx_solved(cvx_status)
        error('Algorithm 1 第 %d 次迭代失败，CVX 状态为 "%s"。', k, cvx_status);
    end
    if ~(isfinite(tf) && isfinite(ts) && tf > 0 && ts > 0)
        error('Algorithm 1 得到非正或非有限的 tf/ts。');
    end

    gap = ts - tf^2;
    history(k) = struct('iteration',k,'sigma',sigma,'tf',tf,'ts',ts, ...
        'gap',gap,'cvx_status',cvx_status,'cvx_optval',cvx_optval, ...
        'solve_time',solve_time);
    if params.verbose
        fprintf('tf=%.9g, ts=%.9g, gap=%.3e, status=%s\n', tf, ts, gap, cvx_status);
    end

    % 负 gap 只能来自求解器容差；按论文停止条件将其视为收敛。
    if gap <= params.eps_t
        history = history(1:k);
        sol = pack_solution(r,vbar,abar,tf,ts,sigma,history,params);
        return;
    end

    radicand = (sigma - 2*tf)^2 + 4*gap;
    if radicand < -1e-8
        error('sigma 更新根号内出现负数 %.3e，请检查求解精度。', radicand);
    end
    sigma = sigma + sqrt(max(radicand,0));
end
error('Algorithm 1 在 %d 次外层迭代内未达到 gap<=%.3e。', ...
    params.max_outer_iter, params.eps_t);
end

function sol = pack_solution(r,vbar,abar,tf,ts,sigma,history,params)
sol.r = r; sol.vbar = vbar; sol.abar = abar;
sol.tf = tf; sol.ts = ts; sol.sigma = sigma;
sol.gap = ts-tf^2; sol.history = history;
sol.cvx_status = history(end).cvx_status;
sol.solve_time_total = sum([history.solve_time]);
sol.validation = physical_violations(sol,params);
end

function out = physical_violations(sol,params)
v = sol.vbar/sol.tf; a = sol.abar/sol.ts;
out.max_velocity = max(vecnorm(v,2,1)-params.vmax);
out.max_acceleration = max(vecnorm(a,2,1)-params.amax);
tilt = atan2(vecnorm(a(1:2,:),2,1),a(3,:));
out.max_tilt_rad = max(tilt-params.phi_max);
out.gap = sol.ts-sol.tf^2;
[out.max_r_dynamics_residual, out.max_v_dynamics_residual] = dynamics_residuals(sol,params);
end

function [max_r_residual,max_v_residual] = dynamics_residuals(sol,params)
N = size(sol.r,2);
h = 1/(N-1);
max_r_residual = 0;
max_v_residual = 0;
for n = 1:N-1
    r_residual = sol.r(:,n+1) - sol.r(:,n) - h*sol.vbar(:,n);
    v_residual = sol.vbar(:,n+1) - sol.vbar(:,n) ...
        - h*(sol.abar(:,n) + sol.ts*params.g_vec);
    max_r_residual = max(max_r_residual,norm(r_residual,2));
    max_v_residual = max(max_v_residual,norm(v_residual,2));
end
end

function tf_ok = is_cvx_solved(status)
tf_ok = strcmpi(strtrim(status),'Solved') || strcmpi(strtrim(status),'Inaccurate/Solved');
end

function check_inputs(r0,v0,rf,vf,params)
validateattributes(r0,{'numeric'},{'size',[3,1],'real','finite'});
validateattributes(v0,{'numeric'},{'size',[3,1],'real','finite'});
validateattributes(rf,{'numeric'},{'size',[3,1],'real','finite'});
validateattributes(vf,{'numeric'},{'size',[3,1],'real','finite'});
assert(params.N>=2 && params.N==round(params.N),'params.N 必须是至少为 2 的整数。');
assert(norm(v0)<=params.vmax+1e-12 && norm(vf)<=params.vmax+1e-12, ...
    '端点速度超过 vmax，问题必然不可行。');
end
