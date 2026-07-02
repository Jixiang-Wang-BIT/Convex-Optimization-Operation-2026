function sol = solve_alg2_cvx(r0, v0, rf, vf, obstacles, r_init, params)
%SOLVE_ALG2_CVX 用双层循环求解带椭圆柱障碍的最短时间轨迹。
%   外层更新 sigma 以收紧 tf^2<=ts；内层在当前轨迹处线性化
%   凹障碍函数。线性化上界小于零可保证原非凸避障约束成立。

validateattributes(r_init,{'numeric'},{'size',[3,params.N],'real','finite'});
N = params.N; h = 1/(N-1); sigma = 0; r_guess = r_init;
outer_history = repmat(struct('iteration',0,'sigma',0,'tf',nan,'ts',nan, ...
    'gap',nan,'inner_iterations',0,'cvx_status','', ...
    'solve_time',nan,'solve_time_source',''), ...
    1, params.max_outer_iter);
inner_history = cell(1,params.max_outer_iter);
successive_ground_tracks = cell(1,params.max_outer_iter);

for k = 1:params.max_outer_iter
    local_hist = repmat(struct('iteration',0,'trajectory_change',nan, ...
        'cvx_status','','cvx_optval',nan, ...
        'solve_time',nan,'solve_time_source','','r',[]), ...
        1, params.max_inner_iter);
    tracks = cell(1,params.max_inner_iter+1);
    tracks{1} = r_guess(1:2,:);
    inner_converged = false;

    for j = 1:params.max_inner_iter
        lin = linearize_obstacles(r_guess,obstacles);
        if params.verbose
            fprintf('\nAlgorithm 2 外层 %d / 内层 %d，sigma=%.9g\n',k,j,sigma);
        end
        cvx_begin
            cvx_solver(params.cvx_solver_name)
            cvx_precision(params.cvx_precision_name)
            % tf 是 MATLAB 工具箱函数名，CVX 不允许同名优化变量。
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
                    vbar(:,n+1) == vbar(:,n) + h*(abar(:,n)+ts_cvx*params.g_vec);
                end
                for n = 1:N
                    norm(vbar(:,n),2) <= params.vmax*tf_cvx;
                    for i = 1:numel(obstacles)
                        lin.g0(i,n) + lin.grad(:,n,i)'*(r(:,n)-r_guess(:,n)) <= 0;
                    end
                end
                for n = 1:N-1
                    norm(abar(:,n),2) <= params.amax*ts_cvx;
                    norm(abar(1:2,n),2) <= tan(params.phi_max)*abar(3,n);
                    abar(3,n) >= 0;
                end
        cvx_log = evalc('cvx_end');
        if params.verbose
            fprintf('%s',cvx_log);
        end
        [solve_time,solve_time_source] = parse_solver_time(cvx_log);
        if params.verbose && ~isfinite(solve_time)
            fprintf('Warning: solver time was not found in CVX log; no wall-clock time is counted.\n');
        end
        tf = tf_cvx;
        ts = ts_cvx;

        if ~is_cvx_solved(cvx_status)
            error('Algorithm 2 外层 %d、内层 %d 失败，CVX 状态为 "%s"。',k,j,cvx_status);
        end
        if ~(isfinite(tf) && isfinite(ts) && tf>0 && ts>0)
            error('Algorithm 2 得到非正或非有限的 tf/ts。');
        end
        trajectory_change = max(abs(r(:)-r_guess(:)));
        local_hist(j) = struct('iteration',j,'trajectory_change',trajectory_change, ...
            'cvx_status',cvx_status,'cvx_optval',cvx_optval, ...
            'solve_time',solve_time,'solve_time_source',solve_time_source,'r',r);
        tracks{j+1} = r(1:2,:);
        if params.verbose
            fprintf('轨迹变化量 %.3e m，tf %.6f s，gap %.3e s^2\n', ...
                trajectory_change,tf,ts-tf^2);
        end
        r_guess = r;
        if trajectory_change <= params.eps_r
            inner_converged = true;
            local_hist = local_hist(1:j);
            tracks = tracks(1:j+1);
            break;
        end
    end
    if ~inner_converged
        error('Algorithm 2 外层 %d 的内层循环在 %d 次内未收敛。',k,params.max_inner_iter);
    end

    gap = ts-tf^2;
    outer_history(k) = struct('iteration',k,'sigma',sigma,'tf',tf,'ts',ts, ...
        'gap',gap,'inner_iterations',j,'cvx_status',cvx_status, ...
        'solve_time',sum_finite([local_hist.solve_time]), ...
        'solve_time_source','sum_inner_solver_logs');
    inner_history{k} = local_hist;
    successive_ground_tracks{k} = tracks;

    if gap <= params.eps_t
        outer_history = outer_history(1:k);
        inner_history = inner_history(1:k);
        successive_ground_tracks = successive_ground_tracks(1:k);
        sol.r=r; sol.vbar=vbar; sol.abar=abar; sol.tf=tf; sol.ts=ts;
        sol.sigma=sigma; sol.gap=gap; sol.cvx_status=cvx_status;
        sol.outer_history=outer_history; sol.history=outer_history;
        sol.inner_history=inner_history;
        sol.successive_ground_tracks=successive_ground_tracks;
        sol.obstacle_margin = obstacle_margins(r,obstacles);
        sol.min_obstacle_margin = min(sol.obstacle_margin(:));
        sol.min_margin_each_obstacle = min(sol.obstacle_margin,[],2);
        sol.solve_time_total = sum_finite([outer_history.solve_time]);
        sol.validation = physical_violations(sol,params);
        if sol.min_obstacle_margin < -1e-6
            error('最终轨迹障碍最小 margin=%.3e<0，未满足安全约束。',sol.min_obstacle_margin);
        end
        return;
    end
    radicand = (sigma-2*tf)^2 + 4*gap;
    if radicand < -1e-8
        error('sigma 更新根号内出现负数 %.3e，请检查求解精度。',radicand);
    end
    sigma = sigma + sqrt(max(radicand,0));
end
error('Algorithm 2 在 %d 次外层迭代内未达到 gap<=%.3e。', ...
    params.max_outer_iter,params.eps_t);
end

function margins = obstacle_margins(r,obstacles)
margins = zeros(numel(obstacles),size(r,2));
for i=1:numel(obstacles)
    for n=1:size(r,2)
        d=r(:,n)-obstacles(i).center;
        margins(i,n)=d'*obstacles(i).P*d-1;
    end
end
end

function out = physical_violations(sol,params)
v=sol.vbar/sol.tf; a=sol.abar/sol.ts;
out.max_velocity=max(vecnorm(v,2,1)-params.vmax);
out.max_acceleration=max(vecnorm(a,2,1)-params.amax);
out.max_tilt_rad=max(atan2(vecnorm(a(1:2,:),2,1),a(3,:))-params.phi_max);
out.gap=sol.ts-sol.tf^2;
out.min_obstacle_margin=sol.min_obstacle_margin;
out.min_margin_each_obstacle=sol.min_margin_each_obstacle;
[out.max_r_dynamics_residual,out.max_v_dynamics_residual]=dynamics_residuals(sol,params);
end

function [max_r_residual,max_v_residual]=dynamics_residuals(sol,params)
N=size(sol.r,2);
h=1/(N-1);
max_r_residual=0;
max_v_residual=0;
for n=1:N-1
    r_residual=sol.r(:,n+1)-sol.r(:,n)-h*sol.vbar(:,n);
    v_residual=sol.vbar(:,n+1)-sol.vbar(:,n) ...
        - h*(sol.abar(:,n)+sol.ts*params.g_vec);
    max_r_residual=max(max_r_residual,norm(r_residual,2));
    max_v_residual=max(max_v_residual,norm(v_residual,2));
end
end

function tf_ok=is_cvx_solved(status)
tf_ok=strcmpi(strtrim(status),'Solved') || strcmpi(strtrim(status),'Inaccurate/Solved');
end

function [solver_time,source]=parse_solver_time(cvx_log)
solver_time=nan;
source='unavailable';
tokens=regexp(cvx_log,'Optimizer terminated\.\s*Time:\s*([0-9.eE+-]+)','tokens');
if ~isempty(tokens)
    solver_time=str2double(tokens{end}{1});
    source='mosek_log';
    return;
end
tokens=regexp(cvx_log,'Total CPU time \(secs\)\s*=\s*([0-9.eE+-]+)','tokens');
if ~isempty(tokens)
    solver_time=str2double(tokens{end}{1});
    source='sdpt3_log';
end
end

function total=sum_finite(values)
values=values(isfinite(values));
if isempty(values)
    total=nan;
else
    total=sum(values);
end
end
