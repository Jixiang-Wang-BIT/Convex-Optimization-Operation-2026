function traj = recover_physical_trajectory(sol, params)
%RECOVER_PHYSICAL_TRAJECTORY 从凸化变量恢复真实时间、速度和推力加速度。
%   论文变量满足 vbar=tf*v、abar=ts*a。注意 a 为推力加速度，
%   真实总加速度为 a+g；倾角也由推力加速度而非总加速度计算。

validateattributes(sol.tf, {'numeric'}, {'scalar','real','positive','finite'});
validateattributes(sol.ts, {'numeric'}, {'scalar','real','positive','finite'});

N = size(sol.r, 2);
traj.t = linspace(0, sol.tf, N);
traj.r = sol.r;
traj.v = sol.vbar / sol.tf;
traj.a = sol.abar / sol.ts;
traj.t_a = traj.t(1:size(traj.a, 2));
traj.speed_norm = vecnorm(traj.v, 2, 1);
traj.accel_norm = vecnorm(traj.a, 2, 1);
traj.tilt_deg = atan2(vecnorm(traj.a(1:2,:), 2, 1), traj.a(3,:))*180/pi;
traj.total_accel = traj.a + repmat(params.g_vec, 1, size(traj.a,2));

traj.max_velocity_violation = max(traj.speed_norm - params.vmax);
traj.max_acceleration_violation = max(traj.accel_norm - params.amax);
traj.max_tilt_violation_deg = max(traj.tilt_deg - params.phi_max*180/pi);
traj.gap = sol.ts - sol.tf^2;
[traj.max_r_dynamics_residual, traj.max_v_dynamics_residual] = dynamics_residuals(sol, params);

tol = 5e-5;
if params.verbose
    fprintf('tf=%.6f s, ts=%.6f s^2, gap=%.3e s^2\n', sol.tf, sol.ts, traj.gap);
    fprintf('最大约束违反量: speed %.3e m/s, thrust %.3e m/s^2, tilt %.3e deg\n', ...
        max(0,traj.max_velocity_violation), max(0,traj.max_acceleration_violation), ...
        max(0,traj.max_tilt_violation_deg));
    fprintf('最大动力学残差: r %.3e, vbar %.3e\n', ...
        traj.max_r_dynamics_residual, traj.max_v_dynamics_residual);
end
if traj.max_velocity_violation > tol || traj.max_acceleration_violation > tol || ...
        traj.max_tilt_violation_deg > 1e-3
    warning('恢复轨迹存在小幅约束违反；请检查 CVX 精度或求解器状态。');
end
end

function [max_r_residual, max_v_residual] = dynamics_residuals(sol, params)
N = size(sol.r, 2);
h = 1/(N-1);
max_r_residual = 0;
max_v_residual = 0;
for n = 1:N-1
    r_residual = sol.r(:,n+1) - sol.r(:,n) - h*sol.vbar(:,n);
    v_residual = sol.vbar(:,n+1) - sol.vbar(:,n) ...
        - h*(sol.abar(:,n) + sol.ts*params.g_vec);
    max_r_residual = max(max_r_residual, norm(r_residual,2));
    max_v_residual = max(max_v_residual, norm(v_residual,2));
end
end
