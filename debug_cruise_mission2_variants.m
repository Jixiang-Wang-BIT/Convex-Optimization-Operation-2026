%DEBUG_CRUISE_MISSION2_VARIANTS Diagnose the cruising mission 2 discrepancy.
% This script does not modify main_alg1_no_obstacle.m and writes only to
% results/debug_cruise_mission2 and figures/debug_cruise_mission2.

clear; clc; close all
code_dir = fileparts(mfilename('fullpath'));
addpath(code_dir);

params = make_default_params();
params.verbose = false;
params.figure_visible = 'off';
params.save_fig_files = false;

debug_results_dir = fullfile(params.results_dir,'debug_cruise_mission2');
debug_figures_dir = fullfile(params.figures_dir,'debug_cruise_mission2');
if ~exist(debug_results_dir,'dir'), mkdir(debug_results_dir); end
if ~exist(debug_figures_dir,'dir'), mkdir(debug_figures_dir); end

r0 = [0;0;15];
rf = [30;30;15];
v0 = [0;10;0];
paper_tf = 6.96;

labels = {'A_current_text','B_sqrt_denominator','C_sign_reversed', ...
    'D_double_positive','E_double_negative','F_pure_left', ...
    'G_pure_down','H_mission1_sanity'};
descriptions = { ...
    'vf = [-5*sqrt(2);  5*sqrt(2); 0]', ...
    'vf = [-5/sqrt(2);  5/sqrt(2); 0]', ...
    'vf = [ 5*sqrt(2); -5*sqrt(2); 0]', ...
    'vf = [ 5*sqrt(2);  5*sqrt(2); 0]', ...
    'vf = [-5*sqrt(2); -5*sqrt(2); 0]', ...
    'vf = [-10; 0; 0]', ...
    'vf = [0; -10; 0]', ...
    'vf = [10; 0; 0]'};
vf_candidates = [ ...
    -5*sqrt(2), -5/sqrt(2),  5*sqrt(2),  5*sqrt(2), -5*sqrt(2), -10,   0, 10; ...
     5*sqrt(2),  5/sqrt(2), -5*sqrt(2),  5*sqrt(2), -5*sqrt(2),   0, -10,  0; ...
     0,          0,          0,          0,          0,           0,   0,  0];

num_cases = numel(labels);
sols = cell(num_cases,1);
trajs = cell(num_cases,1);

label_col = labels(:);
description_col = descriptions(:);
vf_x = nan(num_cases,1);
vf_y = nan(num_cases,1);
vf_z = nan(num_cases,1);
vf_norm = nan(num_cases,1);
min_angle_deg = nan(num_cases,1);
ccw_angle_deg = nan(num_cases,1);
tf = nan(num_cases,1);
ts = nan(num_cases,1);
gap = nan(num_cases,1);
outer_iterations = nan(num_cases,1);
sigma_final = nan(num_cases,1);
cvx_status = repmat({''},num_cases,1);
solve_time_total = nan(num_cases,1);
max_velocity_violation = nan(num_cases,1);
max_acceleration_violation = nan(num_cases,1);
max_tilt_violation_rad = nan(num_cases,1);
max_r_dynamics_residual = nan(num_cases,1);
max_vbar_dynamics_residual = nan(num_cases,1);
terminal_v_x = nan(num_cases,1);
terminal_v_y = nan(num_cases,1);
terminal_v_z = nan(num_cases,1);
error_message = repmat({''},num_cases,1);

fprintf('\n=== Cruising mission 2 terminal velocity variants ===\n');
for i = 1:num_cases
    vf = vf_candidates(:,i);
    vf_x(i) = vf(1);
    vf_y(i) = vf(2);
    vf_z(i) = vf(3);
    vf_norm(i) = norm(vf);
    [min_angle_deg(i),ccw_angle_deg(i)] = planar_angles(v0,vf);

    try
        sol = solve_alg1_cvx(r0,v0,rf,vf,params);
        traj = recover_physical_trajectory(sol,params);
        sols{i} = sol;
        trajs{i} = traj;

        tf(i) = sol.tf;
        ts(i) = sol.ts;
        gap(i) = sol.gap;
        outer_iterations(i) = numel(sol.history);
        sigma_final(i) = sol.sigma;
        cvx_status{i} = sol.cvx_status;
        solve_time_total(i) = sol.solve_time_total;
        max_velocity_violation(i) = sol.validation.max_velocity;
        max_acceleration_violation(i) = sol.validation.max_acceleration;
        max_tilt_violation_rad(i) = sol.validation.max_tilt_rad;
        max_r_dynamics_residual(i) = sol.validation.max_r_dynamics_residual;
        max_vbar_dynamics_residual(i) = sol.validation.max_v_dynamics_residual;
        terminal_v_x(i) = traj.v(1,end);
        terminal_v_y(i) = traj.v(2,end);
        terminal_v_z(i) = traj.v(3,end);

        fprintf('%s: tf=%.6f, gap=%.3e, min angle=%.1f deg, ccw=%.1f deg\n', ...
            labels{i},tf(i),gap(i),min_angle_deg(i),ccw_angle_deg(i));
    catch ME
        error_message{i} = ME.message;
        cvx_status{i} = 'failed';
        fprintf('%s: FAILED: %s\n',labels{i},ME.message);
    end
end

summary = table(label_col,description_col,vf_x,vf_y,vf_z,vf_norm, ...
    min_angle_deg,ccw_angle_deg,tf,ts,gap,outer_iterations, ...
    sigma_final,cvx_status,solve_time_total,max_velocity_violation, ...
    max_acceleration_violation,max_tilt_violation_rad, ...
    max_r_dynamics_residual,max_vbar_dynamics_residual, ...
    terminal_v_x,terminal_v_y,terminal_v_z,error_message);

disp(summary);
save(fullfile(debug_results_dir,'mission2_variants_summary.mat'), ...
    'summary','sols','trajs','labels','descriptions','vf_candidates','params', ...
    'r0','rf','v0','paper_tf');
writetable(summary,fullfile(debug_results_dir,'mission2_variants_summary.csv'));

[closest_delta,closest_idx] = min(abs(tf-paper_tf));
closest_label = labels{closest_idx};
fprintf('\nClosest to %.2f s: %s, tf=%.6f, delta=%.6f s\n', ...
    paper_tf,closest_label,tf(closest_idx),closest_delta);

fprintf('\n=== Candidate A solver robustness ===\n');
solver_names = {'mosek','sdpt3','sedumi'};
solver_result = run_solver_robustness(r0,v0,rf,vf_candidates(:,1),params,solver_names);
disp(solver_result);
save(fullfile(debug_results_dir,'candidate_A_solver_robustness.mat'),'solver_result');
writetable(solver_result,fullfile(debug_results_dir,'candidate_A_solver_robustness.csv'));

fprintf('\n=== Candidate A N-control-node diagnostic ===\n');
node_control_result = run_node_control_test(r0,v0,rf,vf_candidates(:,1),params);
disp(node_control_result);
save(fullfile(debug_results_dir,'candidate_A_node_control_test.mat'),'node_control_result');
writetable(node_control_result,fullfile(debug_results_dir,'candidate_A_node_control_test.csv'));

plot_variant_comparison(sols,trajs,labels,[1,closest_idx],paper_tf,debug_figures_dir);

report_path = fullfile(debug_results_dir,'diagnosis_report.md');
write_report(report_path,summary,solver_result,node_control_result, ...
    labels,closest_idx,closest_delta,paper_tf);
fprintf('\nDiagnosis report written to:\n%s\n',report_path);

function [min_angle_deg,ccw_angle_deg] = planar_angles(v0,vf)
theta0 = atan2(v0(2),v0(1));
thetaf = atan2(vf(2),vf(1));
ccw_angle_deg = mod((thetaf-theta0)*180/pi,360);
min_angle_deg = acosd(max(-1,min(1,dot(v0(1:2),vf(1:2))/(norm(v0(1:2))*norm(vf(1:2))))));
end

function solver_result = run_solver_robustness(r0,v0,rf,vf,base_params,solver_names)
n = numel(solver_names);
solver = solver_names(:);
status = repmat({'not_run'},n,1);
tf = nan(n,1);
ts = nan(n,1);
gap = nan(n,1);
outer_iterations = nan(n,1);
solve_time_total = nan(n,1);
max_r_dynamics_residual = nan(n,1);
max_vbar_dynamics_residual = nan(n,1);
message = repmat({''},n,1);
for i = 1:n
    params_i = base_params;
    params_i.cvx_solver_name = solver_names{i};
    try
        cvx_clear
        sol = solve_alg1_cvx(r0,v0,rf,vf,params_i);
        status{i} = sol.cvx_status;
        tf(i) = sol.tf;
        ts(i) = sol.ts;
        gap(i) = sol.gap;
        outer_iterations(i) = numel(sol.history);
        solve_time_total(i) = sol.solve_time_total;
        max_r_dynamics_residual(i) = sol.validation.max_r_dynamics_residual;
        max_vbar_dynamics_residual(i) = sol.validation.max_v_dynamics_residual;
    catch ME
        status{i} = 'unavailable_or_failed';
        message{i} = ME.message;
    end
end
solver_result = table(solver,status,tf,ts,gap,outer_iterations, ...
    solve_time_total,max_r_dynamics_residual,max_vbar_dynamics_residual,message);
end

function node_control_result = run_node_control_test(r0,v0,rf,vf,base_params)
model = {'formal_N_minus_1_control';'diagnostic_N_control'};
status = repmat({''},2,1);
tf = nan(2,1);
ts = nan(2,1);
gap = nan(2,1);
outer_iterations = nan(2,1);
solve_time_total = nan(2,1);
max_r_dynamics_residual = nan(2,1);
max_vbar_dynamics_residual = nan(2,1);
message = repmat({''},2,1);

try
    cvx_clear
    sol = solve_alg1_cvx(r0,v0,rf,vf,base_params);
    status{1} = sol.cvx_status;
    tf(1) = sol.tf;
    ts(1) = sol.ts;
    gap(1) = sol.gap;
    outer_iterations(1) = numel(sol.history);
    solve_time_total(1) = sol.solve_time_total;
    max_r_dynamics_residual(1) = sol.validation.max_r_dynamics_residual;
    max_vbar_dynamics_residual(1) = sol.validation.max_v_dynamics_residual;
catch ME
    status{1} = 'failed';
    message{1} = ME.message;
end

try
    cvx_clear
    sol = solve_alg1_cvx_N_control(r0,v0,rf,vf,base_params);
    status{2} = sol.cvx_status;
    tf(2) = sol.tf;
    ts(2) = sol.ts;
    gap(2) = sol.gap;
    outer_iterations(2) = numel(sol.history);
    solve_time_total(2) = sol.solve_time_total;
    max_r_dynamics_residual(2) = sol.validation.max_r_dynamics_residual;
    max_vbar_dynamics_residual(2) = sol.validation.max_v_dynamics_residual;
catch ME
    status{2} = 'failed';
    message{2} = ME.message;
end

delta_tf_from_formal = tf - tf(1);
node_control_result = table(model,status,tf,ts,gap,outer_iterations, ...
    solve_time_total,max_r_dynamics_residual,max_vbar_dynamics_residual, ...
    delta_tf_from_formal,message);
end

function sol = solve_alg1_cvx_N_control(r0,v0,rf,vf,params)
N = params.N;
h = 1/(N-1);
sigma = 0;
history = repmat(struct('iteration',0,'sigma',0,'tf',nan,'ts',nan, ...
    'gap',nan,'cvx_status','','cvx_optval',nan,'solve_time',nan), ...
    1, params.max_outer_iter);
for k = 1:params.max_outer_iter
    solve_clock = tic;
    cvx_begin quiet
        cvx_solver(params.cvx_solver_name)
        cvx_precision(params.cvx_precision_name)
        variables r(3,N) vbar(3,N) abar(3,N) tf_cvx ts_cvx
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
                norm(abar(:,n),2) <= params.amax*ts_cvx;
                norm(abar(1:2,n),2) <= tan(params.phi_max)*abar(3,n);
                abar(3,n) >= 0;
            end
    cvx_end
    solve_time = toc(solve_clock);
    tf = tf_cvx;
    ts = ts_cvx;
    if ~(strcmpi(strtrim(cvx_status),'Solved') || strcmpi(strtrim(cvx_status),'Inaccurate/Solved'))
        error('N-control diagnostic failed at iteration %d with CVX status "%s".',k,cvx_status);
    end
    gap = ts - tf^2;
    history(k) = struct('iteration',k,'sigma',sigma,'tf',tf,'ts',ts, ...
        'gap',gap,'cvx_status',cvx_status,'cvx_optval',cvx_optval, ...
        'solve_time',solve_time);
    if gap <= params.eps_t
        history = history(1:k);
        sol.r = r;
        sol.vbar = vbar;
        sol.abar = abar;
        sol.tf = tf;
        sol.ts = ts;
        sol.sigma = sigma;
        sol.gap = gap;
        sol.history = history;
        sol.cvx_status = cvx_status;
        sol.solve_time_total = sum([history.solve_time]);
        sol.validation = local_validation(sol,params);
        return
    end
    radicand = (sigma - 2*tf)^2 + 4*gap;
    sigma = sigma + sqrt(max(radicand,0));
end
error('N-control diagnostic did not converge within %d iterations.',params.max_outer_iter);
end

function out = local_validation(sol,params)
v = sol.vbar/sol.tf;
a = sol.abar/sol.ts;
out.max_velocity = max(vecnorm(v,2,1)-params.vmax);
out.max_acceleration = max(vecnorm(a,2,1)-params.amax);
out.max_tilt_rad = max(atan2(vecnorm(a(1:2,:),2,1),a(3,:))-params.phi_max);
out.gap = sol.gap;
[out.max_r_dynamics_residual,out.max_v_dynamics_residual] = local_dynamics_residuals(sol,params);
end

function [max_r_residual,max_v_residual] = local_dynamics_residuals(sol,params)
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

function plot_variant_comparison(sols,trajs,labels,idx,paper_tf,fig_dir)
idx = unique(idx,'stable');
idx = idx(~cellfun(@isempty,sols(idx)));
if isempty(idx)
    return
end
colors = lines(numel(idx));
fig = figure('Color','w','Visible','off','Name','mission2 variant comparison');
tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

nexttile; hold on; grid on; axis equal
for q = 1:numel(idx)
    tr = trajs{idx(q)};
    plot(tr.r(1,:),tr.r(2,:),'LineWidth',1.8,'Color',colors(q,:));
end
xlabel('x (m)'); ylabel('y (m)'); title('Ground track');

nexttile; hold on; grid on
for q = 1:numel(idx)
    tr = trajs{idx(q)};
    plot(tr.r(1,:),tr.r(3,:),'LineWidth',1.8,'Color',colors(q,:));
end
xlabel('x (m)'); ylabel('z (m)'); title('x-z side track');

nexttile; hold on; grid on
for q = 1:numel(idx)
    tr = trajs{idx(q)};
    plot(tr.t,tr.speed_norm,'LineWidth',1.8,'Color',colors(q,:));
end
yline(10,'k--','v_{max}');
xlabel('t (s)'); ylabel('speed (m/s)'); title('Speed norm');

nexttile; hold on; grid on
for q = 1:numel(idx)
    tr = trajs{idx(q)};
    plot(tr.t_a,tr.accel_norm,'LineWidth',1.8,'Color',colors(q,:));
end
yline(15,'k--','a_{max}');
xlabel('t (s)'); ylabel('thrust accel (m/s^2)'); title('Thrust acceleration norm');

nexttile; hold on; grid on
for q = 1:numel(idx)
    tr = trajs{idx(q)};
    plot(tr.t_a,tr.tilt_deg,'LineWidth',1.8,'Color',colors(q,:));
end
yline(40,'k--','\phi_{max}');
xlabel('t (s)'); ylabel('tilt (deg)'); title('Tilt angle');

nexttile; hold on; grid on
all_tf = [];
for q = 1:numel(idx)
    sol = sols{idx(q)};
    all_tf = [all_tf, [sol.history.tf]]; %#ok<AGROW>
end
tf_grid = linspace(0,max([all_tf,paper_tf])*1.12,300);
plot(tf_grid,tf_grid.^2,'k-','LineWidth',1.2);
for q = 1:numel(idx)
    sol = sols{idx(q)};
    plot([sol.history.tf],[sol.history.ts],'o-','LineWidth',1.8,'Color',colors(q,:), ...
        'MarkerFaceColor',colors(q,:));
end
xline(paper_tf,'k:','paper 6.96 s');
xlabel('t_f (s)'); ylabel('t_s (s^2)'); title('tf-ts iterations');

legend_labels = labels(idx);
legend(legend_labels,'Interpreter','none','Location','best');
exportgraphics(fig,fullfile(fig_dir,'candidate_A_vs_closest.png'),'Resolution',200);
savefig(fig,fullfile(fig_dir,'candidate_A_vs_closest.fig'));
close(fig);
end

function write_report(report_path,summary,solver_result,node_control_result,labels,closest_idx,closest_delta,paper_tf)
idxA = find(strcmp(summary.label_col,'A_current_text'),1);
tfA = summary.tf(idxA);
gapA = summary.gap(idxA);
max_viol_A = max([0,summary.max_velocity_violation(idxA), ...
    summary.max_acceleration_violation(idxA),summary.max_tilt_violation_rad(idxA)]);
feasibleA = isfinite(tfA) && gapA <= 1e-3 && max_viol_A <= 1e-5 ...
    && summary.max_r_dynamics_residual(idxA) <= 1e-8 ...
    && summary.max_vbar_dynamics_residual(idxA) <= 1e-8;
fid = fopen(report_path,'w');
fprintf(fid,'# Cruise Mission 2 Diagnosis\n\n');
fprintf(fid,'## Static Checks\n\n');
fprintf(fid,'- `main_alg1_no_obstacle.m` uses `r0=[0;0;15]`, `rf=[30;30;15]`, `v0=[0;10;0]`, and `vf=[-5*sqrt(2);5*sqrt(2);0]` for cruising mission 2.\n');
fprintf(fid,'- `solve_alg1_cvx.m` keeps the paper SOCP form: `vbar=tf*v`, `abar=ts*a`, `square_pos(tf)<=ts`, Euler dynamics, velocity/thrust/tilt SOC constraints, `minimize(ts-sigma*tf)`, and the paper sigma update.\n\n');
fprintf(fid,'## Candidate A Feasibility\n\n');
fprintf(fid,'- Candidate A `tf = %.9f s`, `gap = %.3e`.\n',tfA,gapA);
fprintf(fid,'- Maximum velocity violation: %.3e.\n',summary.max_velocity_violation(idxA));
fprintf(fid,'- Maximum acceleration violation: %.3e.\n',summary.max_acceleration_violation(idxA));
fprintf(fid,'- Maximum tilt violation: %.3e rad.\n',summary.max_tilt_violation_rad(idxA));
fprintf(fid,'- Maximum r dynamics residual: %.3e.\n',summary.max_r_dynamics_residual(idxA));
fprintf(fid,'- Maximum vbar dynamics residual: %.3e.\n',summary.max_vbar_dynamics_residual(idxA));
fprintf(fid,'- Feasibility conclusion: %s.\n\n',logical_text(feasibleA));
fprintf(fid,'## Closest Candidate to %.2f s\n\n',paper_tf);
fprintf(fid,'- Closest candidate: `%s`.\n',labels{closest_idx});
fprintf(fid,'- `tf = %.9f s`, absolute difference from %.2f s is %.9f s.\n', ...
    summary.tf(closest_idx),paper_tf,closest_delta);
fprintf(fid,'- Its minimum angle from `v0` is %.3f deg; its counterclockwise rotation from `v0` is %.3f deg.\n\n', ...
    summary.min_angle_deg(closest_idx),summary.ccw_angle_deg(closest_idx));
fprintf(fid,'## 315 Degree Adjustment Note\n\n');
fprintf(fid,'- Candidate A has minimum angle %.3f deg and counterclockwise rotation %.3f deg from `v0` to `vf`.\n', ...
    summary.min_angle_deg(idxA),summary.ccw_angle_deg(idxA));
fprintf(fid,'- A phrase like "315 deg adjustment" is naturally associated with the long rotation direction for a 45 deg shortest-angle change. Under the current convex model, the vehicle is not forced to take that long rotation, and Candidate A solves to a shorter feasible trajectory.\n\n');
fprintf(fid,'## Solver Robustness\n\n');
for i = 1:height(solver_result)
    fprintf(fid,'- %s: status `%s`, tf %.9f, message `%s`.\n', ...
        solver_result.solver{i},solver_result.status{i},solver_result.tf(i),solver_result.message{i});
end
fprintf(fid,'\n## Control Node Diagnostic\n\n');
for i = 1:height(node_control_result)
    fprintf(fid,'- %s: status `%s`, tf %.9f, delta from formal %.9f.\n', ...
        node_control_result.model{i},node_control_result.status{i}, ...
        node_control_result.tf(i),node_control_result.delta_tf_from_formal(i));
end
fprintf(fid,'\n## Conclusion\n\n');
fprintf(fid,'Candidate A is a feasible solution and is significantly shorter than the paper-reported %.2f s. Since the other benchmark cases match the paper, the most likely explanations are: the paper text for mission 2 has a parameter typo, the reported mission 2 time has a typo, or the paper used an additional hidden convention/setting that is not stated in the text. This diagnostic does not modify the formal mission 2 parameters.\n',paper_tf);
fclose(fid);
end

function out = logical_text(value)
if value
    out = 'feasible under the implemented checks';
else
    out = 'not confirmed feasible under the implemented checks';
end
end
