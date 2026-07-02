%MAIN_ALG1_CUSTOM_COMPARE 自定义无障碍场景的 CON/CVX 与 NLP 对比。
%   场景参数来自 E:\convex optimization\code\main_alg1_no_obstacle_personal.m。
%   结果使用 custom_alg1_case_* 命名，避免覆盖论文原始任务。
clear all; clc;
close all
code_dir=fileparts(mfilename('fullpath')); addpath(code_dir);
params=load_scene_and_params();

cases(1)=struct('name','custom_alg1_case_1','r0',[0;0;40],'v0',[0;0;0], ...
    'rf',[10;0;0],'vf',[0;0;0]);
cases(2)=struct('name','custom_alg1_case_2','r0',[0;0;40],'v0',[0;0;0], ...
    'rf',[10;0;0],'vf',[5;5;0]);
cases(3)=struct('name','custom_alg1_case_3','r0',[0;0;40],'v0',[0;0;0], ...
    'rf',[10;0;5],'vf',[0;-10;0]);

summary = repmat(struct( ...
    'case_name','','method','','tf',nan,'ts',nan,'gap',nan, ...
    'max_speed_violation',nan,'max_acc_violation',nan, ...
    'max_tilt_violation',nan,'min_obstacle_margin',nan, ...
    'solver_only_time',nan,'nlp_wall_time',nan, ...
    'solve_time_source','','status',''), 1, 2*numel(cases));

row = 0;
for i=1:numel(cases)
    fprintf('\n========== %s CON/CVX ==========\n',cases(i).name);
    sol=solve_alg1_cvx(cases(i).r0,cases(i).v0,cases(i).rf,cases(i).vf,params);
    traj=recover_physical_trajectory(sol,params);
    save(fullfile(params.results_dir,[cases(i).name '.mat']), ...
        'sol','traj','params');
    row = row + 1;
    summary(row)=make_summary_row(cases(i).name,'CON/CVX',sol,nan);

    fprintf('\n========== %s NLP ==========\n',cases(i).name);
    nlp_clock=tic;
    sol=solve_alg1_nlp(cases(i).r0,cases(i).v0,cases(i).rf,cases(i).vf,params);
    nlp_wall_time=toc(nlp_clock);
    sol.nlp_wall_time = nlp_wall_time;
    traj=recover_physical_trajectory(sol,params);
    save(fullfile(params.results_dir,[cases(i).name '_nlp.mat']), ...
        'sol','traj','params');
    row = row + 1;
    summary(row)=make_summary_row(cases(i).name,'NLP',sol,nlp_wall_time);
end

summary_table=struct2table(summary);
disp(summary_table);
save(fullfile(params.results_dir,'custom_alg1_compare_summary.mat'), ...
    'summary','summary_table','cases','params');

fprintf('\nplot_alg1_con_vs_nlp.m was not called because its output figure names are shared by the paper cases.\n');
fprintf('The custom comparison is saved as tables and independent .mat result files only.\n');

function row=make_summary_row(case_name,method,sol,nlp_wall_time)
validation = sol.validation;
if isfield(sol,'history') && isfield(sol.history,'solve_time_source')
    source = sol.history(end).solve_time_source;
elseif isfield(sol,'solve_time_source')
    source = sol.solve_time_source;
else
    source = '';
end
if strcmpi(method,'NLP')
    solver_only_time = nan;
else
    solver_only_time = sol.solve_time_total;
end
row=struct( ...
    'case_name',case_name, ...
    'method',method, ...
    'tf',sol.tf, ...
    'ts',get_field_or_nan(sol,'ts'), ...
    'gap',get_field_or_nan(sol,'gap'), ...
    'max_speed_violation',get_field_or_nan(validation,'max_velocity'), ...
    'max_acc_violation',get_field_or_nan(validation,'max_acceleration'), ...
    'max_tilt_violation',get_field_or_nan(validation,'max_tilt_rad'), ...
    'min_obstacle_margin',nan, ...
    'solver_only_time',solver_only_time, ...
    'nlp_wall_time',nlp_wall_time, ...
    'solve_time_source',source, ...
    'status',get_field_or_empty(sol,'cvx_status'));
end

function value=get_field_or_nan(s,field_name)
if isfield(s,field_name)
    value=s.(field_name);
else
    value=nan;
end
end

function value=get_field_or_empty(s,field_name)
if isfield(s,field_name)
    value=s.(field_name);
else
    value='';
end
end
