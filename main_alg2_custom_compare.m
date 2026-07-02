%MAIN_ALG2_CUSTOM_COMPARE 自定义避障场景的 CON/CVX 与 NLP 对比。
%   初末状态和 waypoints 来自 E:\convex optimization\code\main_alg2_obstacle_personal.m。
%   障碍物参数来自 E:\convex optimization\code\make_obstacles_case2.m，并已转换为 scenes/obstacle_case2.json。
%   结果使用 obstacle_case2_custom 命名，避免覆盖论文 obstacle_case1 结果。
clear all; clc;
close all
code_dir=fileparts(mfilename('fullpath')); addpath(code_dir);
[params,obstacles]=load_scene_and_params('obstacle_case2');
params.cvx_solver_name='Mosek';%Mosek  sdpt3
params.cvx_precision_name = 'medium'; %精度调高了会无解
r0=[0;0;0]; v0=[-1;1;2]; rf=[40;40;40]; vf=[5;5;5];
waypoints = [
     0,  -6,    5,   20,   40;
     0,  18,   35,   45,   40;
     0,  10,   20,   30,   40
];
r_init=interpolate_polyline_arclength(waypoints,params.N);

fprintf('\n========== obstacle_case2_custom CON/CVX ==========\n');
sol=solve_alg2_cvx(r0,v0,rf,vf,obstacles,r_init,params);
traj=recover_physical_trajectory(sol,params);
save(fullfile(params.results_dir,'obstacle_case2_custom.mat'), ...
    'sol','traj','params','obstacles','r_init','waypoints');
summary(1)=make_summary_row('obstacle_case2_custom','CON/CVX',sol,nan);

fprintf('\n========== obstacle_case2_custom NLP ==========\n');
nlp_clock=tic;
sol=solve_alg2_nlp(r0,v0,rf,vf,obstacles,r_init,params);
nlp_wall_time=toc(nlp_clock);
sol.nlp_wall_time = nlp_wall_time;
traj=recover_physical_trajectory(sol,params);
save(fullfile(params.results_dir,'obstacle_case2_custom_nlp.mat'), ...
    'sol','traj','params','obstacles','r_init','waypoints');
summary(2)=make_summary_row('obstacle_case2_custom','NLP',sol,nlp_wall_time);

summary_table=struct2table(summary);
disp(summary_table);
save(fullfile(params.results_dir,'obstacle_case2_custom_compare_summary.mat'), ...
    'summary','summary_table','params','obstacles','r_init','waypoints');

fprintf('\nplot_alg2_con_vs_nlp.m was not called because it is specialized for obstacle_case1 filenames.\n');
fprintf('The custom comparison is saved as tables and independent .mat result files only.\n');

function row=make_summary_row(case_name,method,sol,nlp_wall_time)
validation = sol.validation;
if isfield(sol,'outer_history') && isfield(sol.outer_history,'solve_time_source')
    source = sol.outer_history(end).solve_time_source;
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
    'min_obstacle_margin',get_field_or_nan(sol,'min_obstacle_margin'), ...
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

function r=interpolate_polyline_arclength(points,N)
% 按三维折线累计弧长均匀插值，保留来源脚本给出的折点几何形状。
segment_length=vecnorm(diff(points,1,2),2,1);
s=[0,cumsum(segment_length)];
s_nodes=linspace(0,s(end),N);
r=zeros(3,N);
for d=1:3
    r(d,:)=interp1(s,points(d,:),s_nodes,'linear');
end
end
