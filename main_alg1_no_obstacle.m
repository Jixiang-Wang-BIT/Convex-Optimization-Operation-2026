%MAIN_ALG1_NO_OBSTACLE 复现论文第 VI.A/VI.B 节的五个无障碍任务。
%   通过 params.json 中的 method 字段选择求解方式：
%     "CON" — 凸优化（Algorithm 1，默认）
%     "NLP" — 非线性规划（fmincon SQP）
clear all; clc;
close all
code_dir=fileparts(mfilename('fullpath')); addpath(code_dir);
params=load_scene_and_params();

method = params.method;
fprintf('========== 求解方法: %s ==========\n', method);

cases(1)=struct('name','cruise_mission_1','r0',[0;0;15],'v0',[0;10;0], ...
    'rf',[30;30;15],'vf',[10;0;0]);
cases(2)=struct('name','cruise_mission_2','r0',[0;0;15],'v0',[0;10;0], ...
    'rf',[30;30;15],'vf',[-5*sqrt(2);5*sqrt(2);0]);
cases(3)=struct('name','cruise_mission_3','r0',[0;0;15],'v0',[0;-10;0], ...
    'rf',[30;30;15],'vf',[0;-10;0]);
cases(4)=struct('name','descent_mission_1','r0',[0;0;30],'v0',[8;2;1], ...
    'rf',[20;-10;6],'vf',[5;0;0]);
cases(5)=struct('name','descent_mission_2','r0',[0;0;30],'v0',[8;2;1], ...
    'rf',[10;-10;6],'vf',[5;0;0]);

% 根据方法选择文件名后缀
if strcmpi(method, 'NLP')
    suffix = '_nlp';
else
    suffix = '';   % CON / 默认
end

summary=struct('name',{},'tf',{},'ts',{},'gap',{},'outer_iterations',{},'solve_time',{});
sols=cell(1,numel(cases));
trajs=cell(1,numel(cases));
for i=1:numel(cases)
    fprintf('\n========== %s ==========\n',cases(i).name);

    if strcmpi(method, 'NLP')
        sol=solve_alg1_nlp(cases(i).r0,cases(i).v0,cases(i).rf,cases(i).vf,params);
    else
        sol=solve_alg1_cvx(cases(i).r0,cases(i).v0,cases(i).rf,cases(i).vf,params);
    end

    traj=recover_physical_trajectory(sol,params);
    sols{i}=sol;
    trajs{i}=traj;

    % 按方法区分保存文件名
    save(fullfile(params.results_dir,[cases(i).name suffix '.mat']),'sol','traj','params');

    % 单独绘图
    plot_alg1_results(traj,sol,params,[cases(i).name suffix]);

    summary(i)=struct('name',[cases(i).name suffix],'tf',sol.tf,'ts',sol.ts,'gap',sol.gap, ...
        'outer_iterations',numel(sol.history),'solve_time',sol.solve_time_total);
end

% 综合图（仅 CON 时有效；NLP 时也绘制但历史较短）
if strcmpi(method, 'CON')
    plot_alg1_combined_results(trajs,sols,params,cases);
end

save(fullfile(params.results_dir,['alg1_summary' suffix '.mat']),'summary','cases','params');
disp(struct2table(summary));
