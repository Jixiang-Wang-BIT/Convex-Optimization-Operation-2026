%MAIN_ALG1_NO_OBSTACLE 复现论文第 VI.A/VI.B 节的五个无障碍任务。
clear all; clc;
close all
code_dir=fileparts(mfilename('fullpath')); addpath(code_dir);
params=make_default_params();

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

summary=struct('name',{},'tf',{},'ts',{},'gap',{},'outer_iterations',{},'solve_time',{});
sols=cell(1,numel(cases));
trajs=cell(1,numel(cases));
for i=1:numel(cases)
    fprintf('\n========== %s ==========\n',cases(i).name);
    sol=solve_alg1_cvx(cases(i).r0,cases(i).v0,cases(i).rf,cases(i).vf,params);
    traj=recover_physical_trajectory(sol,params);
    sols{i}=sol;
    trajs{i}=traj;
    save(fullfile(params.results_dir,[cases(i).name '.mat']),'sol','traj','params');
    plot_alg1_results(traj,sol,params,cases(i).name);
    summary(i)=struct('name',cases(i).name,'tf',sol.tf,'ts',sol.ts,'gap',sol.gap, ...
        'outer_iterations',numel(sol.history),'solve_time',sol.solve_time_total);
end
plot_alg1_combined_results(trajs,sols,params,cases);
save(fullfile(params.results_dir,'alg1_summary.mat'),'summary','cases','params');
disp(struct2table(summary));
