%MAIN_ALG1_NO_OBSTACLE 自定义初末状态。
clear all; clc;
close all
code_dir=fileparts(mfilename('fullpath')); addpath(code_dir);
params=make_default_params();

cases(1)=struct('name','cruise_mission_1','r0',[0;0;40],'v0',[0;0;0], ...
    'rf',[10;0;0],'vf',[0;0;0]);
cases(2)=struct('name','cruise_mission_2','r0',[0;0;40],'v0',[0;0;0], ...
    'rf',[10;0;0],'vf',[5;5;0]);
cases(3)=struct('name','cruise_mission_3','r0',[0;0;40],'v0',[0;0;0], ...
    'rf',[10;0;5],'vf',[0;-10;0]);



summary=struct('name',{},'tf',{},'ts',{},'gap',{},'outer_iterations',{},'solve_time',{});
sols=cell(1,numel(cases));
trajs=cell(1,numel(cases));
for i=1:numel(cases)
    fprintf('\n========== %s ==========\n',cases(i).name);
    sol=solve_alg1_cvx(cases(i).r0,cases(i).v0,cases(i).rf,cases(i).vf,params);
    traj=recover_physical_trajectory(sol,params);
    sols{i}=sol;
    trajs{i}=traj;
    %save(fullfile(params.results_dir,[cases(i).name '.mat']),'sol','traj','params');
    %plot_alg1_results(traj,sol,params,cases(i).name);
    summary(i)=struct('name',cases(i).name,'tf',sol.tf,'ts',sol.ts,'gap',sol.gap, ...
        'outer_iterations',numel(sol.history),'solve_time',sol.solve_time_total);
end
%plot_alg1_combined_results(trajs,sols,params,cases);
%save(fullfile(params.results_dir,'alg1_summary.mat'),'summary','cases','params');
disp(struct2table(summary));
