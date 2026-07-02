%MAIN_ALG2_OBSTACLE 自定义避障任务。
clear all; clc;
close all
code_dir=fileparts(mfilename('fullpath')); addpath(code_dir);
params=make_default_params(); obstacles=make_obstacles_case2();
params.cvx_solver_name='Mosek';%Mosek  sdpt3
r0=[0;0;0]; v0=[-1;1;2]; rf=[40;40;40]; vf=[5;5;5];
waypoints = [
     0,  -6,    5,   20,   40;
     0,  18,   35,   45,   40;
     0,  10,   20,   30,   40
];
r_init=interpolate_polyline_arclength(waypoints,params.N);

sol=solve_alg2_cvx(r0,v0,rf,vf,obstacles,r_init,params);
traj=recover_physical_trajectory(sol,params);
fprintf('solver-only time total = %.6f s\n',sol.solve_time_total);
if isfield(sol,'inner_history') && ~isempty(sol.inner_history)
    inner_times = [sol.inner_history{end}.solve_time];
    fprintf('inner solver-only times =');
    fprintf(' %.6f',inner_times);
    fprintf(' s\n');
end
fprintf('障碍最小安全 margin = %.6e\n',sol.min_obstacle_margin);
for i=1:numel(sol.min_margin_each_obstacle)
    fprintf('obstacle %d min margin = %.6e\n',i,sol.min_margin_each_obstacle(i));
end
% save(fullfile(params.results_dir,'obstacle_case1.mat'), ...
%     'sol','traj','params','obstacles','r_init','waypoints');
% plot_alg2_results(traj,sol,obstacles,params,'obstacle_case1');

function r=interpolate_polyline_arclength(points,N)
% 按三维折线累计弧长均匀插值，保留论文给出的折点几何形状。
segment_length=vecnorm(diff(points,1,2),2,1);
s=[0,cumsum(segment_length)];
s_nodes=linspace(0,s(end),N);
r=zeros(3,N);
for d=1:3
    r(d,:)=interp1(s,points(d,:),s_nodes,'linear');
end
end
