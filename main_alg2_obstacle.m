%MAIN_ALG2_OBSTACLE 复现论文第 VI.C 节第一组（三障碍）避障任务。
clear all; clc;
close all
code_dir=fileparts(mfilename('fullpath')); addpath(code_dir);
params=make_default_params(); obstacles=make_obstacles_case1();
params.cvx_solver_name='sdpt3';
r0=[0;0;10]; v0=[0;-10;0]; rf=[30;-30;14]; vf=[-10;0;0];
waypoints=[0,14,20,40,30; 0,-6,-18,-24,-30; 10,11,12,13,14];
r_init=interpolate_polyline_arclength(waypoints,params.N);

sol=solve_alg2_cvx(r0,v0,rf,vf,obstacles,r_init,params);
traj=recover_physical_trajectory(sol,params);
fprintf('障碍最小安全 margin = %.6e\n',sol.min_obstacle_margin);
for i=1:numel(sol.min_margin_each_obstacle)
    fprintf('obstacle %d min margin = %.6e\n',i,sol.min_margin_each_obstacle(i));
end
save(fullfile(params.results_dir,'obstacle_case1.mat'), ...
    'sol','traj','params','obstacles','r_init','waypoints');
plot_alg2_results(traj,sol,obstacles,params,'obstacle_case1');

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
