function plot_alg1_results(traj, sol, params, case_name)
%PLOT_ALG1_RESULTS 绘制并保存 Algorithm 1 的轨迹、约束和收敛结果。

safe_name = regexprep(case_name,'[^a-zA-Z0-9_-]','_');
c = [0.00 0.45 0.74];

f1 = figure('Color','w','Visible',params.figure_visible,'Name',[case_name ' tracks']);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; plot3(traj.r(1,:),traj.r(2,:),traj.r(3,:),'LineWidth',2,'Color',c); grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); title('3D trajectory'); view(35,25);
nexttile; plot(traj.r(1,:),traj.r(2,:),'LineWidth',2,'Color',c); grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); title('Ground track');
nexttile; plot(traj.r(1,:),traj.r(3,:),'LineWidth',2,'Color',c); grid on;
xlabel('x (m)'); ylabel('z (m)'); title('x-z side track');
nexttile; plot(traj.r(2,:),traj.r(3,:),'LineWidth',2,'Color',c); grid on;
xlabel('y (m)'); ylabel('z (m)'); title('y-z side track');
sgtitle(strrep(case_name,'_',' '));
save_figure(f1,fullfile(params.figures_dir,[safe_name '_tracks']),params);

f2 = figure('Color','w','Visible',params.figure_visible,'Name',[case_name ' constraints']);
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(traj.t,traj.speed_norm,'LineWidth',1.8); hold on; yline(params.vmax,'r--','v_{max}'); grid on;
ylabel('Speed (m/s)'); title('Velocity constraint');
nexttile; plot(traj.t_a,traj.accel_norm,'LineWidth',1.8); hold on; yline(params.amax,'r--','a_{max}'); grid on;
ylabel('Thrust accel. (m/s^2)'); title('Thrust acceleration constraint');
nexttile; plot(traj.t_a,traj.tilt_deg,'LineWidth',1.8); hold on; yline(params.phi_max*180/pi,'r--','\phi_{max}'); grid on;
xlabel('t (s)'); ylabel('Tilt (deg)'); title('Thrust tilt constraint');
save_figure(f2,fullfile(params.figures_dir,[safe_name '_constraints']),params);

f3 = figure('Color','w','Visible',params.figure_visible,'Name',[case_name ' convergence']);
tf_grid = linspace(0,max([sol.history.tf])*1.15,300);
plot(tf_grid,tf_grid.^2,'k-','LineWidth',1.5); hold on;
plot([sol.history.tf],[sol.history.ts],'o-','LineWidth',1.8,'MarkerFaceColor',c);
grid on; xlabel('t_f (s)'); ylabel('t_s (s^2)'); title('Algorithm 1 outer iterations');
legend('t_s=t_f^2','Iterates','Location','best');
save_figure(f3,fullfile(params.figures_dir,[safe_name '_tf_ts']),params);
end

function save_figure(fig,base_name,params)
if exist('exportgraphics','file')==2
    exportgraphics(fig,[base_name '.png'],'Resolution',200);
else
    saveas(fig,[base_name '.png']);
end
if params.save_fig_files, savefig(fig,[base_name '.fig']); end
end
