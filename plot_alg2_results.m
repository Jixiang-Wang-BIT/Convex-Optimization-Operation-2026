function plot_alg2_results(traj, sol, obstacles, params, case_name)
%PLOT_ALG2_RESULTS 绘制 Algorithm 2 的轨迹、迭代、约束及安全裕度。

safe_name=regexprep(case_name,'[^a-zA-Z0-9_-]','_');
theta=linspace(0,2*pi,160); colors=lines(numel(obstacles));
zmin=min(traj.r(3,:)); zmax=max(traj.r(3,:));

f1=figure('Color','w','Visible',params.figure_visible,'Name',[case_name ' trajectory']);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on;
for i=1:numel(obstacles)
    x=obstacles(i).xc+obstacles(i).ac*cos(theta);
    y=obstacles(i).yc+obstacles(i).bc*sin(theta);
    [X,Z]=meshgrid(x,[zmin-2,zmax+2]); [Y,~]=meshgrid(y,[zmin-2,zmax+2]);
    surf(X,Y,Z,'FaceColor',colors(i,:),'FaceAlpha',0.18,'EdgeColor','none');
end
plot3(traj.r(1,:),traj.r(2,:),traj.r(3,:),'r-','LineWidth',2.2);
grid on; axis equal; view(35,25); xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');title('3D trajectory');
nexttile; hold on;
for i=1:numel(obstacles)
    fill(obstacles(i).xc+obstacles(i).ac*cos(theta), ...
        obstacles(i).yc+obstacles(i).bc*sin(theta),colors(i,:), ...
        'FaceAlpha',0.18,'EdgeColor',colors(i,:),'LineWidth',1.2);
end
plot(traj.r(1,:),traj.r(2,:),'r-','LineWidth',2.2); grid on; axis equal;
xlabel('x (m)');ylabel('y (m)');title('Ground track');
save_figure(f1,fullfile(params.figures_dir,[safe_name '_trajectory']),params);

f2=figure('Color','w','Visible',params.figure_visible,'Name',[case_name ' inner iterations']); hold on;
cc=parula(max(2,sum(cellfun(@numel,sol.successive_ground_tracks)))); q=0;
for k=1:numel(sol.successive_ground_tracks)
    tracks=sol.successive_ground_tracks{k};
    for j=1:numel(tracks)
        q=q+1; xy=tracks{j};
        plot(xy(1,:),xy(2,:),'Color',cc(min(q,size(cc,1)),:),'LineWidth',1.1);
    end
end
for i=1:numel(obstacles)
    plot(obstacles(i).xc+obstacles(i).ac*cos(theta), ...
        obstacles(i).yc+obstacles(i).bc*sin(theta),'k-','LineWidth',1.2);
end
plot(traj.r(1,:),traj.r(2,:),'r-','LineWidth',2.5); axis equal; grid on;
xlabel('x (m)');ylabel('y (m)');title('Successive inner-loop ground tracks');
save_figure(f2,fullfile(params.figures_dir,[safe_name '_successive_tracks']),params);

f3=figure('Color','w','Visible',params.figure_visible,'Name',[case_name ' constraints']);
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(traj.t,traj.speed_norm,'LineWidth',1.8); hold on; yline(params.vmax,'r--'); grid on; ylabel('Speed (m/s)');
nexttile; plot(traj.t_a,traj.accel_norm,'LineWidth',1.8); hold on; yline(params.amax,'r--'); grid on; ylabel('Thrust accel. (m/s^2)');
nexttile; plot(traj.t_a,traj.tilt_deg,'LineWidth',1.8); hold on; yline(params.phi_max*180/pi,'r--'); grid on;
xlabel('t (s)');ylabel('Tilt (deg)');
save_figure(f3,fullfile(params.figures_dir,[safe_name '_constraints']),params);

f4=figure('Color','w','Visible',params.figure_visible,'Name',[case_name ' convergence and margin']);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; tf_grid=linspace(0,max([sol.outer_history.tf])*1.15,300);
plot(tf_grid,tf_grid.^2,'k-','LineWidth',1.5); hold on;
plot([sol.outer_history.tf],[sol.outer_history.ts],'o-','LineWidth',1.8,'MarkerFaceColor','r');
grid on; xlabel('t_f (s)');ylabel('t_s (s^2)');title('Outer iterations');legend('t_s=t_f^2','Iterates','Location','best');
nexttile; plot(traj.t,sol.obstacle_margin','LineWidth',1.4); hold on; yline(0,'k--'); grid on;
xlabel('t (s)');ylabel('Margin');title(sprintf('Obstacle margin, min=%.3g',sol.min_obstacle_margin));
margin_labels=arrayfun(@(i)sprintf('Obstacle %d',i),1:numel(obstacles),'UniformOutput',false);
legend([margin_labels,{'Safety boundary'}],'Location','best');
save_figure(f4,fullfile(params.figures_dir,[safe_name '_convergence_margin']),params);
end

function save_figure(fig,base_name,params)
if exist('exportgraphics','file')==2
    exportgraphics(fig,[base_name '.png'],'Resolution',200);
else
    saveas(fig,[base_name '.png']);
end
if params.save_fig_files, savefig(fig,[base_name '.fig']); end
end
