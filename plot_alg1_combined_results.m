function plot_alg1_combined_results(trajs, sols, params, cases)
%PLOT_ALG1_COMBINED_RESULTS Plot paper-style combined Algorithm 1 figures.

if iscell(trajs), trajs = [trajs{:}]; end
if iscell(sols), sols = [sols{:}]; end
plot_cruise_combined_tracks(trajs(1:3),cases(1:3),params);
plot_alg1_combined_tf_ts(sols,params,cases);
end

function plot_cruise_combined_tracks(trajs,cases,params)
colors = lines(numel(trajs));
labels = arrayfun(@(c) strrep(c.name,'_',' '),cases,'UniformOutput',false);

f3 = figure('Color','w','Visible',params.figure_visible,'Name','Algorithm 1 cruise combined 3D tracks');
hold on;
h3 = gobjects(1,numel(trajs));
for i = 1:numel(trajs)
    tr = trajs(i);
    h3(i) = plot3(tr.r(1,:),tr.r(2,:),tr.r(3,:),'LineWidth',2,'Color',colors(i,:));
    add_thrust_arrows_3d(tr,colors(i,:));
end
h0 = plot3(cases(1).r0(1),cases(1).r0(2),cases(1).r0(3),'ko','MarkerFaceColor','k');
hf = plot3(cases(1).rf(1),cases(1).rf(2),cases(1).rf(3),'ks','MarkerFaceColor','w');
grid on; axis equal; view(35,25);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Cruise trajectories and thrust directions');
legend([h3,h0,hf],[labels,{'Initial point','Terminal point'}],'Location','best');
save_figure(f3,fullfile(params.figures_dir,'alg1_cruise_combined_3d'),params);

fxy = figure('Color','w','Visible',params.figure_visible,'Name','Algorithm 1 cruise combined ground tracks');
hold on;
hxy = gobjects(1,numel(trajs));
for i = 1:numel(trajs)
    tr = trajs(i);
    hxy(i) = plot(tr.r(1,:),tr.r(2,:),'LineWidth',2,'Color',colors(i,:));
    add_thrust_arrows_xy(tr,colors(i,:));
    quiver(tr.r(1,1),tr.r(2,1),tr.v(1,1),tr.v(2,1),0.35, ...
        'Color',colors(i,:),'LineWidth',1.2,'MaxHeadSize',1.5);
    quiver(tr.r(1,end),tr.r(2,end),tr.v(1,end),tr.v(2,end),0.35, ...
        'Color',colors(i,:),'LineWidth',1.2,'MaxHeadSize',1.5,'LineStyle','--');
end
h0 = plot(cases(1).r0(1),cases(1).r0(2),'ko','MarkerFaceColor','k');
hf = plot(cases(1).rf(1),cases(1).rf(2),'ks','MarkerFaceColor','w');
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('Cruise ground tracks, velocities, and thrust directions');
legend([hxy,h0,hf],[labels,{'Initial point','Terminal point'}],'Location','best');
save_figure(fxy,fullfile(params.figures_dir,'alg1_cruise_combined_ground'),params);
end

function plot_alg1_combined_tf_ts(sols,params,cases)
colors = lines(numel(sols));
labels = arrayfun(@(c) strrep(c.name,'_',' '),cases,'UniformOutput',false);
max_tf = 0;
for i = 1:numel(sols)
    max_tf = max(max_tf,max([sols(i).history.tf]));
end

f = figure('Color','w','Visible',params.figure_visible,'Name','Algorithm 1 combined tf-ts iterations');
tf_grid = linspace(0,1.15*max_tf,300);
plot(tf_grid,tf_grid.^2,'k-','LineWidth',1.5); hold on;
for i = 1:numel(sols)
    plot([sols(i).history.tf],[sols(i).history.ts],'o-', ...
        'LineWidth',1.8,'Color',colors(i,:),'MarkerFaceColor',colors(i,:));
end
grid on;
xlabel('t_f (s)'); ylabel('t_s (s^2)');
title('Algorithm 1 outer iterations');
legend(['t_s=t_f^2',labels],'Location','best');
save_figure(f,fullfile(params.figures_dir,'alg1_combined_tf_ts'),params);
end

function add_thrust_arrows_3d(tr,color)
idx = arrow_indices(size(tr.a,2));
scale = 0.11;
quiver3(tr.r(1,idx),tr.r(2,idx),tr.r(3,idx), ...
    tr.a(1,idx),tr.a(2,idx),tr.a(3,idx),scale, ...
    'Color',color,'LineWidth',1.0,'MaxHeadSize',0.8);
end

function add_thrust_arrows_xy(tr,color)
idx = arrow_indices(size(tr.a,2));
scale = 0.12;
quiver(tr.r(1,idx),tr.r(2,idx),tr.a(1,idx),tr.a(2,idx),scale, ...
    'Color',color,'LineWidth',0.9,'MaxHeadSize',0.8);
end

function idx = arrow_indices(num_control_nodes)
count = min(8,num_control_nodes);
idx = unique(round(linspace(3,num_control_nodes-2,count)));
idx = idx(idx >= 1 & idx <= num_control_nodes);
end

function save_figure(fig,base_name,params)
if exist('exportgraphics','file')==2
    exportgraphics(fig,[base_name '.png'],'Resolution',220);
else
    saveas(fig,[base_name '.png']);
end
if params.save_fig_files
    savefig(fig,[base_name '.fig']);
end
end
