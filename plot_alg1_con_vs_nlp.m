function plot_alg1_con_vs_nlp(cases, params)
%PLOT_ALG1_CON_VS_NLP  对比凸优化（CON）与非线性规划（NLP）的求解结果。
%   从 results/ 目录读取 CON 和 NLP 各自保存的 .mat 文件，
%   对每个任务分别绘制轨迹对比、速度/推力/倾角对比，以及 tf 柱状图。

if nargin < 2
    params = load_scene_and_params();
end
if nargin < 1
    % 默认五个任务
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
end

% ---------- 加载 CON 和 NLP 数据 ----------
trajs_con = cell(1, numel(cases));
sols_con  = cell(1, numel(cases));
trajs_nlp = cell(1, numel(cases));
sols_nlp  = cell(1, numel(cases));

for i = 1:numel(cases)
    name = cases(i).name;
    con_file = fullfile(params.results_dir, [name '.mat']);
    nlp_file = fullfile(params.results_dir, [name '_nlp.mat']);

    if isfile(con_file)
        tmp = load(con_file);
        trajs_con{i} = tmp.traj;
        sols_con{i}  = tmp.sol;
    else
        warning('未找到 CON 结果: %s', con_file);
    end
    if isfile(nlp_file)
        tmp = load(nlp_file);
        trajs_nlp{i} = tmp.traj;
        sols_nlp{i}  = tmp.sol;
    else
        warning('未找到 NLP 结果: %s', nlp_file);
    end
end

colors = lines(2);   % CON=蓝色, NLP=红色
con_color = colors(1,:);
nlp_color = colors(2,:);
labels = arrayfun(@(c) strrep(c.name,'_',' '), cases, 'UniformOutput', false);

% ---------- 图1: 各任务轨迹对比 ----------
f1 = figure('Color','w','Visible',params.figure_visible, ...
    'Name','CON vs NLP 轨迹对比');
tiledlayout(numel(cases), 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:numel(cases)
    % 3D 轨迹
    nexttile; hold on; grid on; axis equal; view(35,25);
    if ~isempty(trajs_con{i})
        plot3(trajs_con{i}.r(1,:), trajs_con{i}.r(2,:), trajs_con{i}.r(3,:), ...
            'LineWidth', 1.8, 'Color', con_color);
    end
    if ~isempty(trajs_nlp{i})
        plot3(trajs_nlp{i}.r(1,:), trajs_nlp{i}.r(2,:), trajs_nlp{i}.r(3,:), ...
            'LineWidth', 1.8, 'Color', nlp_color);
    end
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title(sprintf('%s 3D', labels{i}));

    % 地面投影
    nexttile; hold on; grid on; axis equal;
    if ~isempty(trajs_con{i})
        plot(trajs_con{i}.r(1,:), trajs_con{i}.r(2,:), ...
            'LineWidth', 1.8, 'Color', con_color);
    end
    if ~isempty(trajs_nlp{i})
        plot(trajs_nlp{i}.r(1,:), trajs_nlp{i}.r(2,:), ...
            'LineWidth', 1.8, 'Color', nlp_color);
    end
    xlabel('x (m)'); ylabel('y (m)');
    title(sprintf('%s 地面投影', labels{i}));
end
lg = legend({'CON','NLP'}, 'Orientation', 'horizontal');
lg.Layout.Tile = 'south';
sgtitle('CON vs NLP — 轨迹对比');
save_figure(f1, fullfile(params.figures_dir, 'con_vs_nlp_trajectories'), params);

% ---------- 图2: 速度 / 推力 / 倾角对比 ----------
f2 = figure('Color','w','Visible',params.figure_visible, ...
    'Name','CON vs NLP 约束对比');
tiledlayout(3, numel(cases), 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:numel(cases)
    % 速度
    nexttile; hold on; grid on;
    if ~isempty(trajs_con{i})
        plot(trajs_con{i}.t, trajs_con{i}.speed_norm, 'LineWidth',1.8, 'Color',con_color);
    end
    if ~isempty(trajs_nlp{i})
        plot(trajs_nlp{i}.t, trajs_nlp{i}.speed_norm, 'LineWidth',1.8, 'Color',nlp_color);
    end
    yline(params.vmax, 'k--', 'LineWidth', 0.8);
    ylabel('速度 (m/s)'); title(labels{i});
end
for i = 1:numel(cases)
    % 推力
    nexttile; hold on; grid on;
    if ~isempty(trajs_con{i})
        plot(trajs_con{i}.t_a, trajs_con{i}.accel_norm, 'LineWidth',1.8, 'Color',con_color);
    end
    if ~isempty(trajs_nlp{i})
        plot(trajs_nlp{i}.t_a, trajs_nlp{i}.accel_norm, 'LineWidth',1.8, 'Color',nlp_color);
    end
    yline(params.amax, 'k--', 'LineWidth', 0.8);
    ylabel('推力加速度 (m/s^2)');
end
for i = 1:numel(cases)
    % 倾角
    nexttile; hold on; grid on;
    if ~isempty(trajs_con{i})
        plot(trajs_con{i}.t_a, trajs_con{i}.tilt_deg, 'LineWidth',1.8, 'Color',con_color);
    end
    if ~isempty(trajs_nlp{i})
        plot(trajs_nlp{i}.t_a, trajs_nlp{i}.tilt_deg, 'LineWidth',1.8, 'Color',nlp_color);
    end
    yline(params.phi_max*180/pi, 'k--', 'LineWidth', 0.8);
    xlabel('t (s)'); ylabel('倾角 (deg)');
end
lg = legend({'CON','NLP'}, 'Orientation', 'horizontal');
lg.Layout.Tile = 'south';
sgtitle('CON vs NLP — 速度 / 推力 / 倾角 对比');
save_figure(f2, fullfile(params.figures_dir, 'con_vs_nlp_constraints'), params);

% ---------- 图3: tf 柱状图 + 求解时间对比 ----------
f3 = figure('Color','w','Visible',params.figure_visible, ...
    'Name','CON vs NLP tf / 求解时间');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile; hold on; grid on;
tf_con = nan(1, numel(cases));
tf_nlp = nan(1, numel(cases));
for i = 1:numel(cases)
    if ~isempty(sols_con{i}), tf_con(i) = sols_con{i}.tf; end
    if ~isempty(sols_nlp{i}), tf_nlp(i) = sols_nlp{i}.tf; end
end
X = categorical(labels);
bar(X, [tf_con(:), tf_nlp(:)]);
ylabel('t_f (s)'); title('飞行时间对比');
legend({'CON','NLP'}, 'Location', 'best');

nexttile; hold on; grid on;
t_con = nan(1, numel(cases));
t_nlp = nan(1, numel(cases));
for i = 1:numel(cases)
    if ~isempty(sols_con{i}), t_con(i) = sols_con{i}.solve_time_total; end
    if ~isempty(sols_nlp{i}), t_nlp(i) = sols_nlp{i}.solve_time_total; end
end
bar(X, [t_con(:), t_nlp(:)]);
ylabel('求解时间 (s)'); title('求解时间对比');
legend({'CON','NLP'}, 'Location', 'best');
sgtitle('CON vs NLP — 性能对比');
save_figure(f3, fullfile(params.figures_dir, 'con_vs_nlp_performance'), params);

% ---------- 打印汇总表 ----------
fprintf('\n========== CON vs NLP 对比汇总 ==========\n');
for i = 1:numel(cases)
    fprintf('%s:\n', labels{i});
    if ~isempty(sols_con{i})
        fprintf('  CON: tf=%.6f s, 求解时间=%.3f s, 状态=%s\n', ...
            sols_con{i}.tf, sols_con{i}.solve_time_total, ...
            strtrim(sols_con{i}.cvx_status));
    end
    if ~isempty(sols_nlp{i})
        fprintf('  NLP: tf=%.6f s, 求解时间=%.3f s, 状态=%s\n', ...
            sols_nlp{i}.tf, sols_nlp{i}.solve_time_total, ...
            sols_nlp{i}.cvx_status);
    end
end
end

function save_figure(fig, base_name, params)
if exist('exportgraphics', 'file') == 2
    exportgraphics(fig, [base_name '.png'], 'Resolution', 200);
else
    saveas(fig, [base_name '.png']);
end
if params.save_fig_files
    savefig(fig, [base_name '.fig']);
end
end
