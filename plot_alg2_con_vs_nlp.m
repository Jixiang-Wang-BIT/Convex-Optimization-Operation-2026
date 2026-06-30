function plot_alg2_con_vs_nlp(params)
%PLOT_ALG2_CON_VS_NLP  对比凸优化（CON）与非线性规划（NLP）的避障求解结果。
%   从 results/ 目录读取 obstacle_case1.mat 和 obstacle_case1_nlp.mat，
%   绘制轨迹、约束、障碍安全裕度及性能对比图。

if nargin < 1
    params = load_scene_and_params();
end

% ---------- 加载 CON 和 NLP 数据 ----------
con_file = fullfile(params.results_dir, 'obstacle_case1.mat');
nlp_file = fullfile(params.results_dir, 'obstacle_case1_nlp.mat');

if ~isfile(con_file)
    error('未找到 CON 结果: %s。请先用 method=CON 运行 main_alg2_obstacle。', con_file);
end
if ~isfile(nlp_file)
    error('未找到 NLP 结果: %s。请先用 method=NLP 运行 main_alg2_obstacle。', nlp_file);
end

tmp_con = load(con_file);
tmp_nlp = load(nlp_file);

traj_con = tmp_con.traj;
sol_con  = tmp_con.sol;
obstacles = tmp_con.obstacles;
traj_nlp = tmp_nlp.traj;
sol_nlp  = tmp_nlp.sol;

colors = lines(2);
con_color = colors(1,:);   % 蓝
nlp_color = colors(2,:);   % 红

% ---------- 图1: 轨迹对比 ----------
f1 = figure('Color','w','Visible',params.figure_visible, ...
    'Name','CON vs NLP 避障轨迹对比');
tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

theta = linspace(0, 2*pi, 160);
n_obs = numel(obstacles);
obs_colors = lines(n_obs);

% 3D 轨迹 — CON
nexttile; hold on;
plot_obstacle_cylinders_3d(obstacles, theta, traj_con);
plot3(traj_con.r(1,:), traj_con.r(2,:), traj_con.r(3,:), ...
    'LineWidth', 2, 'Color', con_color);
grid on; axis equal; view(35,25);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title(sprintf('CON   tf=%.4f s', sol_con.tf));

% 3D 轨迹 — NLP
nexttile; hold on;
plot_obstacle_cylinders_3d(obstacles, theta, traj_nlp);
plot3(traj_nlp.r(1,:), traj_nlp.r(2,:), traj_nlp.r(3,:), ...
    'LineWidth', 2, 'Color', nlp_color);
grid on; axis equal; view(35,25);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title(sprintf('NLP   tf=%.4f s', sol_nlp.tf));

% 地面投影 — 叠加对比
nexttile; hold on;
for i = 1:n_obs
    fill(obstacles(i).xc + obstacles(i).ac * cos(theta), ...
         obstacles(i).yc + obstacles(i).bc * sin(theta), obs_colors(i,:), ...
         'FaceAlpha', 0.15, 'EdgeColor', obs_colors(i,:), 'LineWidth', 1.0);
end
plot(traj_con.r(1,:), traj_con.r(2,:), 'LineWidth', 2, 'Color', con_color);
plot(traj_nlp.r(1,:), traj_nlp.r(2,:), 'LineWidth', 2, 'Color', nlp_color);
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('Ground track 对比');
legend({'CON','NLP'}, 'Location', 'best');

sgtitle('CON vs NLP — 避障轨迹对比');
save_figure(f1, fullfile(params.figures_dir, 'alg2_con_vs_nlp_trajectories'), params);

% ---------- 图2: 速度 / 推力 / 倾角对比 ----------
f2 = figure('Color','w','Visible',params.figure_visible, ...
    'Name','CON vs NLP 避障约束对比');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

% 速度
nexttile; hold on; grid on;
plot(traj_con.t, traj_con.speed_norm, 'LineWidth', 1.8, 'Color', con_color);
plot(traj_nlp.t, traj_nlp.speed_norm, 'LineWidth', 1.8, 'Color', nlp_color);
yline(params.vmax, 'k--', 'LineWidth', 0.8);
ylabel('速度 (m/s)'); legend({'CON','NLP'}, 'Location', 'best');
title('速度约束');

% 推力加速度
nexttile; hold on; grid on;
plot(traj_con.t_a, traj_con.accel_norm, 'LineWidth', 1.8, 'Color', con_color);
plot(traj_nlp.t_a, traj_nlp.accel_norm, 'LineWidth', 1.8, 'Color', nlp_color);
yline(params.amax, 'k--', 'LineWidth', 0.8);
ylabel('推力加速度 (m/s^2)'); legend({'CON','NLP'}, 'Location', 'best');
title('推力加速度约束');

% 倾角
nexttile; hold on; grid on;
plot(traj_con.t_a, traj_con.tilt_deg, 'LineWidth', 1.8, 'Color', con_color);
plot(traj_nlp.t_a, traj_nlp.tilt_deg, 'LineWidth', 1.8, 'Color', nlp_color);
yline(params.phi_max*180/pi, 'k--', 'LineWidth', 0.8);
xlabel('t (s)'); ylabel('倾角 (deg)'); legend({'CON','NLP'}, 'Location', 'best');
title('推力倾角约束');

sgtitle('CON vs NLP — 速度 / 推力 / 倾角 对比');
save_figure(f2, fullfile(params.figures_dir, 'alg2_con_vs_nlp_constraints'), params);

% ---------- 图3: 障碍安全裕度对比 ----------
f3 = figure('Color','w','Visible',params.figure_visible, ...
    'Name','CON vs NLP 障碍安全裕度');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

margin_labels = arrayfun(@(i) sprintf('障碍 %d', i), 1:n_obs, ...
    'UniformOutput', false);

nexttile; hold on; grid on;
for i = 1:n_obs
    plot(traj_con.t, sol_con.obstacle_margin(i,:)', 'LineWidth', 1.5);
end
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('t (s)'); ylabel('安全裕度');
title(sprintf('CON   min=%.3g', sol_con.min_obstacle_margin));
legend(margin_labels, 'Location', 'best');

nexttile; hold on; grid on;
for i = 1:n_obs
    plot(traj_nlp.t, sol_nlp.obstacle_margin(i,:)', 'LineWidth', 1.5);
end
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('t (s)'); ylabel('安全裕度');
title(sprintf('NLP   min=%.3g', sol_nlp.min_obstacle_margin));
legend(margin_labels, 'Location', 'best');

sgtitle('CON vs NLP — 障碍安全裕度对比');
save_figure(f3, fullfile(params.figures_dir, 'alg2_con_vs_nlp_margin'), params);

% ---------- 图4: 性能对比 ----------
f4 = figure('Color','w','Visible',params.figure_visible, ...
    'Name','CON vs NLP 避障性能对比');
tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

% tf 对比
nexttile; hold on; grid on;
bar(categorical({'CON','NLP'}), [sol_con.tf, sol_nlp.tf]);
ylabel('t_f (s)'); title('飞行时间对比');

% 求解时间对比
nexttile; hold on; grid on;
bar(categorical({'CON','NLP'}), [sol_con.solve_time_total, sol_nlp.solve_time_total]);
ylabel('求解时间 (s)'); title('求解时间对比');

% 最小安全裕度对比
nexttile; hold on; grid on;
bar(categorical({'CON','NLP'}), [sol_con.min_obstacle_margin, sol_nlp.min_obstacle_margin]);
ylabel('最小安全裕度'); title('障碍安全裕度');

sgtitle('CON vs NLP — 避障性能对比');
save_figure(f4, fullfile(params.figures_dir, 'alg2_con_vs_nlp_performance'), params);

% ---------- 汇总表 ----------
fprintf('\n========== Algorithm 2 CON vs NLP 对比汇总 ==========\n');
fprintf('CON: tf=%.6f s, 求解时间=%.3f s, min_margin=%.6e, 状态=%s\n', ...
    sol_con.tf, sol_con.solve_time_total, sol_con.min_obstacle_margin, ...
    strtrim(sol_con.cvx_status));
fprintf('NLP: tf=%.6f s, 求解时间=%.3f s, min_margin=%.6e, 状态=%s\n', ...
    sol_nlp.tf, sol_nlp.solve_time_total, sol_nlp.min_obstacle_margin, ...
    sol_nlp.cvx_status);

for i = 1:n_obs
    fprintf('障碍 %d  min_margin — CON: %.6e  |  NLP: %.6e\n', ...
        i, sol_con.min_margin_each_obstacle(i), sol_nlp.min_margin_each_obstacle(i));
end
end

function plot_obstacle_cylinders_3d(obstacles, theta, traj)
% 在半透明椭圆柱绘制障碍物。
zmin = min(traj.r(3,:));
zmax = max(traj.r(3,:));
n_obs = numel(obstacles);
obs_colors = lines(n_obs);
for i = 1:n_obs
    x = obstacles(i).xc + obstacles(i).ac * cos(theta);
    y = obstacles(i).yc + obstacles(i).bc * sin(theta);
    [X, Z] = meshgrid(x, [zmin-2, zmax+2]);
    [Y, ~] = meshgrid(y, [zmin-2, zmax+2]);
    surf(X, Y, Z, 'FaceColor', obs_colors(i,:), ...
        'FaceAlpha', 0.15, 'EdgeColor', 'none');
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
