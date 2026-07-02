function params = make_default_params()
%MAKE_DEFAULT_PARAMS 集中定义论文复现所需的数值参数。
%   所有主程序均通过此函数取得离散节点数、物理约束、收敛阈值
%   和 CVX 设置，避免参数散落在各个脚本中。

params.N = 81;
params.vmax = 10;                    % 最大速度，m/s
params.amax = 15;                    % 最大推力加速度，m/s^2
params.phi_max = 40*pi/180;          % 最大推力倾角，rad
params.g_vec = [0; 0; -9.807];       % 重力加速度，m/s^2

params.eps_t = 1e-3;                 % ts-tf^2 的外层收敛阈值，s^2
params.eps_r = 0.1;                  % 障碍线性化轨迹的无穷范数阈值，m
params.max_outer_iter = 20;
params.max_inner_iter = 30;

params.cvx_solver_name = 'mosek';    % 可改为 'sedumi' 或已安装的其他求解器 ECOS sdpt3 mosek
params.cvx_precision_name = 'high';
params.verbose = true;
params.figure_visible = 'on';        % 批处理时可改为 'off'
params.save_fig_files = true;        % 同时保存 .fig，便于在 MATLAB 中编辑

params.root_dir = fileparts(mfilename('fullpath'));
params.results_dir = fullfile(params.root_dir, 'results');
params.figures_dir = fullfile(params.root_dir, 'figures');
if ~exist(params.results_dir, 'dir'), mkdir(params.results_dir); end
if ~exist(params.figures_dir, 'dir'), mkdir(params.figures_dir); end
end
