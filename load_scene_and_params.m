function [params, obstacles] = load_scene_and_params(scene_name)
%LOAD_SCENE_AND_PARAMS  读取数值参数，并可选地加载障碍物场景。
%   params = LOAD_SCENE_AND_PARAMS() 从 params.json 读取所有数值参数、
%   求解器设置和输出选项，并计算派生路径，以结构体形式返回。
%
%   [params, obstacles] = LOAD_SCENE_AND_PARAMS(scene_name) 同时从
%   scenes/<scene_name>.json 加载障碍物场景。返回的结构体数组与原有
%   make_obstacles_case1 的输出格式完全一致。
%
%   本函数取代了原先的 make_default_params、make_obstacles_case1 和
%   load_obstacle_scene，统一为一个入口。

% ---------- 定位项目根目录 ----------
root_dir = fileparts(mfilename('fullpath'));

% ---------- 加载通用参数 ----------
params_file = fullfile(root_dir, 'params.json');
if ~isfile(params_file)
    error('load_scene_and_params: 未找到 params.json（"%s"）。', params_file);
end
fid = fopen(params_file, 'r');
raw = fread(fid, inf, 'uint8=>char')';
fclose(fid);
cfg = jsondecode(raw);

% 基本数值参数
params.N                  = cfg.N;
params.vmax               = cfg.vmax;
params.amax               = cfg.amax;
params.phi_max            = cfg.phi_max_deg * pi / 180;   % 度 → 弧度
params.g_vec              = cfg.g_vec(:);                 % 保证列向量
params.eps_t              = cfg.eps_t;
params.eps_r              = cfg.eps_r;
params.max_outer_iter     = cfg.max_outer_iter;
params.max_inner_iter     = cfg.max_inner_iter;

% 求解器与输出设置
params.method              = cfg.method;           % "CON" 或 "NLP"
params.cvx_solver_name    = cfg.cvx_solver_name;
params.cvx_precision_name = cfg.cvx_precision_name;
params.verbose            = cfg.verbose;
params.figure_visible     = cfg.figure_visible;
params.save_fig_files     = cfg.save_fig_files;

% 派生路径
params.root_dir    = root_dir;
params.results_dir = fullfile(root_dir, 'results');
params.figures_dir = fullfile(root_dir, 'figures');
if ~exist(params.results_dir, 'dir'), mkdir(params.results_dir); end
if ~exist(params.figures_dir, 'dir'), mkdir(params.figures_dir); end

% ---------- 加载障碍物场景（可选） ----------
if nargin < 1 || isempty(scene_name)
    obstacles = [];
    return
end

validateattributes(scene_name, {'char','string'}, {'scalartext'});
scene_name = char(scene_name);

% 允许只给场景名（自动补 .json），也允许带扩展名
[~, base, ext] = fileparts(scene_name);
if isempty(ext)
    scene_file = fullfile(root_dir, 'scenes', [base '.json']);
else
    scene_file = fullfile(root_dir, 'scenes', scene_name);
end

if ~isfile(scene_file)
    error('load_scene_and_params: 未找到场景文件 "%s"。', scene_file);
end

fid = fopen(scene_file, 'r');
raw = fread(fid, inf, 'uint8=>char')';
fclose(fid);
scene = jsondecode(raw);

if ~isfield(scene, 'obstacles') || ~isstruct(scene.obstacles)
    error('load_scene_and_params: 场景 JSON 必须包含 "obstacles" 对象数组。');
end

data = scene.obstacles;
n = numel(data);

% 预分配结构体数组，与原有 make_obstacles_case1 格式一致
obstacles = repmat(struct('center', zeros(3,1), 'P', zeros(3), ...
    'ac', 0, 'bc', 0, 'xc', 0, 'yc', 0), 1, n);

for i = 1:n
    xc = data(i).xc;
    yc = data(i).yc;
    ac = data(i).ac;
    bc = data(i).bc;

    obstacles(i).center = [xc; yc; 0];
    obstacles(i).P      = diag([1/ac^2, 1/bc^2, 0]);
    obstacles(i).ac     = ac;
    obstacles(i).bc     = bc;
    obstacles(i).xc     = xc;
    obstacles(i).yc     = yc;
end
end
