# JGCD 2025 UAV 最短时间轨迹优化复现

本目录使用 MATLAB、CVX 和 SOCP 复现 Liu 等论文 *Exact Convex Relaxation Using Supporting Hyperplane for Trajectory Optimization of UAVs* 的主要数值算法：

- Algorithm 1：无障碍三维最短时间轨迹，包含论文第 VI.A 节的 3 个巡航任务和第 VI.B 节的 2 个下降任务；
- Algorithm 2：双层循环避障算法，复现第 VI.C 节第一组 3 个椭圆柱障碍场景；
- 轨迹、速度、推力加速度、推力倾角、外层支撑超平面迭代、内层逐次轨迹及障碍安全裕度图。

本复现不包含 NLP solver、IPOPT、GPOPS-II 或 CSCP 对比。算法直接采用论文凸化后的统一时间域离散模型，不使用符号工具箱，也不先建立连续模型再由程序离散。

## 环境与 CVX

建议使用 MATLAB R2019b 或更新版本（最低需要 R2016b，以支持脚本末尾的局部函数）。先从 [CVX 官方网站](https://cvxr.com/cvx/) 下载并安装 CVX，在 MATLAB 中运行：

```matlab
cd('你的/cvx/目录')
cvx_setup
cvx_solver mosek
cvx_save_prefs
```

默认求解器在 `make_default_params.m` 中设为 MOSEK（`params.cvx_solver_name = 'mosek'`）。如果本机没有 MOSEK，可以把 `params.cvx_solver_name` 改成 `sdpt3`、`sedumi` 或本机 CVX 支持的其他 SOCP 求解器；计算时间和数值结果可能会随求解器与精度设置略有差异。运行前可用 `cvx_version` 检查安装。

MATLAB Control System Toolbox 已定义函数 `tf()`，CVX 不允许创建同名优化变量。因此两个求解器内部使用 `tf_cvx` 和 `ts_cvx`，求解后仍通过论文命名 `sol.tf`、`sol.ts` 输出；数学模型没有变化。

## 运行方法

在 MATLAB 中切换到本目录，然后运行：

```matlab
main_alg1_no_obstacle
```

该脚本依次求解 5 个无障碍任务。每个任务保存：

- 3D 轨迹、x-y 地面轨迹、x-z/y-z 侧视轨迹；
- 速度模及 `vmax` 约束线；
- 推力加速度模及 `amax` 约束线；
- 推力倾角及 `phi_max` 约束线；
- `(tf,ts)` 外层迭代点及 `ts=tf^2` 曲线。

运行三障碍案例：

```matlab
main_alg2_obstacle
```

该脚本按折线累计弧长把论文给出的 5 个点插值为 81 个初始节点。为提高该避障案例在常见 CVX 环境中的可运行性，脚本会把 Algorithm 2 的求解器显式设为 `sdpt3`；如需测试 MOSEK 或 SeDuMi，可在 `main_alg2_obstacle.m` 中修改 `params.cvx_solver_name`。脚本保存：

- 3D 轨迹和椭圆柱障碍物；
- x-y 地面轨迹和障碍截面；
- 内层循环的逐次地面轨迹；
- 速度、推力加速度和推力倾角；
- `(tf,ts)` 外层迭代；
- 各障碍在各节点的安全裕度。

### 自定义场景脚本

仓库还包含一组非论文场景，用于测试同一算法在自定义初末状态和自定义障碍物下的表现。这些脚本不属于论文第 VI 节报告结果，运行时默认只在命令行输出结果，不覆盖正式 `results/` 和 `figures/`。

```matlab
main_alg1_no_obstacle_personal
```

该脚本调用 Algorithm 1，包含 3 个自定义无障碍任务：

| Case | r0 | rf | v0 | vf |
| --- | --- | --- | --- | --- |
| 1 | [0, 0, 40] | [10, 0, 0] | [0, 0, 0] | [0, 0, 0] |
| 2 | [0, 0, 40] | [10, 0, 0] | [0, 0, 0] | [5, 5, 0] |
| 3 | [0, 0, 40] | [10, 0, 5] | [0, 0, 0] | [0, -10, 0] |

```matlab
main_alg2_obstacle_personal
```

该脚本调用 Algorithm 2 和 `make_obstacles_case2.m`，测试自定义三维避障任务：

| 参数 | 数值 |
| --- | --- |
| r0 | [0, 0, 0] |
| rf | [40, 40, 40] |
| v0 | [-1, 1, 2] |
| vf | [5, 5, 5] |
| 初始折线点 | [0,0,0]、[-6,18,10]、[5,35,20]、[20,45,30]、[40,40,40] |

`make_obstacles_case2.m` 定义两个椭圆柱障碍物：

| 障碍物 | center x | center y | ac | bc |
| --- | ---: | ---: | ---: | ---: |
| 1 | 10 | 10 | 10 | 10 |
| 2 | 25 | 25 | 10 | 10 |

如需保存自定义场景结果或图片，可在对应 `*_personal.m` 脚本中取消 `save` 和 `plot_*` 行的注释，并建议使用新的 case name，避免覆盖论文复现结果文件。

MAT 数据保存在 `results/`，PNG 和 MATLAB FIG 图片保存在 `figures/`；目录不存在时自动创建。批处理时可在 `make_default_params.m` 中设置：

```matlab
params.figure_visible = 'off';
params.save_fig_files = false;
```

## 数学与结果说明

离散步长为 `h=1/(N-1)`，默认 `N=81`。模型使用论文变量替换

```text
vbar = tf * v
abar = ts * a
tf^2 <= ts
```

其中 `a` 是推力加速度，不是总加速度；总加速度为 `a+g`。恢复后的推力倾角按

```text
phi = atan2(norm(a_xy), a_z)
```

计算。程序检查 CVX 状态、`tf/ts` 正性、速度/推力/倾角违反量、`ts-tf^2`，以及 Algorithm 2 的最小障碍裕度

```text
(r-center)'*P*(r-center)-1
```

理论安全轨迹的裕度应不小于 0；接近零的小量通常来自求解器容差。若结果与论文略有差异，常见原因包括求解器选择、CVX 精度、软件版本以及 Euler 离散和数值容差细节。

## 文件说明

- `make_default_params.m`：集中参数与输出目录；
- `main_alg1_no_obstacle_personal.m`：Algorithm 1 自定义无障碍测试场景；
- `main_alg2_obstacle_personal.m`：Algorithm 2 自定义避障测试场景；
- `make_obstacles_case2.m`：自定义避障场景的两个椭圆柱障碍物；
- `solve_alg1_cvx.m`：Algorithm 1 的 SOCP 和 sigma 外层更新；
- `solve_alg2_cvx.m`：Algorithm 2 的 sigma 外层循环与障碍线性化内层循环；
- `linearize_obstacles.m`：计算障碍函数的一阶上界；
- `recover_physical_trajectory.m`：恢复真实时间、速度、推力加速度与诊断量；
- `plot_alg1_results.m`、`plot_alg2_results.m`：保存论文对应结果图。
