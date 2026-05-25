# Simulation of Mixed Traffic Flow

本仓库用于仿真混合交通流场景下的安全水平，核心脚本为 `simulation_safety.m`。脚本通过随机生成不同类型车辆，模拟车辆跟驰行为，并基于速度、位置和加速度数据计算交通安全评价指标。

## 文件说明

- `simulation_safety.m`：混合交通流安全水平分析主脚本。

脚本内置了主要计算函数，包括车辆初始化、车辆类型生成、跟驰模型、TTC/TET/TIT 计算和速度标准差计算。

## 运行环境

- MATLAB R2016b 或更新版本，需支持脚本文件中的本地函数。
- 如果运行到 `trajectory(...)` 时提示函数不存在，需要提供对应的轨迹绘制/处理函数，或在只关注安全指标时暂时注释该行。

## 快速开始

1. 打开 MATLAB。
2. 将当前目录切换到本仓库目录。
3. 运行：

```matlab
simulation_safety
```

运行完成后，工作区会生成主要结果变量：

- `tet`：Time Exposed Time-to-Collision，TTC 低于阈值的累计暴露时间。
- `tit`：Time Integrated Time-to-Collision，TTC 风险程度的时间积分指标。
- `sd`：车辆速度标准差，用于刻画交通流速度波动。
- `data_x`、`data_v`、`data_a`：车辆位置、速度和加速度仿真数据。
- `data_f`：车辆类型数据。

## 主要仿真参数

脚本开头集中定义了道路、车辆和实验参数，常用参数包括：

- `road_length`：道路长度，默认 `7000 m`。
- `simtime`：仿真总时长，默认 `1500 s`。
- `time_step`：仿真步长，默认 `0.1 s`。
- `num_totals`：仿真车辆数，默认 `100`。
- `flow_rate`：交通流率，默认 `1400 veh/h`。
- `p_car`：人驾汽车渗透率。
- `q_cacc`：CACC 智能网联货车比例。
- `platoon_length`：货车编队长度。
- `ttc_threshold`、`ttc_threshold_t`：TTC 风险阈值。

## 模型说明

脚本区分多类车辆和控制方式：

- 人驾汽车使用 cIDM 跟驰模型。
- 人驾货车使用 tIDM 跟驰模型。
- ACC 货车使用 ACC 控制模型。
- CACC 货车使用 CACC 控制模型。

通过改变 `q_cacc` 和 `platoon_length`，可以比较不同智能网联货车渗透率与编队长度组合下的安全水平差异。

## 注意事项

- 当前脚本为单文件 MATLAB 仿真脚本，运行时间与 `simtime`、`time_step`、`num_totals` 和实验维度有关。
- 若需要重复实验，可增大 `num_epochs`，脚本会对 TET、TIT 和速度标准差取重复实验均值。
- 修改参数后建议先使用较小的 `num_totals` 或 `simtime` 试运行，再进行完整仿真。
