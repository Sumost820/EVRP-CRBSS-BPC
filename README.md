# EVRP-CRBSS 分支定价工程

从原 `Project5 - Final` 工程整理而来，保留原来的算例读取与缩放、初始列、双向标签、支配规则、ng-route/DSSR、缩减网络定价、割分离、弧分支和联合调度检查。当前仍采用每个物理电站一个充电桩、一个换电设备。

## 在 CLion 中运行

1. 打开本文件所在目录，使用 **Visual Studio** 工具链和 **Release** 配置。
2. 重新加载 CMake，编译 `EVRP-CRBSS` 目标。
3. 在 `src/main.cpp` 修改 `instancePath` 和 `timeLimit`，直接运行。默认算例为 `data/c101_21_25.txt`。

`CMakeLists.txt` 中的 `GUROBI_HOME` 默认是 `D:/Gurobi/win64`，链接当前机器的 Gurobi 13。运行需要有效的 Gurobi 许可。项目不包含 Gurobi 安装包；编译时会将本机运行库复制到可执行文件旁。

也可以向程序传入三个简单参数：算例路径、时限（秒）、结果文件夹。后两个参数可省略，`test.py` 使用这一入口。

## 批量测试

安装 `requirements.txt` 中的 pytest，在项目目录运行：

```text
python -m pytest -s -v test.py
python -m pytest -s -v test.py -k c101
```

`test.py` 只有一个参数化函数 `testModel`，按顺序运行 `data` 下的所有 txt 文件。直接修改文件顶部的 `dataDir`、`exePath`、`timeLimit` 即可。默认可执行文件位于 `cmake-build-release-visual-studio/bin/EVRP-CRBSS.exe`；如果 CLion 使用其他构建目录，需要同步修改 `exePath`。

批测默认每例 60 秒用于检查能否正常运行。有些算例在该时限内没有可行解，测试会如实失败；正式实验请增大 `timeLimit`。pytest 的外部超时比算法时限多 60 秒，用于避免进程异常阻塞。保留原算法的计时检查位置，因此时限不是强制精确到秒的停止点。

`checks` 保留了原工程已有的 10 个调度检查用例，用于维护时回归核验。默认不编译；需要时设置 CMake 选项 `BUILD_CHECKS=ON`，编译后运行 CTest。

## 源代码分工

| 文件 | 内容 |
|---|---|
| `src/main.cpp` | 单算例运行参数 |
| `src/runner.cpp` | 调用求解器、建立结果目录、同时输出控制台和日志 |
| `src/instance.h/.cpp` | 算例参数、读入、站点节点构造、弧预处理、ng 集合 |
| `src/energy.h/.cpp` | 分段线性时间—电量函数及前后向更新 |
| `src/pricing.h` | 路线、标签及定价接口 |
| `src/labels.cpp` | 标签回溯、支配、载重分桶、不可达节点更新 |
| `src/pricing.cpp` | 双向标签扩展与拼接 |
| `src/master.h/.cpp` | 受限主问题、添加列、对偶价、人工变量与分支限制 |
| `src/cuts.cpp` | lm-SRC 和 CVRPSEP 容量割分离 |
| `src/schedule.h/.cpp` | 固定路线的全局调度检查与结果复核 |
| `src/solver.h/.cpp` | 求解状态、总流程、可行解更新、路线冲突检查 |
| `src/initialRoutes.cpp` | 节约算法和单客户初始列 |
| `src/columnGeneration.cpp` | 列生成循环、分层定价、割与 DSSR 调用 |
| `src/branching.cpp` | 弧分支和递归搜索 |
| `src/report.cpp` | 原格式的最终路线明细、CSV 求解摘要 |
| `src/timeLimit.h/.cpp` | 原有全局计时机制 |

主要使用普通结构体、vector、map 和 set。`Solver` 保存一次求解状态，`MasterProblem` 管理 Gurobi 主问题。标签仍使用位集合和共享前驱，以保留原来的支配检查速度及路线回溯方式；未引入模板框架或额外类层级。变量和函数使用驼峰命名。

`third_party/cvrpsep` 是原工程使用的外部容量割库，按原样保留，许可证见该目录的 `License.txt`；其命名风格未改动。`src/app.manifest` 使 Windows 程序能够使用 UTF-8 中文路径。

## 结果文件

每个算例写入 `results/算例名/`：

- `solver.log`：求解过程和最终路线明细，仍采用原来的对齐表格和两位小数。
- `routes.csv`：当前最好可行解的车辆类型、成本和路线；未找到可行解时不生成。
- `result.csv`：算例、停止状态、是否有可行解、目标值、原程序给出的下界、耗时和路线数。

重跑覆盖该算例的日志并移除上次的两份结果，防止把旧解误认为本次求解结果。不会建立 backup 文件夹。

`FINISHED` 表示算法按原流程结束，`TIME_LIMIT` 表示达到时限，`STOPPED` 表示因检查失败等原因停止。退出码为 0 表示有通过联合调度复核的可行解且未异常停止，2 表示正常结束或超时但无可行解，1 表示错误或未完成认证的停止。pytest 会把退出码 1 和 2 报为失败。

此次改造保持原算法逻辑。`FINISHED` 和批测通过均不等于新的最优性证明；原有定价、支配、割系数及界处理的精确性问题仍需单独核查。联合调度检查继续采用增量式分段线性充电，出发时间和初始电量只受各自范围约束，`Unknown` 不用于生成不可行割。

## 迁移验证记录（2026-10-16）

指定 CLion 目录中的 Release 工程编译成功，原有 10 个调度回归用例全部通过。每例 60 秒的 pytest 批测结果为 4 通过、1 失败：

| 算例 | 程序状态 | 可行解目标值 |
|---|---|---:|
| c101_21_25 | FINISHED | 622.712678 |
| c102_21_25 | FINISHED | 512.623087 |
| c103_21_25 | TIME_LIMIT | 415.899359 |
| c104_21_25 | TIME_LIMIT，无可行解，pytest 失败 | — |
| c105_21_25 | FINISHED | 520.993343 |

四个有解算例的路线 CSV 与原程序一致；c101、c102、c105 的分支记录、定价产列数和目标更新记录也一致。原程序的 c103、c104 对照运行在 90 秒后由外部停止，其中 c103 保存了相同路线，c104 未保存可行解。c104 的失败表示给定时限内没有找到可行解，不能据此判断原问题不可行。
