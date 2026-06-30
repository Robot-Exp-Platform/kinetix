# Kinetix

Kinetix 是一个实验性的 Rust 刚体动力学工作区，设计上深受
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) 启发。它的目标不是逐项复刻
Pinocchio，而是探索：如果把空间代数、Model/Data 分离、递归刚体动力学算法这些核心思想放进
Rust 的类型系统和零成本抽象里，一个机器人动力学库可以长成什么样。

当前项目仍处在早期阶段。已经实现的重点包括：类型安全的空间代数、固定尺寸且无运行时分配的动力学核心、
URDF 加载、一阶前向自动微分、教学例程，以及与 Pinocchio 对齐的正确性/性能对比工具。

## 初衷

机器人动力学代码里最麻烦的 Bug，往往不是语法错误，而是“看起来都对”的语义错误：
坐标系混用、力和运动变换方向搞反、实时控制循环里偷偷分配内存、多个线程共享同一份临时缓存等。
Kinetix 试图从设计层面减少这些问题：

- **类型驱动的坐标系安全**：空间向量在类型里携带表达坐标系，不同坐标系的运动向量不能直接相加。
- **热路径零分配**：`kinetix-spatial` 和 `kinetix-core` 都是 `#![no_std]`，使用静态尺寸矩阵/向量。
- **Model/Data 分离**：机器人固有结构放在不可变 `Model` 中，算法临时状态放在可复用 `Data` 中，继承
  Pinocchio 的核心设计哲学。
- **静态自由度**：关节向量和矩阵通过 `const DOF: usize` 固定尺寸，给编译器更多优化空间。
- **泛型标量路径**：同一套动力学代码既能跑 `f64`，也能跑自动微分用的双数 `Dual`。

## Pinocchio 参照

Pinocchio 是本项目最重要的设计参考和数值参考。它是一个成熟的 C++ 刚体动力学库，实现了高效的多体系统
算法和解析导数，建立在 Roy Featherstone 的经典刚体动力学算法基础上，并广泛用于机器人研究与工业应用。

Kinetix 当前借鉴了 Pinocchio 的若干关键设计：

- 空间运动向量、空间力向量和空间惯量；
- RNEA 逆动力学；
- CRBA 广义惯性矩阵；
- ABA 前向动力学；
- 不可变 `Model` 与可复用 `Data` 的分离；
- 从 URDF 离线构建模型。

仓库中包含本地参考源码 `ref/pinocchio`。`benchmarks` 目录里的工具可以直接使用这份本地 Pinocchio，
把 Kinetix 的输出与 Pinocchio 的输出逐项比较。需要强调的是，Kinetix 目前不是 Pinocchio 的替代品；
它是一个 Rust 优先、功能范围更窄、强调类型安全和实时友好性的探索项目。

## 性能快照

最近一次本地性能快照生成于 2026-06-30，使用命令：

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Iterations 50000 -Warmup 5000 -NoInit
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef -Iterations 50000 -Warmup 5000 -NoInit
python .\benchmarks\scripts\compare.py `
  .\benchmarks\results\kinetix.csv `
  .\benchmarks\results\pinocchio.csv
```

对比使用本地 `ref/pinocchio` 源码。在解读性能之前，需要先通过 correctness gate。下表单位是
ns/iter；`speedup = pinocchio_ns / kinetix_ns`，因此大于 `1.0` 表示 Kinetix 更快。

| 工作负载 | Kinetix 与 Pinocchio 的关系 |
| --- | --- |
| 空间代数微基准 | Kinetix 快 `1.47x` 到 `2.30x` |
| 串联链 RNEA | Kinetix 快 `1.11x` 到 `1.23x` |
| 串联链 ABA | Kinetix 快 `1.30x` 到 `1.62x` |
| 串联链 CRBA | Pinocchio 更快；Kinetix 为 `0.54x` 到 `0.61x` |

部分原始数据如下：

| 算法 | 自由度 | Kinetix ns/iter | Pinocchio ns/iter | Speedup |
| --- | ---: | ---: | ---: | ---: |
| RNEA | 2 | `156.114` | `190.772` | `1.222x` |
| RNEA | 12 | `1062.394` | `1174.760` | `1.106x` |
| RNEA | 30 | `2544.262` | `3008.250` | `1.182x` |
| ABA | 2 | `268.400` | `435.544` | `1.623x` |
| ABA | 12 | `2222.222` | `3030.868` | `1.364x` |
| ABA | 30 | `6025.446` | `7828.530` | `1.299x` |
| CRBA | 2 | `165.264` | `97.306` | `0.589x` |
| CRBA | 12 | `3056.106` | `1644.726` | `0.538x` |
| CRBA | 30 | `15448.452` | `9464.182` | `0.613x` |

这是一份实现阶段快照，不是对所有机器人、所有机器的普遍性能宣称。它也很清楚地指出了下一步优化目标：
CRBA 仍然在通用空间惯量和力变换上花了太多时间。

## 工作区结构

```text
kinetix/
|-- kinetix/             # 总入口 crate 和例程
|-- kinetix-spatial/     # no_std，坐标系类型安全的空间代数
|-- kinetix-core/        # no_std，RNEA、CRBA、ABA、Model/Data
|-- kinetix-urdf/        # std，URDF 解析和拓扑展平
|-- kinetix-autodiff/    # no_std，前向自动微分辅助
|-- kinetix-py/          # 未来 Python 绑定的占位模块
|-- benchmarks/          # Kinetix 与本地 Pinocchio 的正确性/性能对比
`-- ref/pinocchio/       # Pinocchio 本地参考源码
```

顶层 `kinetix` crate 会重新导出各子库，推荐例程和应用先使用：

```rust
use kinetix::prelude::*;
```

如果需要更底层的控制，也可以直接依赖各个子 crate。

## 快速开始

测试整个工作区：

```powershell
cargo test --workspace --offline
```

运行第一个逆动力学例程：

```powershell
cargo run -p kinetix --example 02_inverse_dynamics --offline
```

如果要观察接近实时控制场景的 Data 复用模式，建议使用 release：

```powershell
cargo run -p kinetix --example 07_realtime_data_reuse --offline --release
```

## 教学例程

例程位于 `kinetix/examples`，按照学习顺序组织：

| 例程 | 主题 | 学习内容 |
| --- | --- | --- |
| `01_spatial_algebra` | 空间代数 | 角速度在前的 6D 运动/力向量、叉乘、坐标变换 |
| `02_inverse_dynamics` | RNEA | 根据 `q`、`q_dot`、`q_ddot` 计算关节力矩 |
| `03_forward_dynamics_roundtrip` | RNEA + ABA | 用 RNEA 算出的力矩喂给 ABA，恢复目标加速度 |
| `04_mass_matrix` | CRBA | 构造广义惯性矩阵 `M(q)` |
| `05_urdf_loading` | URDF | 从内存中的双摆 URDF 字符串加载模型 |
| `06_rnea_derivatives` | 自动微分 | 用同一套 RNEA 计算 `d tau / d q` 和 `d tau / d q_dot` |
| `07_realtime_data_reuse` | 实时模式 | 只创建一次 `Data`，在控制循环中反复复用 |

从工作区根目录运行：

```powershell
cargo run -p kinetix --example 01_spatial_algebra --offline
cargo run -p kinetix --example 02_inverse_dynamics --offline
cargo run -p kinetix --example 03_forward_dynamics_roundtrip --offline
cargo run -p kinetix --example 04_mass_matrix --offline
cargo run -p kinetix --example 05_urdf_loading --offline
cargo run -p kinetix --example 06_rnea_derivatives --offline
cargo run -p kinetix --example 07_realtime_data_reuse --offline --release
```

Pinocchio 中还有可视化、碰撞检测、接触动力学、逆运动学、模型裁剪等丰富例程。Kinetix 目前还没有对应模块，
所以 README 里没有强行写这些例程；等相关模块实现后再补会更诚实。

## 只读 README 也能看的小例程

`kinetix/examples` 中的例程是可直接运行的完整版本。下面这些片段更适合只阅读 README 的读者，用来快速理解
Kinetix 的 API 形状。

### 带坐标系标签的空间代数

```rust
use kinetix::prelude::*;

let motion_w = SpatialMotion::<WorldFrame>::from_vector(Vector6::new(
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
));

let x_w_l1 = SpatialTransform::<WorldFrame, LinkFrame<1>>::new(
    Matrix3::identity(),
    Vector3::new(0.4, -0.2, 0.7),
);

let motion_l1 = x_w_l1.apply_motion(&motion_w);
println!("motion in link frame = {}", motion_l1.to_vector().transpose());
```

坐标系是类型的一部分。`SpatialMotion<WorldFrame>` 不能和
`SpatialMotion<LinkFrame<1>>` 被误加在一起。

### 一个最小逆动力学调用

```rust
use kinetix::prelude::*;

fn rigid_inertia(mass: f64, com: Vector3) -> SpatialInertia<WorldFrame> {
    SpatialInertia::from_mass_com_inertia(mass, com, Matrix3::identity() * 0.01)
}

let model = Model::<2>::new(
    [0, 0],
    [
        SpatialTransform::identity(),
        SpatialTransform::new(Matrix3::identity(), Vector3::new(1.0, 0.0, 0.0)),
    ],
    [
        rigid_inertia(1.7, Vector3::new(0.45, 0.0, 0.0)),
        rigid_inertia(1.1, Vector3::new(0.35, 0.0, 0.0)),
    ],
    [JointType::RevoluteZ, JointType::RevoluteZ],
    Vector3::new(0.0, -9.81, 0.0),
);

let mut data = Data::<2>::default();
data.q = JointVector::<2>::new(0.3, -0.8);
data.q_dot = JointVector::<2>::new(0.7, -0.2);
data.q_ddot = JointVector::<2>::new(1.1, -0.4);

rnea(&model, &mut data, None);
println!("tau = {}", data.tau.transpose());
```

### 在控制循环中复用 `Data`

```rust
use kinetix::prelude::*;

// 你自己的初始化阶段模型构建函数，或 URDF 加载函数。
let model: Model<2> = build_model_once();
let mut data = Data::<2>::default();

loop {
    // 从传感器或控制器更新 q、q_dot、q_ddot 或 tau。
    rnea(&model, &mut data, None);

    // 将 data.tau 发送到执行器层。
    break; // README 片段中只用于避免无限循环
}
```

`Model` 是不可变结构，可以共享；`Data` 是算法工作区，由单个实时线程独占并反复复用。

### 初始化阶段加载 URDF

```rust
use kinetix::prelude::*;

let urdf = r#"
<robot name="one_joint">
  <link name="base"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
</robot>
"#;

let model = load_from_str::<1>(urdf)?;
```

### 使用同一套 RNEA 计算导数

```rust
use kinetix::prelude::*;

// 你自己的初始化阶段模型构建函数，或 URDF 加载函数。
let model: Model<2> = build_model_once();
let q = JointVector::<2>::new(0.31, -0.43);
let q_dot = JointVector::<2>::new(0.37, -0.22);
let q_ddot = JointVector::<2>::new(0.91, -0.34);

let (dtau_dq, dtau_dq_dot) = compute_rnea_derivatives(&model, &q, &q_dot, &q_ddot);
println!("d tau / d q =\n{dtau_dq}");
println!("d tau / d q_dot =\n{dtau_dq_dot}");
```

## 核心设计

### 空间代数

`kinetix-spatial` 提供：

- `SpatialMotion<F, T>`：空间运动向量 `[omega; v]`；
- `SpatialForce<F, T>`：空间力向量 `[torque; force]`；
- `SpatialTransform<From, To, T>`：空间变换；
- `SpatialInertia<F, T>`：空间刚体惯量；
- `WorldFrame`、`LinkFrame<ID>` 等坐标系标签。

这里的 `F` 是编译期坐标系信息。比如，`WorldFrame` 中表达的运动量不能直接与 `LinkFrame<1>` 中表达的运动量相加。

### 动力学核心

`kinetix-core` 提供：

- `Model<DOF, T = f64>`：不可变机器人拓扑、相邻变换、惯量、关节类型、重力；
- `Data<DOF, T = f64>`：可复用算法工作区；
- `rnea`：逆动力学；
- `crba`：广义惯性矩阵；
- `aba`：前向动力学。

核心算法热路径不进行堆分配。`Model` 是只读共享结构，`Data` 由执行算法的线程独占并重复使用。

### URDF 加载

`kinetix-urdf` 允许使用 `std`。XML 解析、字符串处理、图遍历都发生在初始化阶段，不进入高频控制回路。
它可以从文件或字符串读取 URDF，对活动关节拓扑排序，并展平成 `Model<DOF>`。

### 自动微分

`kinetix-autodiff` 提供一阶双数和导数提取接口。核心思想是：不为导数重写一套 RNEA，而是把标量从 `f64`
换成 `Dual`，让同一套泛型动力学代码传播切向量。

## 与 Pinocchio 的正确性对齐

`benchmarks` 中包含一个行为一致性门禁：分别导出 Kinetix 和 Pinocchio 在同一组确定性输入下的结果，
然后逐元素比较。

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Correctness
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef -Correctness

python .\benchmarks\scripts\compare_correctness.py `
  .\benchmarks\results\kinetix_correctness.csv `
  .\benchmarks\results\pinocchio_correctness.csv `
  1e-9
```

当前门禁覆盖空间代数，以及 `1, 2, 6, 7, 12, 18, 30` 自由度串联旋转链上的 `rnea`、`crba`、`aba`。

## 性能对比

通过正确性门禁后，可以运行性能对比：

```powershell
.\benchmarks\scripts\run_kinetix.ps1 -Iterations 50000 -Warmup 5000 -NoInit
.\benchmarks\scripts\run_pinocchio.ps1 -UseLocalRef -Iterations 50000 -Warmup 5000 -NoInit

python .\benchmarks\scripts\compare.py `
  .\benchmarks\results\kinetix.csv `
  .\benchmarks\results\pinocchio.csv
```

解释结果时要注意边界：Kinetix 当前是静态尺寸、关节类型较少的实现；Pinocchio 是功能完整得多的通用运行时模型库。
因此性能对比主要用于观察实时热路径潜力，而不是完整库能力对比。

## 当前限制

- 目前只实现了 `RevoluteZ`、`PrismaticZ` 和 `Fixed`。
- 还没有几何、碰撞、接触动力学、闭链约束、逆运动学模块。
- URDF 支持范围较小，只覆盖当前动力学核心支持的关节和惯量模式。
- 项目仍处在实验阶段，API 尚不稳定。

## 路线图

- 扩展关节模型集合。
- 将热路径中仍存在的通用 6x6 运算替换为专用 3D 展开公式。
- 增强 URDF 覆盖面和模型诊断能力。
- 添加运动学与 Jacobian API。
- 增加几何/碰撞模块。
- 探索固定机器人专用代码生成内核。
- 在 Rust API 稳定后推进 Python 绑定。

## 引用

如果你在学术工作中使用 Kinetix，请引用本软件仓库：

```bibtex
@software{kinetix2026,
   author = {{Robot-Exp Platform contributors}},
   title = {Kinetix: A Rust-first rigid-body dynamics library with type-driven frame safety},
   url = {https://github.com/Robot-Exp-Platform/kinetix},
   year = {2026},
   note = {Experimental software}
}
```

Kinetix 深受 Pinocchio 启发，并在 benchmark 中将 Pinocchio 作为数值参考。如果你的工作依赖这种对齐关系或
Pinocchio 的设计传承，也请同时引用 Pinocchio。

To cite **Pinocchio** in your academic research, please consider citing the
[software paper](https://laas.hal.science/hal-01866228v2/file/19-sii-pinocchio.pdf)
and use the following BibTeX entry:

```bibtex
@inproceedings{carpentier2019pinocchio,
   title={The Pinocchio C++ library -- A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives},
   author={Carpentier, Justin and Saurel, Guilhem and Buondonno, Gabriele and Mirabel, Joseph and Lamiraux, Florent and Stasse, Olivier and Mansard, Nicolas},
   booktitle={IEEE International Symposium on System Integrations (SII)},
   year={2019}
}
```

And the following one for the link to the GitHub codebase:

```bibtex
@misc{pinocchioweb,
   author = {Justin Carpentier and Florian Valenza and Nicolas Mansard and others},
   title = {Pinocchio: fast forward and inverse dynamics for poly-articulated systems},
   howpublished = {https://stack-of-tasks.github.io/pinocchio},
   year = {2015--2021}
}
```

## 致谢

Kinetix 的存在建立在 Pinocchio 已经证明的一件事情上：一个精心设计的刚体动力学库可以非常强大。
本项目把 Pinocchio 作为概念参考、数值标尺和性能对照，同时尝试走一条 Rust 原生的路线：
用类型系统承载坐标系语义，用固定尺寸数据结构服务实时控制，用泛型标量支持自动微分。
