## 第一期
第一页


![[Pasted image 20261027104356.jpg]]
##### 1. 速度更新公式


在第 $t+1$ 次迭代中，第 $i$ 个粒子在第 $d$ 维的速度更新为：

$$v_{id}^{t+1} = \omega v_{id}^t + c_1 r_1 \left(p_{id}^t - x_{id}^t\right) + c_2 r_2 \left(p_{gd}^t - x_{id}^t\right)$$公式拆解由三部分组成：

- **惯性项 $\omega v_{id}^t$**：粒子保持先前飞行速度的惯性，反映对当前运动状态的信任。$\omega$ 为**惯性权重**，较大有利于全局探索（跳出局部极值），较小有利于局部精确开发。
    
- **认知项（自我认知）$c_1 r_1 (p_{id}^t - x_{id}^t)$**：拉动粒子向自身历史最优位置靠拢。$c_1$ 为个体学习因子。
    
- **社会项（群体协作）$c_2 r_2 (p_{gd}^t - x_{id}^t)$**：拉动粒子向全群历史最优位置靠拢。$c_2$ 为社会学习因子。
    
- $r_1, r_2$：在 $[0, 1]$ 之间均匀分布的独立随机数，为搜索提供随机扰动。
### 四、传统 PSO 的优缺点与改进动机

|**特性**|**表现**|**说明**|
|---|---|---|
|**优点**|原理简单直观|无需目标函数的求导与连续性信息，仅依赖适应度评价。|
||参数少、易实现|核心代码仅需几十行，易于嵌入工程优化（如机械臂逆解、参数辨识）。|
||收敛速度快|在单峰凸优化问题上能够迅速逼近极值区。|
|**主要缺陷**|**早熟收敛（易陷入局部最优）**|后期所有粒子迅速向 $P_g$ 聚集，群体多样性急剧丧失；若 $P_g$ 落在局部极小值，算法将无法跳出。|
||**后期搜索精度不足**|随着速度衰减，粒子在最优解附近的微调能力减弱。|
第二页

量子行为粒子群优化算法（Quantum-behaved Particle Swarm Optimization, QPSO）顶部标题

备注：
根据经典力学原理，粒子在每一时刻的轨迹都是确定且连续的，这导致其在搜索空间中的探索能力完全受限于速度更新步长；一旦种群粒子向全局最优聚集且速度衰减至接近零，粒子便无法脱离当前引力势阱，极易陷入局部最优（早熟收敛）。为了克服这一缺陷，Sun 等人于 2004 年提出了**量子行为粒子群优化算法（Quantum-behaved Particle Swarm Optimization, QPSO）**。



### 位置更新公式本质上由三个核心动作拼接而成：也在顶部，标题下面

$$x_{id}^{t+1} = \underbrace{P_{id}^t}_{\text{① 基准锚点}} \quad \underbrace{\pm}_{\text{② 掷硬币定方向}} \quad \underbrace{\alpha \cdot \vert{}C_d^t - x_{id}^t\vert{} \cdot \ln(1/u)}_{\text{③ 随机步长生成器}}$$

#### 1. 局部吸引中心 $P_{i}$    在下半左侧部分

每个粒子受自身历史最优 $pbest_i$ 和全局最优 $gbest$ 的共同牵引：

$$P_{id}^t = \phi_{d}^t \cdot pbest_{id}^t + (1 - \phi_{d}^t) \cdot gbest_{d}^t, \quad \phi_{d}^t \sim \mathcal{U}(0, 1)$$

其中 $P_{id}^t$ 是粒子 $i$ 在第 $d$ 维的引力势阱中心，
$pbest$ 为个体最优，
$gbest$ 为全局最优，
$\phi$ 为均匀分布的随机数。

#### ② 节流阀：$\alpha$（人为设定的步长阻尼）

- 它就像一个油门旋钮。
    
- $\alpha$ 设得大，步长整体放大，鼓励去探索远方；$\alpha$ 设得小，步长压缩，让粒子安分地在 $P_{id}^t$ 附近搜索。


右侧图片
![[Pasted image 20261027111908.png]]

第三页
顶部放置标题
==🔴**概率密度函数==（Probability Density Function, PDF）**

$$x_{id}^{t+1} = \underbrace{P_{id}^t}_{\text{① 基准锚点}} \quad \underbrace{\pm}_{\text{② 掷硬币定方向}} \quad \underbrace{\alpha \cdot \vert{}C_d^t - x_{id}^t\vert{} \cdot \ln(1/u)}_{\text{③ 随机步长生成器}}$$
下半右侧写优势
####  1. 消除传统速度更新的“边界截断”与“搜索盲区”
它在实数轴的任意位置都严格保持非零概率，且尾部比高斯分布“厚”得多。这意味着无论当前粒子群聚集得多紧密，在统计学上它都保留了直接“瞬间跃迁”到任意远处的数学可能（模拟量子隧道效应），从理论上赋予了算法真正的**全局遍历性（Ergodicity）**
#### 2. 可以导出闭式解析解，避免昂贵的数值计算
如果使用很多其他重尾分布计算机没有初等解析反函数，必须通过查表法或多次级数展开逼近（如 Box-Muller 算法），在高维、多代数的粒子群迭代中会显著增加计算耗时。

下半左侧使用这个图片

![[Pasted image 20261027120441.jpg|657]]

从中心 $P$ 向右做累积积分
$$\text{累积概率 } u = \int \vert{}\psi(y)\vert{}^2 dy = 1 - e^{-\frac{2(x - P)}{L}}$$
把均匀随机数简化代换后，便得到了：

$$\Delta x = \frac{L}{2} \ln\left(\frac{1}{u}\right)$$

第四页

上方用左中右三个方程块中间加上箭头和变化解释下面三部分方程
### 角速度线速度的雅可比矩阵解析
$$\boldsymbol{\omega}_e^{(i)} = \mathbf{z}_{i-1} \dot{\theta}_i$$
$$\mathbf{v}_e^{(i)} = \boldsymbol{\omega}_e^{(i)} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) = \left( \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \right) \dot{\theta}_i$$
通过将每个关节的线速度和角速度贡献按列拼装：
$$\mathbf{V}_e = \begin{bmatrix} \mathbf{v}_e \\ \boldsymbol{\omega}_e \end{bmatrix} = \mathbf{J}(\boldsymbol{\theta}) \dot{\boldsymbol{\theta}} = \begin{bmatrix} \mathbf{J}_{v1} & \mathbf{J}_{v2} & \cdots & \mathbf{J}_{vn} \\ \mathbf{J}_{\omega 1} & \mathbf{J}_{\omega 2} & \cdots & \mathbf{J}_{\omega n} \end{bmatrix} \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_2 \\ \vdots \\ \dot{\theta}_n \end{bmatrix}$$
得到从关节空间（Joint Space）”与“笛卡尔操作空间（Cartesian Operational Space）”的变换的雅可比矩阵
$$\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{vi} \\ \mathbf{J}_{\omega i} \end{bmatrix} = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}$$

下半左半部分放
#### 直观几何意义

若让其他所有关节锁死不动（$\dot{\theta}_j = 0, \forall j \neq i$），仅让第 $i$ 个关节以单位速度转动（$\dot{\theta}_i = 1$），此时末端执行器产生的空间速度为：

$$\mathbf{V}_e = \mathbf{J} \begin{bmatrix} 0 \\ \vdots \\ 1 \\ \vdots \\ 0 \end{bmatrix} = \mathbf{J}_i = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}$$

**雅可比矩阵的第 $i$ 列，就是“当只有第 $i$ 个关节以 $1\text{ rad/s}$ 旋转时，末端执行器所产生的纯速度响应矢量”**。每一列代表一个关节的基底贡献，拼在一起就构成了整个多自由度系统的总速度变换矩阵。

下半右半部分放
#### 整机的“空间奇异性”（Kinematic Singularity）

如果单独看线速度部分 $\boldsymbol{J}_v$（$3 \times n$）或角速度部分 $\boldsymbol{J}_\omega$（$3 \times n$），它们各自可能都是满秩的；但一旦按列拼装成 $6 \times n$ 的整体雅可比矩阵 $\boldsymbol{J}$：

- **行列式与秩的变化带来新的物理状态**：当且仅当把两者拼在一起时，$\det(\boldsymbol{J}\boldsymbol{J}^T) = 0$ 才代表**整机在 6 自由度任务空间中丧失了某个方向的运动或姿态调整能力**。
    
- 例如：机械臂完全伸直时，仅看某些角度的旋转能力并未丧失，但整体在径向平移与俯仰角速度之间产生了强耦合退化，导致自由度丢失。这种系统级的退化特性，单看局部矩阵是无法判定的。

#### 完整描述刚体在空间中的瞬时位姿变化率。
- 单看角速度贡献 $\boldsymbol{J}_\omega$，完全不知道末端在空间中的移动快慢；
    
- 单看线速度贡献 $\boldsymbol{J}_v$，它其实由基座沿途所有关节的**转动诱导线速度**叠加而来，其数值直接受各关节角速度方向的制约。
第五页

### 经典灵活性指标（Classic Flexibility Indicators）
上三分之一写
#### **单位超球面（Unit Hypersphere）

    $$\Vert{}\dot{\theta}\Vert{}^2 = \dot{\theta}_1^2 + \dot{\theta}_2^2 + \cdots + \dot{\theta}_n^2 \le 1$$
    
在多维空间中，满足平方和 $\le 1$ 的几何体在定义上就是一个**单位超球面（Unit Hypersphere）**。**物理意义**：这代表给机械臂提供一个“在关节空间内朝任何方向都完全等长、不偏不倚的单位测试信号集”。

给一个向左下箭头   箭头上写   经过雅可比行列式的映射 $$\dot{x} = J\dot{\theta}$$
箭头下面写
**速度操作度椭球（Manipulability Ellipsoid）**：

$$\dot{x}^T (J(\theta)J^T(\theta))^{-1} \dot{x} \le 1$$
给一个向右箭头 指向操作度指标d

操作度指标 $M(\theta)$       Yoshikawa（吉川恒夫）的经典定义
$$M(\theta) = \sqrt{\det\left(J(\theta)J^T(\theta)\right)}$$

公式右侧写一下几点
$M(\theta)$ 正比于操作度椭球的==🔴体积==。它综合度量了机械臂末端在全方位（所有任务自由度）的速度传递能力，灵活性。如果越扁，越接近奇异点
 - **机械臂几何极限（手臂完全伸直或完全折叠）**
 - **关节轴线共线或平行（自由度冗余退化）**

再在这部分下面放一个向下箭头 箭头上写 $M(\theta) = \sqrt{\det(JJ^T)}$ 刻画的是这个==🔴**椭球的体积**==，无法观测**某一个特定切线方向**的运动性能

引出
#### 方向可操作度（Directional Manipulability, DM）
占下半的右边
左部分写这个
$$\dot{x} = \alpha v \quad (\alpha > 0)$$
下方标注
V关节方向
X实际运动速度
向右箭头，肩头上写$\dot{x}^T (JJ^T)^{-1} \dot{x} = 1$
箭头右边 $$\alpha^2 = \frac{1}{v (JJ^T)^{-1} v^T}$$
标注   最大速度模长平方 $\alpha^2$

下半的左边
#### 3. 各向同性指标（Isotropic Index）

经典方法常用雅可比矩阵的条件数（Condition Number）来评估机构的各向同性，但条件数需要计算矩阵奇异值分解（SVD），在多自由度在线优化中计算开销极大。文中引入基于矩阵代数特性的各向同性指标 $\Delta$（式 3）：

$$\Delta = \frac{M}{\psi}$$



$$M = \sqrt[n]{\det(JJ^T)} = \sqrt[n]{\lambda_1 \lambda_2 \dots \lambda_n}$$



$$\psi = \frac{\mathrm{trace}(JJ^T)}{n} = \frac{\lambda_1 + \lambda_2 + \dots + \lambda_n}{n}$$
分子 $M$ 是特征值的几何平均数（Geometric Mean）的平方根尺度；
        
    - 分母 $\psi$ 是特征值的**算术平均数（Arithmetic Mean）**；
        
    - 根据几何-算术均值不等式，恒有 $0 \le \Delta \le 1$。当 $\Delta \to 1$ 时，特征值趋向于完全相等，操作度椭球退化为超球面，表明末端在各个方向上的运动灵活性完全一致（完全各向同性）；当 $\Delta \to 0$ 时，表明矩阵接近奇异，机械臂处于或接近奇异位形。
        
- **核心优势**：计算仅依赖对称正定矩阵 $JJ^T$ 的**行列式（$\det$）**与**迹（$\mathrm{trace}$）**，避免了 SVD 奇异值分解，大幅节省了计算资源和时间。
## 第二期
#### 第一页
DMII-IQPSO 逆解优化算法
 - **基于各向同性指标约束方向可操纵性的改进量子粒子群优化算法
 - DMII：Directional Maneuverability constrained by Isotropic Indexes（各向同性指标约束方向可操纵性，用于冗余机械臂避奇异）
 - IQPSO：Improved Quantum Particle Swarm Optimization（改进量子粒子群优化）
#### 第二页
### DMII。第二页标题
第二页上方
**IV-A 带物理罚函数的逆解优化模型**：
- 作者提出该指标的根本目的，是为了**同时兼顾机械臂在任务指定方向上的运动能力，并彻底杜绝机械臂陷入奇异构型（Singular Configuration）**，同时还要避免传统方法计算雅可比矩阵条件数（SVD 分解）的高额算力开销
==🔴DMII 的数学表达式（式 6）定义如下：==
$$V_{DMII}(\theta) = \underbrace{\left(1 - \xi^{(\Delta - \Delta_{\max})}\right)}_{\text{各向同性/防奇异项}} \times \underbrace{\xi^{V_{DM}(\theta)}}_{\text{指定方向速度增益项}}$$
第二页下半左边写参数解释


- **$\theta = [\theta_1, \theta_2, \dots, \theta_7]$**：机械臂当前的 7 个关节角构型。
    
- **$V_{DM}(\theta)$：[[#2.方向可操作度（Directional Manipulability, DM）]]
    
     - 其中 $v$ 是任务给定的末端期望运动方向单位向量（例如末端插补直线方向）。该值越大，说明机械臂沿着该方向的运动响应越灵敏。
    
- **$\Delta$**：[[#3. 各向同性指标（Isotropic Index）]]
    
    -  $\Delta \in [0, 1]$ 度量了末端全方位运动的均匀程度。$\Delta \to 1$ 表示各向同性极佳（接近正球体，远离奇异）；$\Delta \to 0$ 表示接近奇异位形。
    -  **$\Delta_{\max}$**：任务所设定的最大期望各向同性指标阈值。
    
- **$\xi$**：调节底数常数（Constant）。
第二页下半右边写函数相应机理
- **防奇异机制（软屏障 / 门控作用）**：
    
    - 当机械臂构型逼近奇异位形时，$\Delta \to 0$。
        
    - 在选定合适的基底常数 $\xi$ 条件下，前项 $\left(1 - \xi^{(\Delta /\Delta_{\max})}\right)$ 会迅速逼近 0，导致整体指标 **$V_{DMII}(\theta) \to 0$**。
        
- **方向灵活性增益**：
    
    - 在构型远离奇异（$\Delta$ 处于安全范围）的前提下，末端沿目标方向的操作度 $V_{DM}(\theta)$ 越大，后项所提供的增益就越高，从而拉高整体指标 $V_{DMII}$。
        
- **二者权衡**：
    
    - 只有当机械臂既保证非奇异各向同性（安全），又在目标方向具备充足速度储备（高效）时，$V_{DMII}(\theta)$ 才能取得较大值。
第三页
Improved Quantum Particle Swarm Optimization（改进量子粒子群优化）。标题
分三个部分
第一部分
#### 适应度（Fitness）
- 是评价一个解有多优秀的数学打分指标**
    
    $$\text{适应度} = \underbrace{(p_x - p_x')^2 + (p_y - p_y')^2 + (p_z - p_z')^2}_{\text{末端到达目标点的空间欧氏距离平方}} + \underbrace{\varphi(\theta)}_{\text{灵巧度避奇异惩罚项}}$$
    - 将末端三维欧氏距离误差作为基础适应度函数
    - 将 DMII 的倒数 $1 / V_{DMII}(\theta)$ 构建为不等式惩罚项 $\varphi(\theta)$，在数学上迫使逆解收敛到高灵巧度、非奇异构型。
    - 目标是求**极小化**（Minimization Problem）
    - 适应度数值越小，说明机械臂末端离目标点越近，且姿态越远离奇异区，该解越“适宜”。
#### 惯性权重（Inertia Weight, $\omega$）
在标准 PSO 速度更新公式中：

$$v^{t+1} = \mathbf{\omega} v^t + \dots$$

- **物理定义**：$\omega$ 描述了粒子**保留上一时刻运动惯性**的程度。
    
- **作用机制**：
    
    - **当 $\omega$ 较大时（如 0.9）**：粒子倾向于按原来的速度惯性往前冲，较少被历史最优拉扯，具有极强的**全局探索能力（Exploration）**，能快速在全工作空间大跨步扫视，不容易陷入局部陷阱。
        
    - **当 $\omega$ 较小时（如 0.4）**：粒子惯性小，受自身和群体历史最优的引力主导，专注于在当前区域慢慢盘旋，具有极强的**局部开发精细搜索能力（Exploitation）**。
第二部分
**双轨更新与动态惯性权重 $\omega_i^t$**
- **改进动机**：在变异更新分支中，固定惯性权重无法顾及群体内不同适应度个体的差异。
    
- **具体设计**：定义动态自适应惯性权重系数：
    
    $$\omega_i^t = \begin{cases} \omega_{\min} + (\omega_{\max} - \omega_{\min}) \frac{f(x_i^t) - f_{\min}^t}{f_{\mathrm{avg}}^t - f_{\min}^t}, & f(x_i^t) \le f_{\mathrm{avg}}^t \\ \omega_{\max}, & f(x_i^t) > f_{\mathrm{avg}}^t \end{cases}$$
    
    其中设 $\omega_{\min} = 0.4$，$\omega_{\max} = 0.9$，$f_{\mathrm{avg}}^t$ 与 $f_{\min}^t$ 分别为第 $t$ 代群体的平均适应度与最小适应度。
    
- **改进效果**：对于表现差（$f(x_i^t) > f_{\mathrm{avg}}^t$）的粒子，赋予最大的惯性权重 $\omega_{\max} = 0.9$，迫使其以大步长飞离差解区域；对于表现优于平均水平的粒子，根据其优势程度自适应缩小权重，引导其在当前区域进行精细微调。

第四页
上方给出这个曲线和这个示意图
![[Attachments/Pasted image 20261102141933.png|319]]
#### 三段切线分段 Logistic 混沌映射 
$$\sigma_{k+1} = \begin{cases} (3+2\sqrt{2})\mu\sigma_k(\sqrt{2}-1-\sigma_k), & 0 \le \sigma_k \le \sqrt{2}-1 \\  1 - (3+2\sqrt{2})(\sigma_k - 1 + \frac{\sqrt{2}}{2})\mu(\frac{3\sqrt{2}}{2} - 2 - \sigma_k), & \sqrt{2}-1 < \sigma_k \le 2-\sqrt{2} \\  (3+2\sqrt{2})\mu(\sigma_k - 2 + \sqrt{2})(\sigma_k - 1), & 2-\sqrt{2} \le \sigma_k \le 1 \end{cases}$$
下方右半
经典的 Logistic 映射公式是单峰抛物线：$$\sigma_{k+1} = \mu \sigma_k (1 - \sigma_k)$$**拓宽完全混沌的参数范围**：
	2. **多峰增强遍历均匀性**：
	3. **更强的初值敏感性**：
$$f^{(k)}(\sigma) = \underbrace{f(f(\dots f(\sigma)\dots))}_{k \text{ 次}}$$
#### 遍历性（Ergodicity）
只要时间足够长，混沌变量生成的轨迹点能够无遗漏地访问整个目标空间中的每一个角落，不会形成盲区。   
#### 非重复性（Non-periodicity）
它虽然由确定的确定性数学公式生成，但生成的序列永远不呈周期性循环，永不重复。

下方左半
#### 参数 $\mu$ 的作用：调节系统的非线性强度
		1. 当 $\mu = 4$ 时（完全混沌态 / Fully Developed Chaos）:系统的动力学特性达到物理极限与拓扑遍历的最大状态。
  - #### 为什么引入 $(3 + 2\sqrt{2})$ 这个特定系数？
	 - 因为在区间一中，抛物线的跨度从常规的 $1$ 缩减到了 $L = \sqrt{2}-1$。 抛物线 $x(L - x)$ 的最大值为：
$$\left(\frac{L}{2}\right)^2 = \frac{(\sqrt{2}-1)^2}{4} = \frac{3-2\sqrt{2}}{4}$$
	 - 为了在 $\mu=4$ 时让极值刚好充满整个 $[0, 1]$ 空间（即最大值归一化到 $1$），必须在前面乘上其倒数：$$\frac{4}{3 - 2\sqrt{2}} = 4(3 + 2\sqrt{2})$$
	 - 当控制参数 $\mu=4$ 时，乘积项恰为 $(3+2\sqrt{2})\mu$，保证了映射输出值域严格被约束在 $[0, 1]$ 之间。
	下面是备注
	标准 Logistic 映射只有当参数 $\mu$ 精确维持在 $3.57 \sim 4.0$ 的极窄边缘时才呈现完全混沌，在此区间内还穿插着大量周期窗口（会导致生成伪随机序列死循环）。三段切线结构极大地拓宽了强混沌区间。
	单峰 Logistic 映射生成的数值序列呈现“两头密、中间疏”的 U 型分布，容易导致粒子密集扎堆在边界 $0$ 和 $1$ 附近。改成三峰/分段振荡结构后，解空间内部的概率密度分布更加平坦均匀，不会漏掉中间维度的可行解。
	分段后曲线在区间内的平均斜率绝对值远大于普通单峰抛物线，初值只要有一丁点扰动，相邻状态点被拉扯发散的速度成倍加快。
第五页
### 2. **自适应变异概率 $P_m^t$**上方标题和公式
 **一个随代数平滑退出的“PSO-QPSO 混合退火开关
$$P_m^t = c \cdot \left(1 - \frac{t}{t_{\max}}\right)^2$$
左半解释参数
- $t$：当前迭代代数。
    
- $t_{\max}$：预设的最大迭代代数（论文中设为 1000）。
    
- $c$：变异概率初值系数，论文中取固定常数 $c = 0.5$。
    
- $P_m^t$：第 $t$ 代时的变异阈值，初值处于 $0.5$，随着 $t$ 逼近 $t_{\max}$，该值以二次抛物线单调平滑衰减至 $0$。

右半解释策略
**在每一步迭代时，算法抽取一个均匀随机数 $\gamma \in [0, 1]$ 与 $P_m^t$ 进行条件分流：
==🟢分流策略==
- 分支 A：当 $\gamma > P_m^t$ 时，执行[[#二 、量子行为粒子群优化算法（Quantum-behaved Particle Swarm Optimization, QPSO）]]
- 分支 B：当 $\gamma \le P_m^t$ 时，执行[[#一 、粒子群优化算法（Particle Swarm Optimization, PSO）]]
==🟢原理解释==
- **前期（$t$ 较小，$P_m^t \approx 0.5$）——强行注入动能**： 纯粹的 QPSO 是没有速度概念的，容易被均值最优位置 $M_{\text{best}}$ 过早拉扯成一团。作者在前期以 $50\%$ 的极高概率强行切换到式 (17)，利用速度项 $v$ 和自适应惯性权重 $\omega_{\max}=0.9$，像搅拌机一样把粒子随机“甩”出去，强迫粒子群在 7 个关节的整个限位空间内大范围散开。
    
- **后期（$t \to t_{\max}$，$P_m^t \to 0$）——彻底剥离速度，交给量子吸引子**： 机械臂逆运动学具有极其严重的非线性多解性。如果后期继续保留 PSO 的速度更新，由于粒子速度有惯性（Inertia），粒子会很容易直接“冲过”狭窄的高精度误差漏斗。 作者希望在后期把速度项完全丢掉，让粒子进入 QPSO 机制，牢牢被吸引中心 $P_i$ 束缚住。
	