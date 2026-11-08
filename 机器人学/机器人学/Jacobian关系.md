# 机器人学基础：从微积分视角透视雅可比矩阵（Jacobian）

> **核心口诀**：
> 
> 机器人 Jacobian 并非全新数学概念，本质就是**“末端位姿对关节变量的一阶偏导矩阵”**。
> 
> 微小位移线性传递，除以时间即为速度映射；行列式判退化，降秩不可逆即为奇异。
> 
>   

## 1. 从多元微积分的 Jacobian 出发

在高等数学与微积分中，假定存在两个输入变量 $q_1, q_2$ 与两个输出变量 $x, y$ 的映射关系：

  

$$\begin{aligned} x &= f_1(q_1, q_2) \\ y &= f_2(q_1, q_2) \end{aligned}$$

其一阶偏导数构成的映射矩阵即为 [[雅可比矩阵]]（Jacobian Matrix）：

  

$$J = \begin{bmatrix} \frac{\partial x}{\partial q_1} & \frac{\partial x}{\partial q_2} \\ \frac{\partial y}{\partial q_1} & \frac{\partial y}{\partial q_2} \end{bmatrix}$$

- **直观几何物理意义**：当输入空间产生微小摄动 $\Delta q_1, \Delta q_2$ 时，输出空间 $\Delta x, \Delta y$ 产生的线性变化率。
    
      
    

## 2. 机器人学映射：以二维平面二连杆为例

在机器人学中，输入变量被赋予物理实体含义——**关节变量**（Joint Variables），输出变量对应**末端位姿**（End-Effector Pose）。

  

设平面二连杆机械臂连杆长度分别为 $L_1, L_2$，关节旋转角度分别为 $q_1, q_2$。

  

### 2.1 正运动学方程（Forward Kinematics, FK）

根据平面几何投影，末端空间位置为：

  

$$\begin{cases} x = L_1 \cos q_1 + L_2 \cos(q_1 + q_2) \\ y = L_1 \sin q_1 + L_2 \sin(q_1 + q_2) \end{cases}$$

这本质上即为高数中的非线性向量映射 $x = f(q)$。

  

### 2.2 求解雅可比矩阵

对末端坐标分别对各关节变量计算偏导数：

  

$$J = \begin{bmatrix} \frac{\partial x}{\partial q_1} & \frac{\partial x}{\partial q_2} \\ \frac{\partial y}{\partial q_1} & \frac{\partial y}{\partial q_2} \end{bmatrix} = \begin{bmatrix} -L_1 \sin q_1 - L_2 \sin(q_1 + q_2) & -L_2 \sin(q_1 + q_2) \\ L_1 \cos q_1 + L_2 \cos(q_1 + q_2) & L_2 \cos(q_1 + q_2) \end{bmatrix}$$

## 3. 雅可比的核心物理本质：微小变化传递与速度映射

微积分中的一阶全微分近似：

  

$$\mathrm{d}y = f'(x)\,\mathrm{d}x$$

推广至高维向量空间：

  

$$\begin{bmatrix} \mathrm{d}x \\ \mathrm{d}y \end{bmatrix} = J \begin{bmatrix} \mathrm{d}q_1 \\ \mathrm{d}q_2 \end{bmatrix} \implies \Delta x \approx J \Delta q$$

- $\Delta q = \begin{bmatrix} \Delta q_1 \\ \Delta q_2 \end{bmatrix}$：关节空间（Joint Space）的微小转角；
    
      
    
- $\Delta x = \begin{bmatrix} \Delta x \\ \Delta y \end{bmatrix}$：任务空间（Task/Cartesian Space）的微小位移。
    
      
    

对时间 $t$ 求导（即两端同除以 $\Delta t$ 并在 $\Delta t \to 0$ 取极限）：

  

$$\dot{x} = J(q)\,\dot{q}$$

- $\dot{q}$ 为关节角速度向量；
    
      
    
- $\dot{x}$ 为末端操作空间线速度向量；
    
      
    
- $J(q)$ 充当了**关节速度到末端速度的瞬态传递线性变换矩阵**。
    
      
    

## 4. 雅可比行列式与机械臂奇异点（Singularity）

> **关键概念区分**：
> 
>   
> 
> - **雅可比矩阵** $J$：线性映射矩阵；
>     
>       
>     
> - **雅可比行列式** $\det(J)$：标量值（仅在方阵时成立），用于衡量局部映射是否发生体积缩放至零或降维退化。
>     
>       
>     

### 4.1 二连杆行列式计算

针对二连杆的 $J$，利用三角恒等式展开计算行列式：

  

$$\begin{aligned} \det(J) &= \left[-L_1 \sin q_1 - L_2 \sin(q_1 + q_2)\right]\left[L_2 \cos(q_1 + q_2)\right] \\ &\quad - \left[-L_2 \sin(q_1 + q_2)\right]\left[L_1 \cos q_1 + L_2 \cos(q_1 + q_2)\right] \\ &= L_1 L_2 \left[\sin(q_1 + q_2)\cos q_1 - \cos(q_1 + q_2)\sin q_1\right] \\ &= L_1 L_2 \sin q_2 \end{aligned}$$

整理即得：

  

$$\det(J) = L_1 L_2 \sin q_2$$

### 4.2 奇异构型判定

当 $\det(J) = 0$ 时，矩阵退化不可逆，机械臂处于 [[运动学奇异点]]（Kinematic Singularity）：

  

$$\sin q_2 = 0 \implies q_2 = 0^\circ \quad \text{或} \quad q_2 = 180^\circ$$

- **几何现象**：
    
      
    - $q_2 = 0^\circ$ 时两杆完全伸展共线（边界奇异）；
        
          
        
    - 机械臂沿手臂延伸方向瞬时丧失移动自由度；
        
          
        
    - 即使关节以无限大速度旋转，末端也无法产生沿着该方向的线速度分量。
        
          
        

### 4.3 线性代数视角：列向量线性相关

将 $J$ 视作各列构成的向量组：

  

$$J = \begin{bmatrix} \vert & \vert \\ J_1 & J_2 \\ \vert & \vert \end{bmatrix}$$

- $J_1$ 代表仅旋转关节 1 时末端的速度基向量；
    
      
    
- $J_2$ 代表仅旋转关节 2 时末端的速度基向量；
    
      
    
- 当构型处于奇异点时，$J_1$ 与 $J_2$ 平行共线（线性相关），所张成的运动空间（Range Space）从 2 维塌缩为 1 维。
    
      
    

## 5. 雅可比矩阵在逆运动学（IK）中的应用

已知正运动学微分关系：

  

$$\dot{x} = J(q)\,\dot{q}$$

在 [[逆运动学]]（Inverse Kinematics, IK）中，控制目标是根据末端期望运动求解关节控制量：

  

### 5.1 正方满秩情况

若 $J$ 为方阵且 $\det(J) \neq 0$（非奇异），可通过直接求逆获得解析更新步长：

  

$$\dot{q} = J^{-1}\dot{x}$$

### 5.2 奇异点与冗余机械臂情况

- 若机械臂遭遇奇异点，$\det(J) = 0$，$J^{-1}$ 不存在；在奇异点邻域，直接求逆会导致关节速度趋近于无穷大 $\dot{q} \to \infty$。
    
      
    
- 当机械臂自由度大于任务空间自由度（例如 7 轴机械臂控制 6 维末端位姿）时，$J$ 为非方阵（长方矩阵），需引入 [[Moore–Penrose 伪逆]]（Pseudo-inverse）$J^+$：
    
      
    

$$\dot{q} = J^+\dot{x}$$

## 6. 知识关联总结对照

|**概念/符号**|**数学意义**|**机器人学物理意义**|
|---|---|---|
|**$x = f(q)$**|向量值函数映射|[[正运动学]]（关节角 $\to$ 末端位姿）|
|**$J = \frac{\partial x}{\partial q}$**|多元函数一阶导数矩阵|[[雅可比矩阵]]（空间瞬态线性变换比）|
|**$\Delta x \approx J \Delta q$**|全微分线性近似|微小关节移动 $\to$ 微小末端位移|
|**$\dot{x} = J \dot{q}$**|速度导数链式法则|关节速度向量 $\to$ 末端操作速度向量|
|**$\det(J)$**|映射体积缩放率|奇异性判别式（判断局部自由度是否退化）|
|**$\det(J) = 0$**|矩阵行列式为零（不可逆）|[[运动学奇异点]]（机械臂丧失特定方向移动能力）|
|**$J^{-1} \big/ J^+$**|逆矩阵 / 伪逆矩阵|[[逆运动学]] 速度控制解析求解|


## 8. 必须牢记的核心演进链

$$\text{正运动学 } x = f(q) \xrightarrow{\text{对 } q \text{ 求导}} J = \frac{\partial f}{\partial q} \xrightarrow{\text{全微分}} \mathrm{d}x = J\,\mathrm{d}q \xrightarrow{\div \mathrm{d}t} \dot{x} = J\dot{q} \xrightarrow{\text{反求关节速度}} \dot{q} = J^{-1}\dot{x}$$
#### 9. 雅可比矩阵求解逆运动学 $J(\theta)$

雅可比矩阵是由正运动学方程对各关节角求一阶偏导构成的矩阵：

$$J(\theta) = \begin{bmatrix} \frac{\partial p_x}{\partial \theta_1} & \frac{\partial p_x}{\partial \theta_2} \\ \frac{\partial p_y}{\partial \theta_1} & \frac{\partial p_y}{\partial \theta_2} \end{bmatrix} = \begin{bmatrix} -l_1 \sin\theta_1 - l_2 \sin(\theta_1 + \theta_2) & -l_2 \sin(\theta_1 + \theta_2) \\ l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2) & l_2 \cos(\theta_1 + \theta_2) \end{bmatrix}$$

#### 3. 数值迭代的具体计算步骤

假设参数：$l_1 = 1\text{ m}, l_2 = 1\text{ m}$。

目标位置：$x_{\text{target}} = [1.0, 1.0]^T$。

设定初始猜测值：$\theta^{(0)} = [0^\circ, 90^\circ]^T = [0, \frac{\pi}{2}]^T \text{ rad}$。

- **第 1 轮迭代（Iteration 1）**：
    
    1. **计算当前末端位置与误差**：
        
        $$p_x^{(0)} = 1\cdot\cos(0) + 1\cdot\cos(0 + \frac{\pi}{2}) = 1 + 0 = 1.0\text{ m}$$
        
        $$p_y^{(0)} = 1\cdot\sin(0) + 1\cdot\sin(0 + \frac{\pi}{2}) = 0 + 1 = 1.0\text{ m}$$
        
        _(注：若初值算出的位置恰好匹配，误差为 0；为了看迭代过程，我们将目标设定为 $x_{\text{target}} = [0.5, 1.2]^T$)_
        
        末端误差向量为：
        
        $$e^{(0)} = x_{\text{target}} - f(\theta^{(0)}) = \begin{bmatrix} 0.5 - 1.0 \\ 1.2 - 1.0 \end{bmatrix} = \begin{bmatrix} -0.5 \\ 0.2 \end{bmatrix}$$
        
    2. **带入当前角度计算雅可比矩阵数值**：
        
        $$J(\theta^{(0)}) = \begin{bmatrix} -\sin(0) - \sin(\frac{\pi}{2}) & -\sin(\frac{\pi}{2}) \\ \cos(0) + \cos(\frac{\pi}{2}) & \cos(\frac{\pi}{2}) \end{bmatrix} = \begin{bmatrix} -1 & -1 \\ 1 & 0 \end{bmatrix}$$
        
    3. **计算雅可比矩阵的逆 $J^{-1}$**：
        
        行列式 $\det(J) = (-1)(0) - (-1)(1) = 1$。
        
        $$J^{-1} = \frac{1}{\det(J)} \begin{bmatrix} 0 & 1 \\ -1 & -1 \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -1 & -1 \end{bmatrix}$$
        
    4. **反解关节角增量 $\Delta \theta$ 并更新**：
        
        $$\Delta \theta^{(0)} = J^{-1} e^{(0)} = \begin{bmatrix} 0 & 1 \\ -1 & -1 \end{bmatrix} \begin{bmatrix} -0.5 \\ 0.2 \end{bmatrix} = \begin{bmatrix} 0.2 \\ 0.5 - 0.2 \end{bmatrix} = \begin{bmatrix} 0.2 \\ 0.3 \end{bmatrix}\text{ rad}$$
        
        新关节角更新为：
        
        $$\theta^{(1)} = \theta^{(0)} + \Delta \theta^{(0)} = \begin{bmatrix} 0 \\ 1.5708 \end{bmatrix} + \begin{bmatrix} 0.2 \\ 0.3 \end{bmatrix} = \begin{bmatrix} 0.2 \\ 1.8708 \end{bmatrix}\text{ rad}$$
        
- **进入第 2 轮迭代**：将 $\theta^{(1)}$ 重新代入正运动学计算新的末端位置，计算新的残差 $e^{(1)}$，重新计算 $J(\theta^{(1)})$ 及其逆，重复上述步长更新，直到残差范数 $\Vert{}e^{(k)}\Vert{} < \epsilon$（如 $10^{-4}$）收敛退出。
    

> **为什么会失效？**
> 
> 当手臂完全伸直（$\theta_2 = 0$）时，$\det(J) = l_1 l_2 \sin\theta_2 = 0$。此时 $J$ 不可逆，数值法在分母为 0 处崩溃；在奇异点边缘时 $\det(J) \approx 0$，$J^{-1}$ 的元素数值变成成百上千，导致 $\Delta \theta$ 暴增（产生巨大角速度突变），造成液压系统发生水锤冲击与振颤。


## jacobian 计算多自由度的关节速度和末端速度


### 一、物理直观：从单关节到全机构的叠加

如果机械臂仅有第 $i$ 个关节在以角速度 $\dot{\theta}_i$ 转动，其他所有关节均锁死（$\dot{\theta}_k = 0, k \neq i$）：


--- start-multi-column: ID_2pc0
```column-settings
Number of Columns: 2
Largest Column: standard
```

- **产生的末端角速度**：方向直接沿该关节的旋转轴单位矢量 $\mathbf{z}_{i-1}$，大小即为 $\dot{\theta}_i$：
    
    $$\boldsymbol{\omega}_e^{(i)} = \mathbf{z}_{i-1} \dot{\theta}_i$$
    
    
    
    
    
    z负责确定方向，范数为1，θ确定速度，为标量

--- column-break ---



#### 矩阵运算机制

- $\mathbf{z}_{i-1} \in \mathbb{R}^{3 \times 1}$ 是第 $i$ 个关节在基坐标系下的旋转轴**单位方向向量**：
    
    $$\mathbf{z}_{i-1} = \begin{bmatrix} z_x \\ z_y \\ z_z \end{bmatrix}$$
    
- $\dot{\theta}_i \in \mathbb{R}$ 是一个**标量**（第 $i$ 个关节的角速度大小，单位 $\text{rad/s}$）。
    
- 运算本质是**标量乘向量（Scalar Multiplication）**：
    
    $$\boldsymbol{\omega}_e^{(i)} = \begin{bmatrix} z_x \cdot \dot{\theta}_i \\ z_y \cdot \dot{\theta}_i \\ z_z \cdot \dot{\theta}_i \end{bmatrix}$$

--- end-multi-column


- **产生的末端线速度**：如同以关节 $i$ 的原点 $\mathbf{p}_{i-1}$ 为定轴圆周运动，线速度由角速度叉乘力臂向量 $(\mathbf{p}_e - \mathbf{p}_{i-1})$ 得到：
    
    $$\mathbf{v}_e^{(i)} = \boldsymbol{\omega}_e^{(i)} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) = \left( \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \right) \dot{\theta}_i$$
	
	- 位矢差 $\mathbf{r} = \mathbf{p}_e - \mathbf{p}_{i-1}$，表示从**关节轴上的基准点**指向**末端执行器点**的力臂向量
	- 叉乘结果 $\mathbf{v}_e^{(i)}$ 的模长为 $\Vert{}\boldsymbol{\omega}\Vert{} \Vert{}\mathbf{r}\Vert{} \sin\phi$（其中 $\Vert{}\mathbf{r}\Vert{}\sin\phi$ 正好是末端点到旋转轴的**垂直最短距离**）。
    
    - 方向同时垂直于转轴 $\mathbf{z}_{i-1}$ 和位矢 $\mathbf{r}$，切于圆周运动轨迹。
    - Φ 是两向量的夹角

当**所有 $n$ 个关节同时运动**时，根据刚体运动学定理，末端执行器的总线速度 $\mathbf{v}_e$ 与总角速度 $\boldsymbol{\omega}_e$ 等于每一个关节单独运动所贡献速度的矢量和：

$$\mathbf{v}_e = \sum_{i=1}^{n} \mathbf{v}_e^{(i)} = \sum_{i=1}^{n} \left[ \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \right] \dot{\theta}_i$$

$$\boldsymbol{\omega}_e = \sum_{i=1}^{n} \boldsymbol{\omega}_e^{(i)} = \sum_{i=1}^{n} \mathbf{z}_{i-1} \dot{\theta}_i$$

# jacobian在计算中的作用

 - **几何雅可比矩阵（Geometric Jacobian $\mathbf{J}(\boldsymbol{\theta})$）本质上是连接“关节空间（Joint Space）”与“笛卡尔操作空间（Cartesian Operational Space）”的核心桥梁**。---角度坐标系到xyz坐标系
 #### 两种**本质等效、但推导路径不同**的雅可比构建方法
## 角速度线速度推导
### 角速度的“点乘”：标量与向量的数乘

公式：

$$\boldsymbol{\omega}_e^{(i)} = \mathbf{z}_{i-1} \dot{\theta}_i$$

#### 矩阵运算机制

- $\mathbf{z}_{i-1} \in \mathbb{R}^{3 \times 1}$ 是第 $i$ 个关节在基坐标系下的旋转轴**单位方向向量**：
    
    $$\mathbf{z}_{i-1} = \begin{bmatrix} z_x \\ z_y \\ z_z \end{bmatrix}$$
    
- $\dot{\theta}_i \in \mathbb{R}$ 是一个**标量**（第 $i$ 个关节的角速度大小，单位 $\text{rad/s}$）。
    
- 运算本质是**标量乘向量（Scalar Multiplication）**：
    
    $$\boldsymbol{\omega}_e^{(i)} = \begin{bmatrix} z_x \cdot \dot{\theta}_i \\ z_y \cdot \dot{\theta}_i \\ z_z \cdot \dot{\theta}_i \end{bmatrix}$$
    

#### 物理含义与效果

- **方向锁定**：刚体绕轴定轴转动时，角速度矢量的方向严格沿着转轴。因此，单位向量 $\mathbf{z}_{i-1}$ 确定了转动的轴向与正负朝向（由右手定则决定）。
    
- **强度缩放**：$\dot{\theta}_i$ 决定了转得有多快。
    
- **雅可比贡献**：将其写成矩阵形式 $\boldsymbol{\omega}_e^{(i)} = \mathbf{J}_{\omega i} \dot{\theta}_i$ 时，对应的角速度雅可比分块就是：
    
    $$\mathbf{J}_{\omega i} = \mathbf{z}_{i-1}$$
    

### 2. 线速度“叉乘”：反对称矩阵与线速度映射

公式：

$$\mathbf{v}_e^{(i)} = \boldsymbol{\omega}_e^{(i)} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) = \left( \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \right) \dot{\theta}_i$$

#### 几何直观

设位矢差 $\mathbf{r} = \mathbf{p}_e - \mathbf{p}_{i-1}$，表示从**关节轴上的基准点**指向**末端执行器点**的力臂向量：

- 高中/大学物理中圆周运动线速度公式为 $\mathbf{v} = \boldsymbol{\omega} \times \mathbf{r}$。
    
- 叉乘结果 $\mathbf{v}_e^{(i)}$ 的模长为 $\Vert{}\boldsymbol{\omega}\Vert{} \Vert{}\mathbf{r}\Vert{} \sin\phi$（其中 $\Vert{}\mathbf{r}\Vert{}\sin\phi$ 正好是末端点到旋转轴的**垂直最短距离**）。
    
- 方向同时垂直于转轴 $\mathbf{z}_{i-1}$ 和位矢 $\mathbf{r}$，切于圆周运动轨迹。

==🟠**本质等效、但推导路径不同**的雅可比构建方法==

--- start-multi-column: ID_wasn
```column-settings
Number of Columns: 2
Largest Column: standard
```

#### 1. 分析法（微积分偏导数路径）

**核心思想**：不考虑刚体转动的物理机制，纯粹将末端位置当作关节变量的显式多元函数，直接套用微积分链式法则。

1. **写出正运动学显式解析解**： 通过齐次变换矩阵相乘（$T_0^e = T_0^1 T_1^2 \cdots T_{n-1}^n$），得到末端坐标在基坐标系下的显式表达式：
    
    $$\mathbf{p}_e(\theta) = \begin{bmatrix} x(\theta_1, \dots, \theta_n) \\ y(\theta_1, \dots, \theta_n) \\ z(\theta_1, \dots, \theta_n) \end{bmatrix}$$
    
2. **对时间求全导数**：
    
    $$\mathbf{v}_e = \frac{d\mathbf{p}_e}{dt} = \sum_{i=1}^n \frac{\partial \mathbf{p}_e}{\partial \theta_i} \frac{d\theta_i}{dt} = \sum_{i=1}^n \frac{\partial \mathbf{p}_e}{\partial \theta_i} \dot{\theta}_i$$
    
3. **提取矩阵列向量**：
    
    $$\mathbf{J}_{vi} = \frac{\partial \mathbf{p}_e}{\partial \theta_i} = \begin{bmatrix} \frac{\partial x}{\partial \theta_i} \\ \frac{\partial y}{\partial \theta_i} \\ \frac{\partial z}{\partial \theta_i} \end{bmatrix}$$
    

- **缺点**：对于多自由度（如 6 自由度及以上）机械臂，正解表达式极其庞大，展开求符号偏导的计算量很大，容易产生复杂的三角函数展开式。

--- column-break ---

#### 2. 几何法（刚体运动学物理路径）

**核心思想**：绕过庞大的三角函数符号求导，直接利用**刚体定轴转动的圆周速度公式** $\mathbf{v} = \boldsymbol{\omega} \times \mathbf{r}$。

当只有关节 $i$ 以角速度 $\dot{\theta}_i$ 转动，其余关节全部锁死时：

1. 关节 $i$ 施加给后续连杆（包括末端 $e$）的角速度为：
    
    $$\boldsymbol{\omega}_i = \dot{\theta}_i \mathbf{z}_{i-1}$$
    
2. 末端执行器相对于转轴上某一点 $\mathbf{p}_{i-1}$ 的位置矢量（即力臂）为：
    
    $$\mathbf{r} = \mathbf{p}_e - \mathbf{p}_{i-1}$$
    
3. 刚体绕固定轴转动引起的线速度直接可写为：
    
    $$\mathbf{v}_{e, i} = \boldsymbol{\omega}_i \times \mathbf{r} = (\dot{\theta}_i \mathbf{z}_{i-1}) \times (\mathbf{p}_e - \mathbf{p}_{i-1}) = \left[ \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \right] \dot{\theta}_i$$
    
4. 对比 $\mathbf{v}_{e, i} = \mathbf{J}_{vi} \dot{\theta}_i$，直接得到：
    
    $$\mathbf{J}_{vi} = \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1})$$
    

- **优点**：$\mathbf{z}_{i-1}$ 和 $\mathbf{p}_{i-1}$ 直接从正运动学中间各连杆变换矩阵 $T_0^{i-1}$ 的旋转矩阵第三列和位移列中读取，无需符号偏导运算，几何意义清晰且计算极其高效。

--- end-multi-column

通过将每个关节的线速度和角速度贡献按列拼装：

$$\mathbf{V}_e = \begin{bmatrix} \mathbf{v}_e \\ \boldsymbol{\omega}_e \end{bmatrix} = \mathbf{J}(\boldsymbol{\theta}) \dot{\boldsymbol{\theta}} = \begin{bmatrix} \mathbf{J}_{v1} & \mathbf{J}_{v2} & \cdots & \mathbf{J}_{vn} \\ \mathbf{J}_{\omega 1} & \mathbf{J}_{\omega 2} & \cdots & \mathbf{J}_{\omega n} \end{bmatrix} \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_2 \\ \vdots \\ \dot{\theta}_n \end{bmatrix}$$

其中第 $i$ 列为：

$$\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{vi} \\ \mathbf{J}_{\omega i} \end{bmatrix} = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}$$
最终得到完整的矩阵形式：

$$\mathbf{V}_e = \mathbf{J}_1 \dot{\theta}_1 + \mathbf{J}_2 \dot{\theta}_2 + \cdots + \mathbf{J}_n \dot{\theta}_n = \begin{bmatrix} \mathbf{J}_1 & \mathbf{J}_2 & \cdots & \mathbf{J}_n \end{bmatrix} \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_2 \\ \vdots \\ \dot{\theta}_n \end{bmatrix} = \mathbf{J}(\boldsymbol{\theta}) \dot{\boldsymbol{\theta}}$$
#### 4. 这一拼装的直观几何意义

若让其他所有关节锁死不动（$\dot{\theta}_j = 0, \forall j \neq i$），仅让第 $i$ 个关节以单位速度转动（$\dot{\theta}_i = 1$），此时末端执行器产生的空间速度为：

$$\mathbf{V}_e = \mathbf{J} \begin{bmatrix} 0 \\ \vdots \\ 1 \\ \vdots \\ 0 \end{bmatrix} = \mathbf{J}_i = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}$$

**雅可比矩阵的第 $i$ 列，就是“当只有第 $i$ 个关节以 $1\text{ rad/s}$ 旋转时，末端执行器所产生的纯速度响应矢量”**。每一列代表一个关节的基底贡献，拼在一起就构成了整个多自由度系统的总速度变换矩阵。
#### 5. 拼装的物理含义
		线速度与角速度在物理上天然耦合，不能割裂

刚体运动学中，**三维刚体上任意一点的线速度，都直接依赖于该刚体的角速度和参考点位置**：

$$\boldsymbol{v}_e = \boldsymbol{v}_o + \boldsymbol{\omega} \times \boldsymbol{r}_{oe}$$

如果把它们割裂开：

- 单看角速度贡献 $\boldsymbol{J}_\omega$，完全不知道末端在空间中的移动快慢；
    
- 单看线速度贡献 $\boldsymbol{J}_v$，它其实由基座沿途所有关节的**转动诱导线速度**叠加而来，其数值直接受各关节角速度方向的制约。
    

将 $\begin{bmatrix} \boldsymbol{J}_v \\ \boldsymbol{J}_\omega \end{bmatrix}$ 上下拼合为列向量，数学上构成了李代数 $se(3)$ 中的空间旋量（Twist）基底。刚体在三维空间中有 6 个自由度（3 平移 + 3 旋转），只有将它们作为一个 6 维刚性整体来处理，才能完整描述刚体在空间中的瞬时位姿变化率。

### 2. 产生了新的物理判定：整机的“空间奇异性”（Kinematic Singularity）

如果单独看线速度部分 $\boldsymbol{J}_v$（$3 \times n$）或角速度部分 $\boldsymbol{J}_\omega$（$3 \times n$），它们各自可能都是满秩的；但一旦按列拼装成 $6 \times n$ 的整体雅可比矩阵 $\boldsymbol{J}$：

- **行列式与秩的变化带来新的物理状态**：当且仅当把两者拼在一起时，$\det(\boldsymbol{J}\boldsymbol{J}^T) = 0$ 才代表**整机在 6 自由度任务空间中丧失了某个方向的运动或姿态调整能力**。
    
- 例如：机械臂完全伸直时，仅看某些角度的旋转能力并未丧失，但整体在径向平移与俯仰角速度之间产生了强耦合退化，导致自由度丢失。这种系统级的退化特性，单看局部矩阵是无法判定的。
    

### 3. 建立了统一的“能量对偶映射”（虚功原理）

在静力学与动力学中，拼装带来的核心物理工具是**虚功等效原理**：

$$\delta W = \boldsymbol{\tau}^T \delta \boldsymbol{q} = \boldsymbol{F}^T \delta \boldsymbol{x} + \boldsymbol{M}^T \delta \boldsymbol{\phi} = \begin{bmatrix} \boldsymbol{F}^T & \boldsymbol{M}^T \end{bmatrix} \begin{bmatrix} \delta \boldsymbol{x} \\ \delta \boldsymbol{\phi} \end{bmatrix}$$

末端受到的**外力 $\boldsymbol{F}$（对应线速度）**与**力矩 $\boldsymbol{M}$（对应角速度）**共同构成了一个 6 维**扳手旋量（Wrench）** $\mathcal{F} = \begin{bmatrix} \boldsymbol{F} \\ \boldsymbol{M} \end{bmatrix}$。

正因为速度雅可比按列统一拼装成 6 维矩阵，才使得**转置矩阵**天然具备了统一映射力与力矩的物理规律：

$$\boldsymbol{\tau} = \boldsymbol{J}^T \mathcal{F} = \boldsymbol{J}_v^T \boldsymbol{F} + \boldsymbol{J}_\omega^T \boldsymbol{M}$$

这是机器人刚度控制、阻抗控制以及动力学方程（如拉格朗日法/牛顿-欧拉法）推导的统一桥梁。

- **横向按列拼**：对应**多关节输入的线性代数叠加**（便于矩阵乘法 $\boldsymbol{J}\dot{\boldsymbol{q}}$）；
    
- **纵向按块拼（线+角）**：对应**空间刚体 6 维自由度的完整动力学与几何统一**，赋予了系统完整的空间旋量、整机奇异性判据以及力/力矩对偶映射能力。














## 【关联概念（双链）】

- 前置依赖概念：
    
      
    - [[多元函数微分与偏导数]]
        
          
        
    - [[线性变换与矩阵的秩]]
        
          
        
    - [[行列式的几何意义]]
        
          
        
- 核心应用概念：
    
      
    - [[正运动学]]
        
          
        
    - [[逆运动学]]
        
          
        
    - [[雅可比矩阵]]
        
          
        
    - [[运动学奇异点]]
        
          
        
- 进阶横向与延伸概念：
    
      
    - [[Moore–Penrose 伪逆]]
        
          
        
    - [[可操作度椭球]]
        
          
        
    - [[阻尼最小二乘法]]
        
          
        
    - [[旋量理论与 Twist/Wrench]]![[Attachments/Pasted image 20261026142854.png]]