
# 轨迹描述（Cartesian Space到Joint-Space）
### 核心概念梳理：${}^{G}X_{T_i}$ 的形成链路与矩阵推导

在机器人运动学与轨迹规划体系中，状态向量 ${}^{G}X_{T_i}$ 的构建本质上是为了解决 **$4 \times 4$ 齐次变换矩阵在笛卡尔空间做插值运算时的非线性失真与参数冗余问题**。

  

#### 1. 形成链路与推导全过程

```
[任务需求] 机械臂末端沿路径移动 (Via points: t_0, t_i, t_f)
   │
   ▼
[完整位姿描述] 齐次变换矩阵 {}_{T}^{G}T_i ∈ SE(3) (4×4 矩阵，含 16 个元素)
   │
   ▼
[参数解耦] 拆分为 3 维位置向量 + 3×3 旋转矩阵 {}^{G}R_T ∈ SO(3)
   │
   ▼
[姿态降维] 欧拉旋转定理：任意旋转等价于绕单位旋转轴 ^G\hat{K}_T 旋转角度 θ
   │
   ▼
[压缩表达] 轴角相乘得等效旋转向量 r = θ · ^G\hat{K}_T (3 维)
   │
   ▼
[最终形式] 6 维紧凑位姿状态向量 ^GX_{T_i} = [x, y, z, r_x, r_y, r_z]^T
```

##### 矩阵推导过程：从 ${}_{T}^{G}T$ 到 ${}^{G}X_T$ 的正向与逆向映射

**步骤一：齐次变换矩阵拆解**

末端坐标系 $\{T\}$ 相对于基准坐标系 $\{G\}$ 的经典 $4 \times 4$ 矩阵描述：

  

$${}_{T}^{G}T = \begin{bmatrix} {}^{G}R_T & {}^{G}P_{T\,org} \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix} = \begin{bmatrix} r_{11} & r_{12} & r_{13} & x \\ r_{21} & r_{22} & r_{23} & y \\ r_{31} & r_{32} & r_{33} & z \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**步骤二：从旋转矩阵 ${}^{G}R_T$ 提取轴角参数 $(\theta, {}^{G}\hat{K}_T)$**

利用矩阵迹（Trace）的不变量性质计算旋转角 $\theta$：

  

$$\operatorname{tr}({}^{G}R_T) = r_{11} + r_{22} + r_{33} = 1 + 2\cos\theta$$

$$\theta = \arccos\left(\frac{r_{11} + r_{22} + r_{33} - 1}{2}\right)$$

通过反对称矩阵提取空间旋转轴 $^{G}\hat{K}_T = [k_x, k_y, k_z]^T$：

  

$${}^{G}R_T - {}^{G}R_T^T = 2\sin\theta \, [{}^{G}\hat{K}_T]_\times = 2\sin\theta \begin{bmatrix} 0 & -k_z & k_y \\ k_z & 0 & -k_x \\ -k_y & k_x & 0 \end{bmatrix}$$

$${}^{G}\hat{K}_T = \frac{1}{2\sin\theta}\begin{bmatrix} r_{32} - r_{23} \\ r_{13} - r_{31} \\ r_{21} - r_{12} \end{bmatrix} \quad (\theta \neq 0, \pi)$$

**步骤三：组装等效旋转向量与最终 6 维位姿向量**

  

$$\mathbf{r} = \theta \cdot {}^{G}\hat{K}_T = \begin{bmatrix} \theta k_x \\ \theta k_y \\ \theta k_z \end{bmatrix} = \begin{bmatrix} r_x \\ r_y \\ r_z \end{bmatrix}$$

组装得到紧凑位姿：

  

$${}^{G}X_T = \begin{bmatrix} {}^{G}P_{T\,org} \\ \mathbf{r} \end{bmatrix} = \begin{bmatrix} x \\ y \\ z \\ r_x \\ r_y \\ r_z \end{bmatrix}$$

**步骤四：从 6 维向量还原旋转矩阵（罗德里格斯公式 Rodrigues' Formula）**

当控制器在笛卡尔空间插值完成后，将 $\mathbf{r} = [r_x, r_y, r_z]^T$ 还原为物理旋转矩阵：

  

1. 计算范数获取角度：$\theta = \Vert{}\mathbf{r}\Vert{}_2 = \sqrt{r_x^2 + r_y^2 + r_z^2}$
    
      
    
2. 获取单位轴：${}^{G}\hat{K}_T = \mathbf{r} / \theta$
    
      
    
3. 重构旋转矩阵：
    
      
    
    $${}^{G}R_T = I + \sin\theta \, [{}^{G}\hat{K}_T]_\times + (1 - \cos\theta) \, [{}^{G}\hat{K}_T]_\times^2$$
    

### 深入拆解：绕 Z 轴旋转示例的几何与物理实质

针对示例：

  

$${}^{G}\hat{K}_T = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}, \quad \theta = \frac{\pi}{2}, \quad \mathbf{r} = \begin{bmatrix} 0 \\ 0 \\ 1.57 \end{bmatrix}$$

#### 1. 这里是什么绕着什么在转？

- **转动的主体**：是**末端工具坐标系 $\{T\}$ 的整个姿态方向**（即机械臂法兰盘或末端工具固定绑定的那套局部坐标轴 $X_T, Y_T, Z_T$）。
- **旋转的轴心线**：是**基准参考系 $\{G\}$ 的 $Z$ 轴方向**。

- **物理运动过程**：想象末端工具原点固定在空间某点，以平行于基座 $Z$ 轴的一根空间虚拟垂线为旋转轴，将末端工具像拧螺丝一样逆时针拧动 $90^\circ$。
#### 2. 以哪里为 0 度转？

- **姿态基准（0度起点）**：以**末端坐标系 $\{T\}$ 的三个轴方向与基准坐标系 $\{G\}$ 的三个轴完全平行同向时**为 0 度姿态（即初始旋转矩阵为单位阵 $I$）。
    
      
    - 0度时：$X_T \parallel X_G, \; Y_T \parallel Y_G, \; Z_T \parallel Z_G \implies R = I$。
        
          
        
- **角度计量的起点**：从 $R = I$ 的初始朝向开始，沿右手螺旋定则，大拇指指向 ${}^{G}\hat{K}_T = [0, 0, 1]^T$（即基座 $Z_G$ 正方向），四指弯曲方向即为角度正方向。从 0 度旋转到 $\frac{\pi}{2}$。
    
      
    

#### 3. 它和位姿的关系是什么？

- **位姿 = 位置（Position）+ 姿态（Orientation）**。
    
      
    
- 向量 $\mathbf{r} = [0, 0, 1.57]^T$ **完全决定了位姿中的“姿态部分”**。
    
      
    
- 代入罗德里格斯公式还原其对应的姿态变换矩阵：
    
      
    
    $${}^{G}R_T = \begin{bmatrix} \cos(90^\circ) & -\sin(90^\circ) & 0 \\ \sin(90^\circ) & \cos(90^\circ) & 0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
    
- **最终空间位姿关系呈现为**：
    
      
    - 末端现在的 $X_T$ 轴指向了基座的 $+Y_G$ 方向（第一列 $[0, 1, 0]^T$）；
        
          
        
    - 末端现在的 $Y_T$ 轴指向了基座的 $-X_G$ 方向（第二列 $[-1, 0, 0]^T$）；
        
          
        
    - 末端现在的 $Z_T$ 轴仍与基座的 $+Z_G$ 方向保持一致（第三列 $[0, 0, 1]^T$）。
        
          
        
    - 位置向量 ${}^{G}P_{T\,org} = [x, y, z]^T$ 确定该姿态在三维空间中的具体驻留坐标。
        
          
        

### Obsidian 双链关键词整理

可以直接复制至个人笔记库作为核心标签与双向链接引用：

  

Markdown

```
## 机械臂运动学与轨迹规划索引
- [[笛卡尔空间]] (Cartesian Space / Task Space)
- [[关节空间]] (Joint Space)
- [[位姿描述]] (Pose Representation)
- [[齐次变换矩阵]] (Homogeneous Transformation Matrix)
- [[姿态（Orientation）]]
- [[三维姿态]]
- [[旋转矩阵]] (SO(3))
- [[欧拉角]] (Euler Angles / RPY)
- [[轴角]] (Axis-Angle Representation)
- [[单位旋转轴向量]] (^G\hat{K}_T)
- [[等效旋转向量]] (Rodrigues Vector)
- [[ROT(^G\hat{K}_T, θ) 对应的矩阵]] (罗德里格斯公式 Rodrigues' Formula)
- [[欧几里得范数]] (Euclidean Norm / L2 Norm)
- [[轨迹插值]] (Trajectory Interpolation)
- [[路标点]] (Via Points)[cite: 1, 2]
- [[逆运动学求解]] (Inverse Kinematics / IK)[cite: 2]
```

### 验证与工程应用流程

在代码实现笛卡尔空间路径规划时，按以下规范进行断言与矩阵校验：

  

1. **输入合法性验证**：提取姿态向量 $\mathbf{r}$ 时，若其模长 $\Vert{}\mathbf{r}\Vert{}_2 < 1\times 10^{-6}$，直接返回单位阵 $I_{3 \times 3}$，跳过除零逻辑。
    
      
    
2. **正交性验证**：由罗德里格斯公式生成的 ${}^{G}R_T$，计算 $\det({}^{G}R_T) = 1.0$ 且 $\Vert{}{}^{G}R_T \cdot {}^{G}R_T^T - I\Vert{} < 1\times 10^{-5}$，保证不出现几何尺寸畸变。
    
      
    
3. **闭环逆解传递**：将重构出的 ${}_{T}^{G}T$ 输入 IK 算法前，确保关节解挑选满足相邻路径点差值 $\min \sum \vert{}\theta_{k} - \theta_{k-1}\vert{}$，规避构型突变。

![[Pasted image 20261016174200.png]]![[Pasted image 20261016174254.png]]

# 轨迹规划-三次多项式插值（Cubic Polynomial Interpolation）

## 1. 基本概念

**三次多项式插值**：在两个已知关节状态之间，构造一条随时间变化的平滑轨迹，使关节在任意时刻都能计算出对应的角度、角速度和角加速度。

机器人关节轨迹规划中：

- (\theta(t))：关节角度
    
- (\dot{\theta}(t))：关节角速度
    
- (\ddot{\theta}(t))：关节角加速度
    

三次多项式的一般形式：

$$  
\theta(t)=a_0+a_1t+a_2t^2+a_3t^3  
$$

对时间求导：

$$  
\dot{\theta}(t)=a_1+2a_2t+3a_3t^2  
$$

$$  
\ddot{\theta}(t)=2a_2+6a_3t  
$$

其中，(a_0,a_1,a_2,a_3) 为待求系数。

## 2. 四个边界约束条件

设运动时间区间为 ([t_i,t_{i+1}])，时间间隔为：

$$  
T=t_{i+1}-t_i  
$$

为了简化计算，令局部时间：

$$  
\tau=t-t_i,\qquad \tau\in[0,T]  
$$

此时多项式写为：

$$  
\theta(\tau)=a_0+a_1\tau+a_2\tau^2+a_3\tau^3  
$$

需要满足四个边界条件：

|条件|数学表达式|含义|
|---|---|---|
|起点位置|(\theta(0)=\theta_i)|起点角度已知|
|终点位置|(\theta(T)=\theta_{i+1})|终点角度已知|
|起点速度|(\dot{\theta}(0)=\dot{\theta}_i)|起点角速度已知|
|终点速度|(\dot{\theta}(T)=\dot{\theta}_{i+1})|终点角速度已知|

**核心逻辑：** 三次多项式有 4 个待定系数，四个独立边界条件可以唯一确定这四个系数。

## 3. 求解多项式系数

将四个边界条件代入多项式及其导数：

$$  
\begin{cases}  
a_0=\theta_i\  
a_0+a_1T+a_2T^2+a_3T^3=\theta_{i+1}\  
a_1=\dot{\theta}_i\  
a_1+2a_2T+3a_3T^2=\dot{\theta}_{i+1}  
\end{cases}  
$$

解得：

$$  
a_0=\theta_i  
$$

$$  
a_1=\dot{\theta}_i  
$$

$$  
a_2=\frac{3(\theta_{i+1}-\theta_i)}{T^2}  
-\frac{2\dot{\theta}_i+\dot{\theta}_{i+1}}{T}  
$$

$$  
a_3=-\frac{2(\theta_{i+1}-\theta_i)}{T^3}  
+\frac{\dot{\theta}_i+\dot{\theta}_{i+1}}{T^2}  
$$

将四个系数代回多项式，即可得到完整的关节运动轨迹。

## 4. 矩阵形式

四个边界条件可以写成线性方程组：

$$  
\boldsymbol{\Theta}=\boldsymbol{M}\boldsymbol{A}  
$$

其中：

$$  
\boldsymbol{\Theta}=  
\begin{bmatrix}  
\theta_i\  
\theta_{i+1}\  
\dot{\theta}_i\  
\dot{\theta}_{i+1}  
\end{bmatrix}  
$$

$$  
\boldsymbol{A}=  
\begin{bmatrix}  
a_0\  
a_1\  
a_2\  
a_3  
\end{bmatrix}  
$$

$$  
\boldsymbol{M}=  
\begin{bmatrix}  
1&0&0&0\  
1&T&T^2&T^3\  
0&1&0&0\  
0&1&2T&3T^2  
\end{bmatrix}  
$$

当 (T\neq0) 时，矩阵可逆，因此：

$$  
\boldsymbol{A}=\boldsymbol{M}^{-1}\boldsymbol{\Theta}  
$$

矩阵行列式为：

$$  
\det(\boldsymbol{M})=T^4\neq0  
$$

因此，给定起止位置、起止速度及非零运动时间，就可以唯一求出三次多项式系数。

## 5. 运动连续性

|物理量|三次多项式的特点|
|---|---|
|位置 (\theta)|在各段连接处满足位置约束时连续|
|速度 (\dot{\theta})|在各段连接处满足相同速度约束时连续|
|加速度 (\ddot{\theta})|一般不能保证各段连接处连续|

注意：单段三次多项式的加速度是时间的一次函数。相邻两段的连接处，如果前一段终点加速度与后一段起点加速度不相等，加速度就会发生跳变。

如果希望同时约束起止位置、速度和加速度，通常采用**五次多项式插值**，其具有 6 个系数，可以满足 6 个边界条件。

## 6. 不同插值方法对比

|方法|最高次数|常见约束|主要特点|
|---|---|---|---|
|线性插值|1 次|起止位置|位置连续，速度可能跳变|
|三次多项式|3 次|起止位置、起止速度|可保证位置和速度连续|
|五次多项式|5 次|起止位置、速度、加速度|可保证位置、速度和加速度连续|

## 7. 机器人中的应用

三次多项式插值常用于机械臂关节的点到点轨迹规划。

基本流程：

1. 确定起点、终点的关节角度。
    
2. 确定起点、终点的角速度。
    
3. 设定运动时间 (T)。
    
4. 求出 (a_0,a_1,a_2,a_3)。
    
5. 将任意时刻 (\tau) 代入多项式，计算角度、角速度和角加速度。
    
6. 将规划结果发送给关节控制器。
    

## 8. 易错点

- **局部时间与绝对时间：** 使用 (\tau=t-t_i) 时，起点对应 (\tau=0)，终点对应 (\tau=T)。
    
- **矩阵行列式：** 按照本文矩阵的行排列，(\det(\boldsymbol{M})=T^4)，不是负值。
    
- **速度连续不等于加速度连续：** 三次多项式不能自动保证多段轨迹的加速度连续。
    
- **多项式次数与约束数量：** (n) 次多项式有 (n+1) 个系数；要唯一确定系数，通常需要 (n+1) 个独立约束。
    
- **更高次数不一定更好：** 在四个约束条件下，四次或五次多项式通常不能唯一确定，必须增加约束或采用其他优化准则。
    

## 总结

**三次多项式插值的核心：利用起点、终点的位置和速度这四个边界条件，求解四个多项式系数，从而得到可计算的关节轨迹。**

$$  
\boxed{  
\text{四个边界条件}  
\longrightarrow  
\text{求解四个系数}  
\longrightarrow  
\text{得到关节轨迹}  
}  
$$
# Cubic Polynomials（三次多项式轨迹规划）

## 1. 多段轨迹中的途经点速度

多段三次多项式轨迹规划中，每一段都需要 4 个边界条件：

- 起点位置、起点速度
    
- 终点位置、终点速度
    

对于中间途经点，角速度通常有两种确定方式。

### 1.1 直接指定速度

人工指定每个途经点的关节角速度：

$$  
\dot{\theta}_i=v_i  
$$

优点：可以根据具体任务控制运动趋势。

缺点：需要人为设计，难度较高；不恰当的速度设置可能造成超调、关节速度过大或轨迹不平滑。

### 1.2 根据相邻轨迹斜率估算速度

定义前一段和后一段的平均斜率：

$$  
m_{i-1}=\frac{\theta_i-\theta_{i-1}}{t_i-t_{i-1}}  
$$

$$  
m_i=\frac{\theta_{i+1}-\theta_i}{t_{i+1}-t_i}  
$$

一种简单的节点速度估算方法为：

$$  
\dot{\theta}_i=\frac{m_{i-1}+m_i}{2}  
$$

特殊情况：

- 如果前后斜率异号，节点为局部极值点时，常将节点速度设为零。
    
- 如果前后斜率同号，可使用平均斜率估算节点速度。
    

**注意：** 直接取两个斜率的算术平均值是一种简化方法，并非适用于所有非等时间间隔轨迹的最佳算法。实际规划中还需考虑速度、加速度限制及关节约束。

当各节点的位置和速度确定后，每段三次多项式可以独立求解，但这通常不能保证加速度在途经点连续。

---

## 2. 单段三次多项式

### 2.1 基本模型

令时间区间为 ([0,T])，其中 (T=\Delta t>0)。

三次多项式：

$$  
\theta(t)=a_0+a_1t+a_2t^2+a_3t^3  
$$

角速度：

$$  
\dot{\theta}(t)=a_1+2a_2t+3a_3t^2  
$$

角加速度：

$$  
\ddot{\theta}(t)=2a_2+6a_3t  
$$

其中 (a_0,a_1,a_2,a_3) 为待求系数。

### 2.2 四个边界条件

$$  
\begin{cases}  
\theta(0)=\theta_i\  
\theta(T)=\theta_{i+1}\  
\dot{\theta}(0)=\dot{\theta}_i\  
\dot{\theta}(T)=\dot{\theta}_{i+1}  
\end{cases}  
$$

将条件代入可得：

$$  
\begin{cases}  
a_0=\theta_i\  
a_0+a_1T+a_2T^2+a_3T^3=\theta_{i+1}\  
a_1=\dot{\theta}_i\  
a_1+2a_2T+3a_3T^2=\dot{\theta}_{i+1}  
\end{cases}  
$$

### 2.3 系数解析解

$$  
a_0=\theta_i  
$$

$$  
a_1=\dot{\theta}_i  
$$

$$  
a_2=  
\frac{3(\theta_{i+1}-\theta_i)}{T^2}  
-\frac{2\dot{\theta}_i+\dot{\theta}_{i+1}}{T}  
$$

$$  
a_3=  
-\frac{2(\theta_{i+1}-\theta_i)}{T^3}  
+\frac{\dot{\theta}_i+\dot{\theta}_{i+1}}{T^2}  
$$

求出系数后，就可以计算任意时刻的关节角度、角速度和角加速度。

### 2.4 矩阵形式

将四个约束写成矩阵方程：

$$  
\boldsymbol{\Theta}=\boldsymbol{M}\boldsymbol{A}  
$$

其中：

$$  
\boldsymbol{\Theta}=  
\begin{bmatrix}  
\theta_i\  
\theta_{i+1}\  
\dot{\theta}_i\  
\dot{\theta}_{i+1}  
\end{bmatrix}  
$$

$$  
\boldsymbol{M}=  
\begin{bmatrix}  
1&0&0&0\  
1&T&T^2&T^3\  
0&1&0&0\  
0&1&2T&3T^2  
\end{bmatrix}  
$$

$$  
\boldsymbol{A}=  
\begin{bmatrix}  
a_0\a_1\a_2\a_3  
\end{bmatrix}  
$$

当 (T\neq0) 时，矩阵可逆：

$$  
\det(\boldsymbol{M})=T^4\neq0  
$$

因此：

$$  
\boxed{\boldsymbol{A}=\boldsymbol{M}^{-1}\boldsymbol{\Theta}}  
$$

实际编程中，通常采用线性方程组求解器，而不是显式计算逆矩阵。

### 2.5 单段轨迹的连续性

- 位置：由边界条件保证到达指定起点和终点。
    
- 速度：由边界条件保证到达指定起点和终点速度。
    
- 加速度：单段内连续，但与相邻轨迹段连接时不一定连续。
    

如果相邻两段在途经点使用相同的位置和速度约束，则位置和速度连续；若两段加速度不相等，则加速度发生跳变。

---

## 3. 多段联立三次多项式

### 3.1 规划目标

单段独立求解时，每一段只满足自身的边界条件。若希望途经点处的加速度也连续，可以将相邻轨迹段联立求解。

以两段轨迹为例：

$$  
\theta_I(t)=a_{10}+a_{11}t+a_{12}t^2+a_{13}t^3  
$$

$$  
t\in[0,T_1]  
$$

$$  
\theta_{II}(s)=a_{20}+a_{21}s+a_{22}s^2+a_{23}s^3  
$$

$$  
s\in[0,T_2]  
$$

其中 (t) 和 (s) 分别是两段轨迹的局部时间。

两段共包含 8 个待求系数，因此需要 8 个独立约束条件。

### 3.2 八个约束条件

设起点位置为 (\theta_0)，途经点位置为 (\theta_1)，终点位置为 (\theta_f)。

**位置约束：**

$$  
\theta_I(0)=\theta_0  
$$

$$  
\theta_I(T_1)=\theta_1  
$$

$$  
\theta_{II}(0)=\theta_1  
$$

$$  
\theta_{II}(T_2)=\theta_f  
$$

**起点与终点速度约束：**

$$  
\dot{\theta}_I(0)=\dot{\theta}_0  
$$

$$  
\dot{\theta}_{II}(T_2)=\dot{\theta}_f  
$$

**途经点速度连续约束：**

$$  
\dot{\theta}_I(T_1)=\dot{\theta}_{II}(0)  
$$

**途经点加速度连续约束：**

$$  
\ddot{\theta}_I(T_1)=\ddot{\theta}_{II}(0)  
$$

以上共 8 个独立方程，可以组成一个 (8\times8) 线性方程组。

### 3.3 矩阵形式

$$  
\boldsymbol{\Theta}_8=\boldsymbol{M}_8\boldsymbol{A}_8  
$$

其中：

$$  
\boldsymbol{A}_8=  
\begin{bmatrix}  
a_{10}\a_{11}\a_{12}\a_{13}\  
a_{20}\a_{21}\a_{22}\a_{23}  
\end{bmatrix}  
$$

右侧约束向量可以写为：

$$  
\boldsymbol{\Theta}_8=  
\begin{bmatrix}  
\theta_0\  
\theta_1\  
\theta_1\  
\theta_f\  
\dot{\theta}_0\  
\dot{\theta}_f\  
0\  
0  
\end{bmatrix}  
$$

矩阵的各行分别由位置、速度、加速度约束展开得到。

对于上述约束排列和局部时间定义，矩阵行列式为：

$$  
\det(\boldsymbol{M}_8)  
=4T_1^3T_2^3(T_1+T_2)  
$$

因此，当 (T_1>0)、(T_2>0) 时，矩阵可逆，可以唯一求出全部 8 个系数。

$$  
\boxed{\boldsymbol{A}_8=\boldsymbol{M}_8^{-1}\boldsymbol{\Theta}_8}  
$$

同样，实际计算中建议使用线性方程组求解器。

### 3.4 多段联立的特点

- 位置连续。
    
- 速度连续。
    
- 加速度连续。
    
- 两段轨迹不再完全独立，系数需要共同求解。
    

**注意：** 两段三次多项式联立并保证加速度连续，并不意味着整个轨迹是全局最优的三次样条。经典三次样条还涉及节点导数或其他边界条件的整体求解。

---

## 4. 两种方案对比

|对比项目|分段独立三次多项式|联立三次多项式|
|---|---|---|
|系数求解|每段单独求解|多段共同求解|
|两段轨迹规模|每段 (4\times4)|两段 (8\times8)|
|位置连续|满足节点位置约束时保证|通过约束保证|
|速度连续|需要统一节点速度|通过约束保证|
|加速度连续|一般不保证|通过约束保证|
|计算复杂度|较低|随联立规模增加|
|适用场景|简单点到点轨迹|对节点平滑性要求更高的轨迹|

---

## 5. 重要知识点

### 5.1 为什么三次多项式有四个系数？

(n) 次多项式具有 (n+1) 个待定系数。

三次多项式：

$$  
\theta(t)=a_0+a_1t+a_2t^2+a_3t^3  
$$

共有 4 个系数，因此需要 4 个独立约束才能唯一确定。

### 5.2 为什么多段轨迹需要联立？

每段独立求解，只能保证各段满足各自的边界条件。若还需要节点加速度连续，就必须增加跨段约束，并联合求解相关系数。

### 5.3 需要注意的工程问题

- 角度、时间及角速度单位必须保持一致。
    
- 运动时间必须大于零。
    
- 规划结果还应检查关节速度、加速度和执行器限制。
    
- 位置与速度连续并不代表加速度连续。
    
- 加速度连续也不代表加加速度（Jerk）连续。
    

---

## 6. 总结

**分段独立三次多项式：** 通过每段 4 个边界条件独立求解，计算简单，但一般不能保证节点加速度连续。

**多段联立三次多项式：** 增加节点速度和加速度连续约束，联合求解多个轨迹段，可以获得更平滑的运动。

核心流程：

$$  
\boxed{  
\text{确定途经点}  
\rightarrow  
\text{建立边界条件}  
\rightarrow  
\text{构造矩阵方程}  
\rightarrow  
\text{求解系数}  
\rightarrow  
\text{检查连续性与运动约束}  
}  
$$

**关键词：** `Cubic Polynomials`、`Joint Trajectory Planning`、`Via Points`、`Boundary Conditions`、`Continuity`、`Trajectory Interpolation`

# ==🟣是你说减肥咖啡==

