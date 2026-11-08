# Policy gradient
**梯度上升
## 参数网络θ
$$\pi_\theta(s) = P[A \mid s; \theta]$$
### 参数解读 
 -  $s$表示策略网络接收当前状态
 - 输出的是**在当前状态下采取各个动作的概率分布**
## 目标函数（Objective Function）
$$J(\theta) = \sum_{\tau} P(\tau; \theta) R(\tau)$$
#### 参数解读
- **$\tau$（读作 Tau，轨迹 / Trajectory）**： 智能体从开始到结束完成一整轮交互所经历的状态和动作序列：$$\tau = (s_0, a_0, s_1, a_1, \dots, s_T, a_T)$$
- **$R(\tau)$（整条轨迹的累计回报 / Return）**： 智能体沿着这条具体轨迹跑完一轮所拿到的所有奖励之和：$$R(\tau) = \sum_{t=0}^T r(s_t, a_t)$$
- **$P(\tau; \theta)$（这条轨迹出现的概率）**： 使用参数为 $\theta$ 的策略网络时，系统走出轨迹 $\tau$ 的联合概率。它由策略网络输出的动作概率以及环境的状态转移概率共同决定：$$P(\tau; \theta) = P(s_0) \prod_{t=0}^{T} \pi_\theta(a_t \mid s_t) P(s_{t+1} \mid s_t, a_t)$$
	- 这个概率不需要求出来，会在反向更新的梯度求导的时候被消掉，（因为没有θ）
	- 
- **$\sum_\tau P(\tau; \theta) R(\tau)$（按概率加权求和）**： 因为环境存在随机性、策略本身也是随机概率分布，所以每一轮采样的轨迹可能不同。将“每条可能跑出来的轨迹的回报 $R(\tau)$”乘以“这条轨迹跑出来的概率 $P(\tau; \theta)$”全部加起来，得到的就是**数学期望（平均表现）**。
## θ参θ参数的更新策略
###  ==🟠使用对数导数技巧==转化-----$\nabla_\theta P(\tau; \theta)$
#### 第一步：在原本的式子上同时乘以并除以同一个非零项 $P(\tau; \theta)$

我们要对目标项 $\nabla_\theta P(\tau; \theta)$ 进行恒等变形。只要乘以并除以同一个数，值保持不变：

$$\nabla_\theta P(\tau; \theta) = P(\tau; \theta) \cdot \frac{\nabla_\theta P(\tau; \theta)}{P(\tau; \theta)}$$

#### 第二步：观察分式部分 $\frac{\nabla_\theta P(\tau; \theta)}{P(\tau; \theta)}$

把上式的后半部分单独提出来看：

$$\frac{\nabla_\theta P(\tau; \theta)}{P(\tau; \theta)} = \frac{1}{P(\tau; \theta)} \cdot \nabla_\theta P(\tau; \theta)$$

根据多元微积分的**复合函数链式法则**（反向套用对数求导）： 对任意正值函数 $u$，都有 $\frac{d}{du}(\log u) = \frac{1}{u}$。 因此：

$$\nabla_\theta \log P(\tau; \theta) = \frac{1}{P(\tau; \theta)} \cdot \nabla_\theta P(\tau; \theta)$$

这说明后半部分的这一坨分式，**在数学上严格等于 $\nabla_\theta \log P(\tau; \theta)$**。

#### 第三步：把对数求导项替换回第一步中

将第二步识别出来的等价值：

$$\frac{\nabla_\theta P(\tau; \theta)}{P(\tau; \theta)} = \nabla_\theta \log P(\tau; \theta)$$

直接替换回第一步的式子里：

$$\nabla_\theta P(\tau; \theta) = P(\tau; \theta) \cdot \nabla_\theta \log P(\tau; \theta)$$
 
### 进行梯度求解$$\nabla_\theta J(\theta) = \mathbb{E} [\; \underbrace{\nabla_\theta \log \pi_\theta(a_t \mid s_t)}_{\text{方向（Direction）}} \;\times\; \underbrace{\hat{A}_t}_{\text{步长与正负（Magnitude \& Sign）}} \;]$$==🔴这个Direction给出的是每条通路的哪个步骤需要增强或者削弱==
 - 把前面写出的整条轨迹概率展开并取对数：

$$P(\tau; \theta) = P(s_0) \prod_{t=0}^{T} \pi_\theta(a_t \mid s_t) P(s_{t+1} \mid s_t, a_t)$$

 - 两边取自然对数 $\log$，乘法全部变成了加法：

$$\log P(\tau; \theta) = \log P(s_0) + \sum_{t=0}^{T} \log \pi_\theta(a_t \mid s_t) + \sum_{t=0}^{T} \log P(s_{t+1} \mid s_t, a_t)$$

 - 关键步骤来了：**现在对网络参数 $\theta$ 求梯度（求导）**：

	- $\log P(s_0)$ 里面没有参数 $\theta$ $\implies \nabla_\theta \log P(s_0) = 0$
    
	- **$\log P(s_{t+1} \mid s_t, a_t)$ 里面完全没有参数 $\theta$ $\implies \nabla_\theta \log P(s_{t+1} \mid s_t, a_t) = 0$**
    

 - 那些我们算不出来、也干预不了的环境物理概率项，在求导时**直接变成了 0，彻底消失了**：

$$\nabla_\theta \log P(\tau; \theta) = \sum_{t=0}^{T} \nabla_\theta \log \pi_\theta(a_t \mid s_t)$$

- 最终求出的策略梯度变成了：
#### $L^{PG}(\theta)$（代理损失 / 目标）
#### $L^{PG}(\theta)$（代理损失 / 目标）

		$$L^{PG}(\theta)=L^{PG}(\theta)=\nabla_\theta J(\theta) = \mathbb{E}_{\tau \sim \pi_\theta} \left[ \sum_{t=0}^{T} \nabla_\theta \log \pi_\theta(a_t \mid s_t) \cdot R(\tau) \right]$$
- 下标 **$\tau \sim \pi_\theta$** 规定了从哪里取平均：**对策略 $\pi_\theta$ 能跑出来的所有可能轨迹取平均**。	
	- **$\tau$（轨迹，Trajectory）**：从起点到终点的一整串状态和动作序列 $(s_0, a_0, s_1, a_1, \dots, s_T)$。
	    
		- **$\sim$（采样 / 服从分布）**：表示“从右边的策略和环境动力学所构成的轨迹分布中抽取出来”。
	    
		- **$\pi_\theta$（当前策略网络）**：动作的决策大脑，参数为 $\theta$。


- 此时$\pi_\theta(a_t \mid s_t)$只是==🔴一个具体发生动作==的概率，不像正向传播时给出所有可能概率

# Actor-Critic 架构
## $R(\tau)$的缺点
- **时序因果倒置（因果性问题）**：
    
    - 假设机械臂在第 0 秒做了一个极好的起手动作，但在第 9 秒电机由于抖动脱轨了，整条轨迹的总分 $R(\tau)$ 极低（甚至是负分）。
        
    - 如果用总分 $R(\tau)$ 加权，第 0 秒那个优秀的起手动作也会跟着被惩罚，这显然不合理。
        
- **方差极大（Variance 爆炸）**：
    
    - 比如一个任务每条轨迹的分数都很高（比如在 980 到 1020 之间波动）。因为 $R(\tau)$ 全是正的大数，导致无论走得好还是走得差，所有动作的概率都在被推高，训练极其震荡、难以收敛。

## 传统Reinforce算法的弊端
- 方差大
- 必须要等轨迹结束才能更新，轨迹不结束就不能更新，
	- 要求任务有限步
##  基于 Q 值的 Actor-Critic 算法

- **Critic 为价值网络**
    - 学习 Q 值，逼近真实的 $Q(s, a) = \mathbb{E}\big[ r_{t+1} + \gamma r_{t+2} + \gamma^2 r_{t+3} + \dots \big]$
### 对照

| **维度**   | **θ 网络（Actor 策略网络）**     | **w 网络（Critic 价值网络）**    |
| -------- | ------------------------ | ------------------------ |
| **角色比喻** | 参加考试的学生（写答案）             | 阅卷打分的老师（看答案给分数）          |
| **网络输入** | 只有状态 $s$（如关节角度、角速度）      | 状态 $s$ + 动作 $a$（如状态加上力矩） |
| **网络输出** | 动作本身的数值或概率分布（直接控电机）      | 一个单一的打分数字（Q值预测）          |
| **更新目标** | 听从 $w$ 的指导，输出能拿更高 Q 值的动作 | 听从真实奖励 $r$ 的指导，把打分预测得更准  |

### W网络定义与更新方式

- **它不是在网络内部去“走一遍最优路径并乘上打分”**：网络前向推理只是一堆矩阵乘法和激活函数，单次前向只需要几毫秒，它不可能在内部动态模拟一遍物理环境去求和。
    
- **它是把长期的期望回报“记忆/压缩”进了网络权重 $w$ 中**：
    
    - 在训练过程中，通过大量的时序差分误差 $\delta_t = r_t + \gamma V_w(s_{t+1}) - V_w(s_t)$；
        
    - 不断把真实的物理奖励 $r_t$ 和未来的折价因子 $\gamma$ 融入权重 $w$；
        
    - 最终让 $w$ 网络学会一个直接的映射函数：**只要看到姿态 $s$，就能瞬间报出“从这里出发、按当前策略走到底的期望累加折现分”**。

--- start-multi-column: ID_353a
```column-settings
Number of Columns: 2
Largest Column: standard
border: off
```

**Q-learning**
    - **TD target**:  $$r_t + \gamma \cdot \max_{a'} Q(s_{t+1}, a'; w)$$
    - **TD error**:  $$r_t + \gamma \cdot \max_{a'} Q(s_{t+1}, a'; w) - Q(s_t, a_t; w)$$

--- column-break ---

- **基于 Q 值的 AC（Actor-Critic）**
    - **TD target**:  $$r_t + \gamma \cdot q(s_{t+1}, a_{t+1}; w)$$
    - **TD error**:$$r_t + \gamma \cdot q(s_{t+1}, a_{t+1}; w) - Q(s_t, a_t; w)$$

--- end-multi-column
### 使用w网络
- **第一步（$\theta$ 出动作）**：将物理环境给出的下一个状态 $s_{t+1}$ 喂给 **$\theta$ 策略网络（Actor）**，得到下一步要采取的动作 $a_{t+1}$；
    
- **第二步（$w$ 给出评分）**：把 $(s_{t+1}, a_{t+1})$ 共同喂给 **$w$ 价值网络（Critic）**，由 $w$ 输出预测值 $q(s_{t+1}, a_{t+1}; w)$。

### w网络的更新
- 使用[[D-Q-learning#Fixed-Q-targets 策略]] 
- 使用Eval Critic $w$、Target Critic $w^-$ 两个网络
#### 1. 构造回归的目标值（TD Target）

如板书所示，这一步环境给出了真实物理奖励 $r_t$，下一动作 $a_{t+1}$ 带来了预估价值。组合起来的“阶段性标准答案”为：

$$y_t = r_t + \gamma \cdot q(s_{t+1}, a_{t+1}; w)$$

_(注意：在深度学习框架中，计算 $y_t$ 时需要截断梯度，即使用 `.detach()`，不让梯度回传到目标端)_

#### 2. 计算预测偏差（TD Error）

当前 Critic 网络对旧动作的估计值是 $Q(s_t, a_t; w)$，两者之间的差距就是 TD error：

$$\delta_t = y_t - Q(s_t, a_t; w) = \Big( r_t + \gamma \cdot q(s_{t+1}, a_{t+1}; w) \Big) - Q(s_t, a_t; w)$$

#### 3. 构造 Critic 损失函数并梯度下降

让预测值尽量逼近真实目标值，直接采用**均方误差（MSE Loss）**：

$$\mathcal{L}(w) = \frac{1}{2} \delta_t^2 = \frac{1}{2} \Big( y_t - Q(s_t, a_t; w) \Big)^2$$

对参数 $w$ 求导，得到的更新梯度为：

$$\nabla_w \mathcal{L}(w) = - \delta_t \cdot \nabla_w Q(s_t, a_t; w)$$

通过梯度下降（SGD / Adam）更新参数 $w$：

$$w \leftarrow w + \alpha_w \cdot \delta_t \nabla_w Q(s_t, a_t; w)$$




**软更新（Soft / Polyak Update）**：每步平滑微调（$w^- \leftarrow \tau w + (1-\tau)w^-$，$\tau \approx 0.005$）


 
##  Advantage Actor-Critic 算法

- **A2C **
### 优势函数（Advantage）：$$A(s_t, a_t) = Q(s_t, a_t) - V(s_t)$$
####  参数解释
- **$Q(s_t, a_t)$**：在状态 $s_t$ 选了特定动作 $a_t$ 后，预计未来能拿多少分。 $$Q(s_t, a_t) = \mathbb{E}[G_t \mid s_t, a_t]$$$$G_t = R_{t+1} + \gamma R_{t+2} + \dots = \sum_{k=0}^{\infty} \gamma^k R_{t+k+1}$$
	1.  使用折扣因子γ，大幅削弱未来时刻扣分带来的影响
		- 设采样频率是 50Hz，9 秒后相当于过了 $k \approx 450$ 步，$\gamma^{450} = 0.99^{450} \approx 0.0108$ 传到 $t=0$ 时已经被衰减了近 **100 倍**
	2. 强化学习为了彻底切断远期未来对起手动作的污染，直接引入了[[#Actor-Critic 架构]]  
		 - 在单步 Actor-Critic（TD 误差）中，第 0 秒的优势函数是这样算的：$$\hat{A}_0 = \underbrace{r_1}_{\text{只看执行第0步之后，第1秒拿到的即时反馈}} + \underbrace{\gamma V_w(s_1)}_{\text{Critic 预估处于状态 } s_1 \text{ 时的潜力}} - \underbrace{V_w(s_0)}_{\text{起点的基准线}}$$
		 - 用 $V(s_1)$ 替代了漫长的未知未来，完全没有后面的 $r_2, r_3, \dots, r_{10}$ 影响
    
- **$V(s_t)$**：在状态 $s_t$ 下，按平均水准预计能拿多少分（基线 Baseline，通常由 [[#Actor-Critic 架构]]网络估出）。

- **二者做差 $\hat{A}_t$**：

	- 如果 $\hat{A}_t > 0$：说明在这一步做出的决定**比平均水平要好**，反向传播**推高**这个动作的概率。
    
	- 如果 $\hat{A}_t < 0$：说明在这一步的操作**拉垮了（比平均预期差）**，反向传播**压低**这个动作的概率。

## 新的θ更新公式
#### 代理损失 $L^{PG}(\theta)$：

$$L^{PG}(\theta) = \hat{\mathbb{E}}_t \left[ \log \pi_\theta(a_t \mid s_t) \hat{A}_t \right]$$
#### 为什么不直接用 $J(\theta)$，非要造出 $L^{PG}(\theta)$ 这个符号？

1. **迎合自动求导（AutoGrad）**： $J(\theta)$ 内部包裹着环境交互的黑箱，PyTorch 的计算图根本没法直接对 $J(\theta)$ 调用 `.backward()`。而构造出 $L^{PG}(\theta)$ 后，`torch.log(prob) * advantage` 是一条清晰张量计算链，框架一行命令就能自动反向传播。
    
2. **为 PPO 等高级算法做铺垫**： 在后续深入到 PPO（近端策略优化）时，算法会直接拿这个 $L^{PG}(\theta)$ 去改写截断边界，变成带有概率比值的截断目标函数 $L^{CLIP}(\theta)$。把它命名为 $L^{PG}$，就是为了明确标明“这是原始策略梯度版本的损失函数”。
## 
# 新的θ更新公式
### 代理损失 $L^{PG}(\theta)$：

$$L^{PG}(\theta) = \hat{\mathbb{E}}_t \left[ \log \pi_\theta(a_t \mid s_t) \hat{A}_t \right]$$
### 为什么不直接用 $J(\theta)$，非要造出 $L^{PG}(\theta)$ 这个符号？

1. **迎合自动求导（AutoGrad）**： $J(\theta)$ 内部包裹着环境交互的黑箱，PyTorch 的计算图根本没法直接对 $J(\theta)$ 调用 `.backward()`。而构造出 $L^{PG}(\theta)$ 后，`torch.log(prob) * advantage` 是一条清晰张量计算链，框架一行命令就能自动反向传播。
    
2. **为 PPO 等高级算法做铺垫**： 在后续深入到 PPO（近端策略优化）时，算法会直接拿这个 $L^{PG}(\theta)$ 去改写截断边界，变成带有概率比值的截断目标函数 $L^{CLIP}(\theta)$。把它命名为 $L^{PG}$，就是为了明确标明“这是原始策略梯度版本的损失函数”。
# W网络对θ网络更新的缺点
## 1. 优势估计本身是带有“局部性”的（Local Approximation）

- 在采集经验时，机械臂是用当时的旧策略 $\pi_{\theta_{\text{old}}}$ 跑出了一批数据。
    
- Critic 价值网络所拟合出来的状态价值 $V(s)$，以及由此算出的优势 $\hat{A} = r + \gamma V(s') - V(s)$，**仅仅在旧策略经常访问的状态空间（State Space）附近才是相对可信的**。
    
- 一旦你在这一批数据上连续多次跑梯度下降（Keep running gradient descent on a single batch），网络参数更新过猛，新策略 $\pi_\theta$ 就会产生极大的漂移。
    

## 2. 致命的“外推谬误”与过高估计（Extrapolation Error）

- 当新策略 $\pi_\theta$ 漂移到了旧数据从未探索过的极端动作区域（Out-of-Distribution / OOD），Critic 价值网络就必须对这个陌生动作进行**外推（Extrapolation）**。
    
- 深度神经网络对未见过的数据分布没有任何先验保证，它常常会因为随机权重扰动，给一个极其荒谬、甚至危险的动作（例如关节超限或剧烈震荡）打出**虚假的奇高估值**。
    
- 结果就是：**优势函数给出了一个彻头彻尾的错误正向信号 $\hat{A} \gg 0$**。
    

## 3. 为什么是“毁灭性（Destroy your policy）”而不是普通退步？

在传统有监督学习中，如果一个 Batch 梯度走错了，下一个 Batch 的标准真实标签（Ground Truth）还能把参数再拉回来。

但在强化学习中，**下一批数据是由新策略自己去环境里探索采样的（Data Collection is Policy-Dependent）**：

1. **策略变坏**：被错误的极大优势值误导后，策略网络把概率狠狠押在了一个糟糕的动作上。
    
2. **数据塌缩**：下一个回合机械臂开始执行这个坏策略，结果一开局就碰撞停机或失稳脱轨，采集回来的全是全负分、毫无信息量的极度崩塌数据。
    
3. **彻底失明**：Critic 网络再也看不到高质量的成功轨迹，价值评估全面崩溃，导致整个策略在悬崖底下越摔越深，**完全无法自我恢复**。
## 4. 对W网络影响不大
- **$w$ 网络只是一个被动的拟合器**：

	- $w$ 算出的优势函数（Advantage）本身就是带有随机噪声的“半吊子估计”。
    
	- 哪怕 $w$ 的估计暂时有偏差，只要智能体动作没有发生剧烈漂移，采集回来的新数据依然在正常轨道上，后序几轮回归就能把 $w$ 的偏差纠正回来。
# TRPO（置信区间策略优化）
## **“策略性能相对差公式”（Relative Policy Performance Identity）**
## 打分函数η（θ） 
 - 不同θ得出的η不同，通过找到最高的η得出最优参数θ$$\eta(\pi') = \underbrace{\eta(\pi)}_{\substack{\text{当前曲线值} \\ \text{(旧策略)}}} + \sum_{s} \underbrace{\rho_{\pi'}(s)}_{\substack{\text{折扣状态访问分布} \\ \text{(状态转移加权)}}} \sum_{a} \underbrace{\pi'(a \mid s)}_{\substack{\text{新策略动作分布} \\ \text{(新策略)}}} \overbrace{A_\pi(s, a)}^{\substack{\text{当前动作优势} \\ \text{(旧策略基准)}}}$$
###  $\rho_{\pi'}(s)$   **“折扣状态访问分布”
**（Discounted State Visitation Distribution）**。
 $$\rho_{\pi'}(s) = \sum_{t=0}^\infty \gamma^t P(s_t = s \mid \pi')$$它的含义是：智能体如果按照**新策略 $\pi'$** 一直在环境里跑，未来**访问到特定状态 $s$ 的总频次（或出现概率）**。
    
- **为什么只和 $s$ 有关？**
	    
	$\rho$ 负责回答的是：“**不管你接下来做什么动作，你的身体究竟有多大概率会路过/身处状态 $s$？**” 它只统计状态本身的出现密度。
    
	- **作用**：充当**空间加权系数**。机械臂可能有一百万个姿态，但大部分姿态根本遇不到；$\rho$ 确保算法只去重点优化那些高频路过的关键状态，不把精力浪费在永远不会发生的状态上。
- 为什么使用老的ρ
	- 一个ρ需要在环境里跑出来，不跑不知道
#####  TRPO 的核心近似（打破死结）：
- Kakade 和 Schulman 发现：如果新策略 $\pi'$ 和老策略 $\pi$ 的差距非常小（加了约束条件），那么**新策略的状态分布 $\rho_{\pi'}(s)$ 就可以直接用老策略采集到的状态分布 $\rho_\pi(s)$ 来近似替代**！$$\rho_{\pi'}(s) \approx \rho_\pi(s)$$
	- 这一步近似，让算法可以直接**拿老策略已经收集好的经验池数据**，去优化新策略的目标函数，这也是 PPO/TRPO 能成立的理论基石。
### 更新部分的含义：
    
    $$\eta(\pi') - \eta(\pi) = \sum_s \rho_{\pi'}(s) \sum_a \pi'(a \mid s) A_\pi(s, a)$$
    
- 这个公式的意思是：**新策略的总收益，等于老策略的总收益，加上新策略在==🔴老W网络==下带来的超额累加净优势！**
    

## ==🟢更新策略==
![[Attachments/Pasted image 20261107140433.png|720]]
    
- 我们不需要去求出某个固定值，我们的目标是**找一组新参数 $\theta'$，让这个等式右边算出来的那个标量值越大越好（寻找最大值）**。
### 策略单调改进的理论下界（Theoretical Lower Bound）**：$$L_{\theta_{\text{old}}}(\theta) - C D_{\text{KL}}^{\max}(\theta_{\text{old}}, \theta)$$
#### 参数解释
- $L_{\theta_{\text{old}}}(\theta)$（本地代理回报）

	- 就是上一张板书里用老策略采集的数据估算出来的期望收益项：$$L_{\theta_{\text{old}}}(\theta) = \eta(\pi') = \eta(\theta_{\text{old}}) + \sum_s \rho_{\theta_{\text{old}}}(s) \sum_a \pi_\theta(a \mid s) A_{\theta_{\text{old}}}(s, a)$$
	- 
	- **物理含义**：如果我们把网络参数更新成 $\theta$，按照目前手头的经验池评估，新策略**看起来**能拿到的总分。
	    
	- **缺点**：这个估计只有在 $\theta$ 离 $\theta_{\text{old}}$ 极近时才准；一旦 $\theta$ 走远，这个分数就会严重虚高
		- 因为使用了老的ρ[[#TRPO 的核心近似（打破死结）：]]
		- 外推幻觉[[#2. 致命的“外推谬误”与过高估计（Extrapolation Error）]]

- **$D_{\text{KL}}^{\max}(\theta_{\text{old}}, \theta)$（新旧策略的最大 KL 散度）**$$D_{\text{KL}}^{\max}(\theta_{\text{old}}, \theta) = \max_s D_{\text{KL}}\big(\pi_{\theta_{\text{old}}}(\cdot \mid s) \;\parallel\; \pi_\theta(\cdot \mid s)\big) = \max_s \sum_{a \in \mathcal{A}} \pi_{\theta_{\text{old}}}(a \mid s) \ln \left( \frac{\pi_{\theta_{\text{old}}}(a \mid s)}{\pi_\theta(a \mid s)} \right)$$
	- **物理含义**：在对应维度上进行ln相减
		- 遍历所有状态，抓出新旧策略动作概率分布发生**最大偏移**的那一点。
	    
		- 它是一把纯粹的“距离标尺”，用来衡量新参数 $\theta$ 相比老参数 $\theta_{\text{old}}$ 到底**变异了多远**。
	- #### 参数解释
		- 点（$\cdot$）
	
			- **它代表“动作参数占位符（Placeholder）”**。
	    
			- 如果写 $\pi(a \mid s)$，通常表示“在状态 $s$ 下，执行某个具体动作 $a$ 对应的标量概率值（一个实数）”。
	    
			- 但 KL 散度比较的不是两个单独的数值，而是**两个完整的概率分布（两条概率密度曲线 / 整个概率柱状图）**。
	    
			- 所以数学家写一个居中的点 $\pi(\cdot \mid s)$，意思就是：**状态 $s$ 已经确定固定了，这里的“点”把所有可能执行的动作 $a \in \mathcal{A}$ 全部遍历占位**。它指代的是“当前状态 $s$ 下的整个动作分布”。
		- $\max_s$ 
			- 用最严格的放大镜扫描所有出现过的状态，抓出新旧策略偏离最夸张的那一个极端状态下的 KL 散度值**。只要最极端的那个点都被惩罚项按住了，就能确保在所有状态下新策略都不会失控出格。
		- $\parallel$ 
			- 相对于（with respect to / relative to）
			- 读作：_“在状态 $s$ 下，新策略 $\pi_\theta$相对于老策略$\pi_{\theta_{\text{old}}}$的 KL 散度（相对熵）”_。
			-  KL 散度是[[#严格不对称]]的

- **C（惩罚系数）**
	- 在 Schulman 的推导中，$C = \frac{4\epsilon\gamma}{(1-\gamma)^2}$（其中 $\epsilon = \max_{s, a} \vert{}A(s, a)\vert{}$ 为最大可能优势，$\gamma$ 为折扣因子）。
	- 板书专门标出“常数”，是因为在对参数$\theta$求解优化时，这个系数与参数 $\theta$无关，属于一个固定的超参数权重。

- **减号在这里起到了“安全气囊 / 刹车片”的作用：**

	- 如果新策略 $\theta$ 只是在老策略身边**微调**：
	    
	    - $D_{\text{KL}}^{\max}$ 接近 0，惩罚项很小；
	        
	    - 整个式子的值由 $L(\theta)$ 主导，算法放心地顺着优势梯度往上走。
        
	- 如果新策略 $\theta$ 步子迈得太大，**妄图一口气跑到未知区域去捞高分**：
	    
	    - 哪怕 $L(\theta)$ 预测的分数看似极高，但后半项的 $D_{\text{KL}}^{\max}$ 会瞬间暴增；
	        
	    - 带有大负号的惩罚项 $- C D_{\text{KL}}^{\max}$ 会直接把整个式子的值拉崩砸穿。

#### 空间展开式
当参数 $\theta$ 距离 $\theta_{\text{old}}$ 很近时，我们在起点 $\theta_{\text{old}}$ 处对这两个函数做[[#高维空间的泰勒展开]]：

- **前半部分 $L(\theta)$（一阶展开）**：
    $$L_{\theta_{\text{old}}}(\theta) \approx L(\theta_{\text{old}}) + g^T (\theta - \theta_{\text{old}})$$
    
    这里的 $g = \nabla_\theta L$ 是策略梯度向量。这是一张**倾斜的高维超平面（一次线性函数）**。
    
- **后半部分 KL 散度（二阶展开）**：
    
    $$D_{\text{KL}}(\theta_{\text{old}} \parallel \theta) \approx \frac{1}{2} (\theta - \theta_{\text{old}})^T H (\theta - \theta_{\text{old}})$$
    
    因为新旧参数相同时散度为 0（一阶导为 0），展开的第一项就是二阶项。$H$ 是 Hessian 矩阵（费雪信息矩阵 FIM）。这是一个**开口向上的高维超椭球碗（严格正定的二次型）**。
    

两者拼在一起：

$$f(\Delta \theta) \approx \text{常数} + \underbrace{g^T \Delta \theta}_{\text{一次倾斜平面}} - \frac{C}{2} \underbrace{\Delta \theta^T H \Delta \theta}_{\text{二次高维碗}}$$

由于二次项前面带负号，这个函数在超空间里变成了一个**开口向下倒扣的高维抛物面（像一个高维的大锅盖）**。
#### 找到顶点，需要遍历吗？

初中数学里求一元二次方程 $y = -a x^2 + b x$ 的最高点，需要去把周围所有的 $x$ 都算一遍吗？**不需要，直接对 $x$ 求导令导数等于 0，顶点就是 $x = \frac{b}{2a}$。**

高维空间完全是一模一样的道理！对整个向量 $\Delta \theta$ 求导并令其等于零向量：

$$\nabla_{\Delta \theta} f = g - C H \Delta \theta = 0$$

$$\Delta \theta^* = \frac{1}{C} H^{-1} g$$

**一步矩阵代数运算，直接算出了那个唯一最高点的坐标向量 $\Delta \theta^*$！** 算法根本不需要、也从来不会去探索周边的超曲面长什么样，完全是通过一阶导向量 $g$ 和二阶曲率矩阵 $H$ 的代数方程直接命中了顶峰。



### 下界作用（单调不减定理）

Kakade 和 Schulman 严格证明了一个不等式：

$$\eta(\theta) \ge L_{\theta_{\text{old}}}(\theta) - C D_{\text{KL}}^{\max}(\theta_{\text{old}}, \theta)$$

- 左边 $\eta(\theta)$ 是新策略在**真实物理世界**里跑出来的真实成绩；
    
- 右边是图里的**理论下界**。

这就给策略更新提供了一张**绝对保底的兜底凭证**： 只要我们能在数学上找到一个 $\theta$，让**右边的下界比老策略的基准线 $\eta(\theta_{\text{old}})$ 更高**，那么真实世界里的回报 $\eta(\theta)$ 就**必定单调递增，绝不退步**！



# PPO（近端策略优化）



# 知识点
## 严格不对称
在普通几何距离中，从北京到上海的距离，必然等于从上海到北京的距离（$d(A, B) = d(B, A)$），这叫**对称性**。

但 KL 散度不是几何距离，调换前后位置，算出来的数值往往天差地别：

$$D_{\mathrm{KL}}(P \parallel Q) \neq D_{\mathrm{KL}}(Q \parallel P)$$

### 1. 用一组具体数字直接看（数值为什么不同）

假设有两个针对动作的概率分布 $P$ 和 $Q$（比如一个二分类动作）：

- **老策略 $P$**：$[0.9, \; 0.1]$
    
- **新策略 $Q$**：$[0.5, \; 0.5]$
    

我们分别代入公式算一下：

#### 正着算：$D_{\mathrm{KL}}(P \parallel Q)$（以 $P$ 为基准）

$$D_{\mathrm{KL}}(P \parallel Q) = 0.9 \ln\left(\frac{0.9}{0.5}\right) + 0.1 \ln\left(\frac{0.1}{0.5}\right)$$

$$= 0.9 \times 0.5878 + 0.1 \times (-1.6094) \approx \mathbf{0.368}$$

#### 反着算：$D_{\mathrm{KL}}(Q \parallel P)$（以 $Q$ 为基准）

$$D_{\mathrm{KL}}(Q \parallel P) = 0.5 \ln\left(\frac{0.5}{0.9}\right) + 0.5 \ln\left(\frac{0.5}{0.1}\right)$$

$$= 0.5 \times (-0.5878) + 0.5 \times (1.6094) \approx \mathbf{0.511}$$

**$0.368 \neq 0.511$**，两者的数值完全对不上！

### 2. 为什么数值会对不上？（核心机制差异）

看展开式里决定数值的两大推手：

$$D_{\mathrm{KL}}(P \parallel Q) = \sum_{a} \underbrace{P(a)}_{\text{权重（以谁为准）}} \cdot \ln \left( \frac{P(a)}{Q(a)} \right)$$

1. **加权的人变了（期望主体不同）**：
    
    - $D_{\mathrm{KL}}(P \parallel Q)$ 是由 **$P(a)$ 站在裁判席上去加权**。$P$ 经常做哪个动作（$P(a)$ 大），该动作的对数差就会被强烈放大；$P$ 根本不做的动作（$P(a) \approx 0$），哪怕 $Q$ 偏得再离谱，总和乘下来也是 0。
        
    - 一旦反过来写成 $D_{\mathrm{KL}}(Q \parallel P)$，裁判席就换成了 $Q(a)$，权重的侧重点彻底颠倒。
        
2. **对“0 概率”的容忍度完全相反（非对称爆炸）**：
    
    - 在 $D_{\mathrm{KL}}(P \parallel Q)$ 中：如果 $P(a) > 0$，而 $Q(a) \to 0$，分子除以分母就是 $\frac{P}{0} \to +\infty$，**散度直接爆炸到无穷大**；
        
    - 但反过来，如果某个动作 $P(a) = 0$，而 $Q(a) > 0$：这一项是 $0 \cdot \ln(0) = 0$，**对散度毫无惩罚，完全合法**！
        

### 3. 为什么 TRPO 必须写成 $D_{\mathrm{KL}}(\pi_{\text{old}} \parallel \pi_{\theta})$ 而绝不能倒过来？

在 TRPO / PPO 的更新中，这个前后位置有决定性的工程原因：

1. **手头只有老策略的数据（采样约束）**：
    
    $$D_{\mathrm{KL}}(\mathbf{\pi_{\text{old}}} \parallel \pi_\theta) = \mathbf{\mathbb{E}_{a \sim \pi_{\text{old}}}} \left[ \ln \frac{\pi_{\text{old}}(a \mid s)}{\pi_\theta(a \mid s)} \right]$$
    
    老策略 $\pi_{\text{old}}$ 放在双竖线左边，意味着这个期望是**对老策略采样的动作**求平均。因为我们刚用老策略在环境里跑了一堆轨迹，直接拿这批经验就能用蒙特卡洛均值把散度算出来；如果倒过来把新策略 $\pi_\theta$ 放在左边，我们就必须根据未知的网络去实时采样，当前这批数据就没法直接用了。
    
2. **安全防御方向不同（包容模式 vs 聚焦模式）**：
    
    - 把老策略放左边（前项）：要求新策略 $\pi_\theta$ 必须“覆盖”老策略的所有安全动作，**老策略走得通的路，新策略绝不能把概率突然砍成 0**，从而杜绝了策略崩溃。
        

这就是为什么数学家不用逗号，而必须用双竖线 $\parallel$——它在提醒你：**前后顺序既改变了计算出来的具体数值，也彻底改变了谁是采样基准、谁是被考核对象。**
### 高维空间的泰勒展开

- **一元标量函数**（$\theta \in \mathbb{R}$）：
    
    $$f(\theta) = f(\theta_{\text{old}}) + f'(\theta_{\text{old}})\Delta\theta + \frac{1}{2!}f''(\theta_{\text{old}})(\Delta\theta)^2 + \frac{1}{3!}f'''(\theta_{\text{old}})(\Delta\theta)^3 + \dots$$
    
- **多元向量函数**（$\boldsymbol{\theta} \in \mathbb{R}^n$）：
    
    $$f(\boldsymbol{\theta}) = f(\boldsymbol{\theta}_{\text{old}}) + \underbrace{\nabla f^T \Delta\boldsymbol{\theta}}_{\text{一阶项（梯度向量）}} + \underbrace{\frac{1}{2!}\Delta\boldsymbol{\theta}^T H \Delta\boldsymbol{\theta}}_{\text{二阶项（Hessian 矩阵二次型）}} + \underbrace{\frac{1}{3!}\mathcal{T}_3[\Delta\boldsymbol{\theta}, \Delta\boldsymbol{\theta}, \Delta\boldsymbol{\theta}]}_{\text{三阶项（3阶张量缩并）}} + \dots$$
    #### 参数解释
    - 导数 张量本身是存储偏导数的多维容器：

		- **0 阶张量（标量）**：原函数值 $f(\boldsymbol{\theta}_{\text{old}})$。
		- **1 阶张量（向量）**：梯度 $\boldsymbol{g} = \nabla f \in \mathbb{R}^n$，元素为 $g_i = \frac{\partial f}{\partial \theta_i}$。
		- **2 阶张量（矩阵）**：Hessian 矩阵 $H \in \mathbb{R}^{n \times n}$，元素为 $H_{ij} = \frac{\partial^2 f}{\partial \theta_i \partial \theta_j}$。
		- **3 阶张量（三维数组）**：$\mathcal{T}_3 \in \mathbb{R}^{n \times n \times n}$，元素为 $\mathcal{T}_{ijk} = \frac{\partial^3 f}{\partial \theta_i \partial \theta_j \partial \theta_k}$。
		- **高阶项求和（张量缩并）**：$$\frac{1}{3!} \sum_{i,j,k} \frac{\partial^3 f}{\partial \theta_i \partial \theta_j \partial \theta_k} \Delta\theta_i \Delta\theta_j \Delta\theta_k$$即把所有高阶偏导数权重与对应的步长分量逐一相乘后求和，输出一个标量。
	- 二次型 $\Delta\boldsymbol{\theta}^T H \Delta\boldsymbol{\theta}$

		1. 结构与维度该式**不是**矩阵乘转置，而是**向量-矩阵-向量**相乘的**二次型（Quadratic Form）**：$$\underbrace{\Delta\boldsymbol{\theta}^T}_{1 \times n} \times \underbrace{H}_{n \times n} \times \underbrace{\Delta\boldsymbol{\theta}}_{n \times 1} = \boldsymbol{1 \times 1 \text{（标量）}}$$
		2. 与标量版本的对应关系

			- 一元展开的二阶项是 $f''(\theta_{\text{old}}) \cdot (\Delta\theta)^2$。
    
			- 多元展开的二次型 $\Delta\boldsymbol{\theta}^T H \Delta\boldsymbol{\theta}$ 正是一元 $(\Delta\theta)^2$ 的高维推广。
    
			- **与模长平方的区别**：$\Delta\boldsymbol{\theta}^T \Delta\boldsymbol{\theta} = \Vert{}\Delta\boldsymbol{\theta}\Vert{}^2$ 仅是 $H = I$（单位阵）时的各向同性特例；而 $H$ 提供了各方向不同的**几何曲率加权**。