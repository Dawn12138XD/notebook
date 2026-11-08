# 机械臂末端坐标系 {B} 的标准 T 矩阵（n/o/a/p 记号，机器人经典！）

$$
T = \begin{pmatrix} 
n_x & o_x & a_x & p_x \\ 
n_y & o_y & a_y & p_y \\ 
n_z & o_z & a_z & p_z \\ 
0 & 0 & 0 & 1 
\end{pmatrix}
$$

## 4 列分别对应图上标注

* **第 1 列 $\boldsymbol{n}$**：normal（法向） $\rightarrow$ 末端坐标系 **X 轴**
* **第 2 列 $\boldsymbol{o}$**：orientation（朝向） $\rightarrow$ 末端坐标系 **Y 轴**
* **第 3 列 $\boldsymbol{a}$**：approach（接近方向） $\rightarrow$ 末端坐标系 **Z 轴**

> 👉 **姿态总结**：  
> 前 3 列合起来就是旋转矩阵：
> $$
> \boldsymbol{R} = \begin{bmatrix} \boldsymbol{n} & \boldsymbol{o} & \boldsymbol{a} \end{bmatrix}
> $$
> 代表末端的 **orientation（姿态）**。这三个向量两两正交、模长为 1，是一组标准正交基。

* **第 4 列 $\boldsymbol{p}$**：position（位置）  
  $$
  \boldsymbol{p} = [p_x, p_y, p_z]^T
  $$
  代表末端坐标系 $\{B\}$ 的原点在参考坐标系 $\{A\}$ 里的实际坐标。

* **底行固定**：  
  $$
  \begin{bmatrix} 0 & 0 & 0 & 1 \end{bmatrix}
  $$
  作为齐次变换矩阵的固定底行（用于保证矩阵可乘与平移计算）。

---

### 右边坐标系图对应关系
* $\boldsymbol{n}$：$X$ 轴（normal）
* $\boldsymbol{o}$：$Y$ 轴（orientation）
* $\boldsymbol{a}$：$Z$ 轴（approach）

---
### 关联概念（双链）
- [[齐次坐标]]
- [[旋转矩阵]]
- [[DH参数法]]