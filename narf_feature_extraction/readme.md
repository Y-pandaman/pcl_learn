NARF 全称 normal aligned radial feature(法线对齐的径向特征)。是一种3D特征点检测和描述算法。
## NARF 特征点提取
NARF 特征点提取算法的两个目标是：
- 提取出的特征点应位于表面稳定的区域(where the surface is stable)以确保对法线的估计具有较强的鲁邦性，同时在其邻域点的法线应具有明显的变化。
- 提取出的点应尽可能的位于物体的边缘处，从而使得在不同的视点下都能提取到大部分相同的特征点。

### 特性
- 能提取到 borders points 和其对应的 shadow points
- 提取到的特征点都是位于物体的外轮廓(边缘)上，更加具有稳定性和可重复性
- 对特征点Normal的估计只使用共面点，因而对Normal的估计更加稳定
- 所有操作都是基于2D的range image，计算量相比于直接操作点云要小

NARF(Normal Aligned Radial Feature)关键点是为了从深度图像中识别物体而提出的，对NARF关键点的提取过程有以下要求：

  - 提取的过程考虑边缘以及物体表面变化信息在内；
  - 在不同视角关键点可以被重复探测；
  - 关键点所在位置有足够的支持区域，可以计算描述子和进行唯一的估计法向量。

  其对应的探测步骤如下：

  - 遍历每个深度图像点，通过寻找在近邻区域有深度变化的位置进行边缘检测。
  - 遍历每个深度图像点，根据近邻区域的表面变化决定一测度表面变化的系数，及变化的主方向。
  - 根据step(2)找到的主方向计算兴趣点，表征该方向和其他方向的不同，以及该处表面的变化情况，即该点有多稳定。
  - 对兴趣值进行平滑滤波。
  - 进行无最大值压缩找到的最终关键点，即为NARF关键点。

