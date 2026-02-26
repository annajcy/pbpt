光谱渲染到 RGB 的核心流程
在 spectral rendering 中，从光谱辐射亮度到最终 RGB 像素值需要经过 CIE 色度学积分：

$$X = \frac{\int L(\lambda) \cdot \bar{x}(\lambda) , d\lambda}{\int \bar{y}(\lambda) , d\lambda}, \quad Y = \frac{\int L(\lambda) \cdot \bar{y}(\lambda) , d\lambda}{\int \bar{y}(\lambda) , d\lambda}, \quad Z = \frac{\int L(\lambda) \cdot \bar{z}(\lambda) , d\lambda}{\int \bar{y}(\lambda) , d\lambda}$$

分母 $\int \bar{y}(\lambda) , d\lambda$ 是 CIE 标准的 等能白归一化因子（约 106.856），它确保等能白光的 Y 值为 1.0。Mitsuba 3 的 scalar_rgb 模式就是用这个公式在场景加载时将 <spectrum> 标签转换为 RGB。

PBPT 原来的问题
PBPT 的 

PixelSensor
 使用 

project_emission
 函数，它做的是：

$$
\text{sensor-rgb}_c = \frac{\int \text{emission}(\lambda) \cdot \text{response}_c(\lambda) , d\lambda}{\int \text{illuminant}(\lambda) \cdot \bar{y}(\lambda) , d\lambda}
$$

这里的 

illuminant
 是 D65，所以分母是 $\int D65(\lambda) \cdot \bar{y}(\lambda) , d\lambda \approx 10567$。

原来的代码为了补偿这个大分母，在发射光谱上预乘了 D65：

$$\text{emission}_{\text{old}}(\lambda) = L(\lambda) \cdot D65(\lambda)$$

这样代入公式：

$$\text{sensor-rgb}_Y = \frac{\int L(\lambda) \cdot D65(\lambda) \cdot \bar{y}(\lambda) , d\lambda}{\int D65(\lambda) \cdot \bar{y}(\lambda) , d\lambda}$$

D65 在分子分母中大致抵消，但因为 D65 是非均匀光谱（蓝端偏高），加权积分并不能完美抵消，导致了 ~23% 的亮度偏差和色偏。

修复方案
第一步：移除发射光谱的 D65 调制
将 StandardEmissionSpectrum 从 PiecewiseLinear × D65 变为纯 PiecewiseLinear，让发射光谱直接就是绝对辐射亮度 $L(\lambda)$。

第二步：补偿 PixelSensor 的归一化差异
移除 D65 后，

project_emission
 计算的是：

$$\text{sensor-rgb}Y = \frac{\int L(\lambda) \cdot \bar{y}(\lambda) , d\lambda}{\underbrace{\int D65(\lambda) \cdot \bar{y}(\lambda) , d\lambda}{g_integral}}$$

而 Mitsuba 期望的是：

$$\text{XYZ}Y = \frac{\int L(\lambda) \cdot \bar{y}(\lambda) , d\lambda}{\underbrace{\int \bar{y}(\lambda) , d\lambda}{y_integral}}$$

两者的比值就是：

$$\frac{\text{Mitsuba}}{\text{PBPT}} = \frac{g_integral}{y_integral} = \frac{\int D65 \cdot \bar{y}}{\int \bar{y}} \approx \frac{10567}{87.5} \approx 120.7$$

所以我在 

PixelSensor
 构造函数中计算 g_integral / y_integral 并乘到 image_ratio 上。最终公式变成：

$$\text{output} = \frac{\int L \cdot \text{CMF}}{g_integral} \times \frac{g_integral}{y_integral} = \frac{\int L \cdot \text{CMF}}{y_integral}$$

这恰好等于 Mitsuba 的 CIE 标准公式。

为什么还有 ~18% 的偏差？
剩余偏差来自 PBPT 和 Mitsuba 在 XYZ → sRGB 转换路径上的差异：

Mitsuba：直接用 CIE XYZ → sRGB 的 3×3 矩阵
PBPT：经过一个 swatch 反射率校准矩阵 M（通过最小二乘法拟合 24 块 Macbeth 色卡样本），再转 XYZ，再转 sRGB
这个校准矩阵 M 引入了一个系统性的缩放差异。这属于 PBPT 色彩管线的设计选择，需要后续单独处理。