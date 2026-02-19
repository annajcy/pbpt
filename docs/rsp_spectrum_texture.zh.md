# RSPSpectrumTexture 技术文档

## 概述
`RSPSpectrumTexture` 是一种纹理实现，旨在将 RGB 到光谱的转换计算开销从运行时（渲染过程中）迁移到加载时（场景初始化期间）。

该纹理不再存储 RGB 值并在每次光线相交时进行查表，而是在纹素中预先存储 RGB Sigmoid Polynomial（RSP，RGB S 型多项式）系数。

## 数据结构
纹理内部使用 `MipMap<T, math::Vector<T, 3>>`。
向量的三个分量分别存储 `RGBSigmoidPolynomialNormalized` 的 `c0`、`c1`、`c2` 系数。

该结构与现有 MipMap 逻辑保持兼容，包括直接作用于系数的三线性过滤。由于多项式系数具有线性性，此插值方式是合法的。

## 双向转换语义
该类支持双向转换：
1.  **位图到 RSP（加载时）**：使用 `radiometry::lookup_srgb_to_rsp`（基于 PBRT 的优化方案）为给定线性 RGB 三元组寻找最优拟合的 sigmoid 多项式。
2.  **RSP 到位图（重建时）**：对多项式重新求值得到光谱，在 CIE D65 光源下计算 XYZ，再使用 sRGB 原色将 XYZ 转换回线性 RGB。这是一种近似重建。

## 误差评估
我们使用往返误差（RGB → RSP → RGB）来评估 RSP 表示的质量。

### 评估指标
*   **RMSE（均方根误差）**：RGB 通道的均方根误差。
*   **DeltaE76**：CIE L\*a\*b\* 色彩空间中的欧氏距离，提供感知误差度量。

### 观测基准
在测试渐变图像上：
*   RGB RMSE：约 0.028（线性空间 [0,1]）
*   平均 DeltaE76：约 6.9
*   最大 DeltaE76：约 33.3

这些误差来源于用 3 系数 sigmoid 多项式拟合任意光谱的数学局限性，以及在 64×64×64 查找表内部进行插值所引入的误差。尽管对部分颜色存在感知上的损失（DeltaE > 2.0），但光谱特性通常能得到很好的保留，足以满足以光谱交互为主要目标的基于物理渲染需求。

## 性能优势
通过预加载 RSP 系数：
*   消除了 `LambertianMaterial::compute_bsdf` 中的 `lookup_srgb_to_rsp` 调用（含关联的五维查表/插值开销）。
*   消除了每个着色点的 `create_srgb_albedo_spectrum` 开销。
*   内存占用与高精度（float）RGB 纹理相同（每纹素 3 个 float）。

## 集成说明
当遇到 `<texture type="bitmap">` 时，加载器会自动使用 `RSPSpectrumTexture`。若需退回旧版行为，目前需要手动修改 `scene_loader.hpp` 中的相关代码。
