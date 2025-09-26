# Physically Based Path Tracer (PBPT)

一个基于物理的光线追踪渲染器，使用现代 C++23 开发，支持光谱渲染和蒙特卡罗积分。

[完整文档](https://annajcy.github.io/pbpt_docs/)

## 项目结构

### 已实现模块

#### 数学模块 (`src/math/`)

- **基础数学类型**
  - `Vector<T, N>`: N维向量类，支持各种数学运算
  - `Point<T, N>`: 点类，基于向量实现
  - `Normal<T, N>`: 法向量类
  - `Matrix<T, M, N>`: 矩阵类，支持矩阵运算
  - `Quaternion<T>`: 四元数类，用于旋转表示
  - `Interval<T>`: 区间类，支持区间运算
  - `Tuple<T, N>`: 元组基类

- **几何数学**
  - `Homogeneous<T, N>`: 齐次坐标系统
  - `Polynomial<T>`: 多项式类和相关运算
  - `Function`: 数学函数工具集
  - `Octahedral`: 八面体映射工具

- **实用工具**
  - `format.hpp`: 格式化输出
  - `operator.hpp`: 运算符重载
  - `utils.hpp`: 数学工具函数
  - `type_alias.hpp`: 类型别名定义

#### 几何模块 (`src/geometry/`)
- **空间变换**
  - `Transform`: 3D变换矩阵和操作
  - `Frame`: 局部坐标系
  - `Ray`: 光线类，包含原点和方向

- **几何原语**
  - `Bounds<T, N>`: 轴对齐包围盒
  - `DirectionalCone`: 方向锥体
  - `Spherical`: 球坐标系工具

#### 核心渲染模块 (`src/core/`)
- **光谱和颜色**
  - `Spectrum`: 完整的光谱表示系统
    - `SampledSpectrum`: 采样光谱
    - `SampledWavelength`: 采样波长
    - `SampledPdf`: 概率密度函数
    - `SpectrumDistribution`: 光谱分布
      - `BlackBody`: 黑体辐射计算
      - `Constant`: 常量光谱
      - `Tabular`: 表格光谱
      - `RGB`: RGB光谱
        - `Albedo`: 反照率RGB
        - `Unbounded`: 无界RGB
        - `Illuminant`: 光源RGB
  - `Color`: 颜色与颜色空间
    - `ColorSpace`: 颜色空间转换
      - `sRGB`: 标准RGB空间
      - `XYZ`: CIE 1931 XYZ颜色空间
      - `DCI-P3`: DCI-P3颜色空间
      - `Rec2020`: Rec. 2020颜色空间
    - `Color`: 颜色类，支持多种颜色空间
      - `XYZColor`: XYZ颜色
      - `RGBColor`: RGB颜色
      - `LABColor`: CIE LAB颜色 
    - `RGBtoSpectrum`: RGB到光谱的转换
      - `RGBSigmoidPoly`: Sigmoid多项式
      - `Optimizer`: Newton优化器

- **辐射度量学**
  - `Radiometry`: 辐射度量学计算
  - `RadiometricIntegrals`: 辐射积分
  - `Interaction`: 表面交互

#### 积分器模块 (`src/integrator/`)
- **蒙特卡罗积分**
  - `Integrator<N>`: 数值积分器基类
  - `MonteCarloIntegrator1D`: 一维蒙特卡罗积分
  - `MonteCarloIntegratorND<N>`: N维蒙特卡罗积分
  - `MonteCarlo`: 蒙特卡罗采样工具

- **采样系统**
  - `Sampler<N>`: 采样器基类和实现
  - `Distribution<N>`: 概率分布
  - `RandomGenerator`: 随机数生成器

#### 工具模块 (`src/utils/`)
- `SpectrumLoader`: 光谱数据加载器

#### 渲染后端支持
- **图形API支持**
  - Vulkan 渲染后端
  - OpenGL 渲染后端
  - 可通过CMake配置选择

#### 相机系统 (`src/core/camera.hpp`)
- [ ] 透视相机
- [ ] 正交相机  
- [ ] 鱼眼相机
- [ ] 全景相机
- [ ] 相机运动模糊

#### 场景管理 (`src/core/scene.hpp`)
- [ ] 场景图结构
- [ ] 空间加速结构 (BVH, KD-Tree)
- [ ] 场景序列化/反序列化
- [ ] 动态场景支持

#### 几何形状 (`src/core/shape.hpp`)
- [ ] 基础形状完善
  - [ ] 球体
  - [ ] 平面
  - [ ] 立方体
  - [ ] 圆柱体
  - [ ] 圆锥体
- [ ] 网格支持
- [ ] 细分表面
- [ ] 程序化几何

