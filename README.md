# 2025重庆大学千里战队能量机关自瞄 rm_rune

## 概述

`rm_rune`，用于检测和预测 RoboMaster 能量机关的运动。它利用图像处理、基于 ONNX 模型的深度学习目标检测以及运动预测算法来实时跟踪能量机关并预测其未来位置。

该软件包主要包括以下功能：
- **图像采集与预处理**：订阅摄像头图像流，并使用 `cv_bridge` 进行格式转换。
- **能量机关检测**：使用 YOLOv8-pose ONNX 模型（通过 OpenVINO 推理）检测图像中的能量机关扇叶及其关键点。
- **轮廓与目标信息处理**：提取和处理检测到的能量机关轮廓信息。
- **运动预测**：基于历史观测数据，使用 Ceres Solver 拟合能量机关的旋转模型，并预测未来的旋转角度和位置。
- **3D 位姿估计**：通过 `cv::solvePnP` 计算能量机关在相机坐标系下的 3D 位姿。
- **坐标变换**：使用 TF2 将预测点位从相机坐标系转换到 `odom` 坐标系。
- **可视化**：发布处理后的图像，包含检测框、关键点和预测点，方便调试。

## ROS 主题

#### 订阅的主题

-   **`/image_raw`** ([sensor_msgs/msg/Image](https://docs.ros2.org/humble/api/sensor_msgs/msg/Image.html))
    -   输入的原始图像流。由 [`qianli_rm_rune::RuneNode`](src/rm_rune/src/rm_rune_node.cpp) 订阅。
-   **`/camera_info`** ([sensor_msgs/msg/CameraInfo](https://docs.ros2.org/humble/api/sensor_msgs/msg/CameraInfo.html))
    -   相机的内参信息。节点会订阅此主题一次以获取相机参数 `camera_matrix_`。由 [`qianli_rm_rune::RuneNode`](src/rm_rune/src/rm_rune_node.cpp) 订阅。

#### 发布的主题

-   **`/rune/prediction`** ([geometry_msgs/msg/PointStamped](https://docs.ros2.org/humble/api/geometry_msgs/msg/PointStamped.html))
    -   预测的能量机关打击点在 `odom` 坐标系下的 3D 坐标。由 [`qianli_rm_rune::RuneNode`](src/rm_rune/src/rm_rune_node.cpp) 发布。
-   **`rune/result_image`** ([sensor_msgs/msg/Image](https://docs.ros2.org/humble/api/sensor_msgs/msg/Image.html))
    -   （通过 `image_transport` 发布）处理后的图像，包含检测框、关键点和预测点，用于调试。由 [`qianli_rm_rune::RuneNode`](src/rm_rune/src/rm_rune_node.cpp) 发布。

## 配置

主要的配置和初始化发生在 [`src/rm_rune/src/rm_rune_node.cpp`](src/rm_rune/src/rm_rune_node.cpp) 的构造函数 [`qianli_rm_rune::RuneNode::RuneNode`](src/rm_rune/src/rm_rune_node.cpp) 中。

-   **ONNX 模型路径与推理参数**:
    当前模型路径、输入图像尺寸、置信度阈值和NMS阈值硬编码在 [`src/rm_rune/src/rm_rune_node.cpp`](src/rm_rune/src/rm_rune_node.cpp) 的构造函数中，用于初始化 [`yolo::Inference`](src/rm_rune/src/openvino_detect.cpp) 对象：
    ````cpp
    // filepath: src/rm_rune/src/rm_rune_node.cpp
    // ...existing code...
    inference(  // ✅ 在初始化列表中构造对象
        "/home/fyk/fyk/mechax_cv_trajectroy_rune_openvino/src/rm_rune/model/buff640_openvino_opset13/best.xml", // 模型路径
        cv::Size(480, 480),          // 输入尺寸
        0.5f,   // 置信度阈值
        0.5f   // NMS阈值
    ) 
    // ...existing code...
    ````
    您需要根据实际模型位置修改此路径。建议将这些参数作为 ROS 参数进行配置。

-   **相机参数**:
    相机内参通过订阅 `/camera_info` 主题自动获取，并存储在 `camera_matrix_` 成员变量中。

-   **运动预测参数**:
    预测相关的参数（如击打延迟 `hit_delay_sec`，重拟合延迟 `refit_delay_sec`）定义在 [`qianli_rm_rune::Configuration`](src/rm_rune/src/configuration.cpp) 类中，并通过 `cfg_` 成员被 [`qianli_rm_rune::Prediction`](src/rm_rune/src/prediction.cpp) 类使用。这些参数可以通过修改 [`src/rm_rune/src/configuration.cpp`](src/rm_rune/src/configuration.cpp) 中的默认值进行调整。

## 主要组件

-   **`RuneNode`** ([`src/rm_rune/include/rm_rune/rm_rune_node.hpp`](src/rm_rune/include/rm_rune/rm_rune_node.hpp), [`src/rm_rune/src/rm_rune_node.cpp`](src/rm_rune/src/rm_rune_node.cpp)):
    主要的 ROS 节点类。负责订阅图像和相机信息，协调图像处理流程（调用检测和预测模块），进行3D位姿估计和坐标变换，并发布最终的预测结果和调试图像。节点内也包含帧率计算逻辑。
-   **`Inference` (YOLO)** ([`src/rm_rune/include/rm_rune/openvino_detect.hpp`](src/rm_rune/include/rm_rune/openvino_detect.hpp), [`src/rm_rune/src/openvino_detect.cpp`](src/rm_rune/src/openvino_detect.cpp)):
    封装了 OpenVINO 的 YOLOv8-pose 模型推理逻辑。该类负责加载模型，执行图像预处理（如缩放、颜色空间转换、归一化），进行推理，以及后处理（包括非极大值抑制NMS）。最终输出检测到的扇叶轮廓关键点列表 (`std::vector<std::vector<cv::Point2f>> contours`)。
-   **`Prediction`** ([`src/rm_rune/include/rm_rune/prediction.hpp`](src/rm_rune/include/rm_rune/prediction.hpp), [`src/rm_rune/src/prediction.cpp`](src/rm_rune/src/prediction.cpp)):
    实现了能量机关运动预测算法。它维护历史观测到的扇叶角度和对应的时间戳。当数据足够且满足重拟合条件时，调用Ceres Solver拟合旋转模型 `angle(t) = k*t + b + a*sin(omega*t + phi)`。
-   **`ContourInfo`** ([`src/rm_rune/include/rm_rune/detect.hpp`](src/rm_rune/include/rm_rune/detect.hpp), [`src/rm_rune/src/detect.cpp`](src/rm_rune/src/detect.cpp)):
    存储和处理单个扇叶轮廓的信息。根据模型输出的关键点数据，提取扇叶中心、R点（圆心）、扇叶索引（标签）和置信度。
-   **`Blade`** ([`src/rm_rune/include/rm_rune/blade.hpp`](src/rm_rune/include/rm_rune/blade.hpp), [`src/rm_rune/src/blade.cpp`](src/rm_rune/src/blade.cpp)):
    封装对单个扇叶目标的进一步处理。核心功能是计算从能量机关中心（R点）指向当前扇叶中心的二维向量，用于运动预测。
-   **`PowerRune`** ([`src/rm_rune/include/rm_rune/power_rune.hpp`](src/rm_rune/include/rm_rune/power_rune.hpp), [`src/rm_rune/src/power_rune.cpp`](src/rm_rune/src/power_rune.cpp)):
    包含与能量机关相关的辅助函数，例如根据给定的角度（弧度）将一个二维向量进行旋转。
-   **`Configuration`** ([`src/rm_rune/include/rm_rune/configuration.hpp`](src/rm_rune/include/rm_rune/configuration.hpp), [`src/rm_rune/src/configuration.cpp`](src/rm_rune/src/configuration.cpp)):
    存储各种可调参数，例如运动预测中的关键参数如 `hit_delay_sec` (击打延迟) 和 `refit_delay_sec` (重拟合延迟)。

## 详细算法流程

能量机关识别与预测算法的核心流程可以分解为以下几个主要步骤：

### 1. 图像采集与目标检测 (YOLOv8-Pose via OpenVINO)

-   **图像输入**: 算法的起点是来自相机节点的原始图像流 (`/image_raw`)。
-   **模型初始化 (`yolo::Inference::InitializeModel`)**:
    -   在 `RuneNode` 初始化时，会创建一个 `yolo::Inference` 对象。
    -   构造函数中指定了 OpenVINO 模型的路径 (`.xml` 文件)、期望的输入图像尺寸 (例如 `480x480`)、置信度阈值和 NMS (非极大值抑制) 阈值。
    -   `InitializeModel` 函数负责加载 OpenVINO 模型。它使用 `ov::Core` 读取模型，并对模型进行预处理设置：
        -   输入张量被配置为接收 `U8` 类型的数据，布局为 `NHWC` (Batch, Height, Width, Channels)，颜色格式为 `BGR`。
        -   预处理步骤包括：将输入数据类型转换为 `FP32`，颜色空间从 `BGR` 转换为 `RGB`，并进行归一化 (除以255)。
        -   模型的输入布局最终设置为 `NCHW`。
        -   输出张量的数据类型设置为 `FP32`。
    -   模型随后被编译到目标硬件 (通过 `"AUTO"` 自动选择)，并创建一个推理请求 (`inference_request_`)。
-   **图像预处理 (`yolo::Inference::Preprocessing`)**:
    -   当 `RuneNode::rune_image_callback` 接收到新图像帧后，会调用 `inference.RunInference(rune_image)`。
    -   `RunInference` 首先调用 `Preprocessing`。
    -   原始图像帧 (`frame`) 被 `cv::resize` 到模型期望的输入尺寸 (`model_input_shape_`)。
    -   计算缩放因子 (`scale_factor_`)，用于后续将模型输出坐标还原到原始图像尺寸。
    -   预处理后的图像数据被封装成一个 `ov::Tensor` 对象，并设置为推理请求的输入张量。
-   **模型推理 (`inference_request_.infer()`)**:
    -   执行推理请求，模型对预处理后的图像进行前向传播。
-   **后处理 (`yolo::Inference::PostProcessing`)**:
    -   从 `inference_request_` 获取输出张量。
    -   YOLOv8-Pose 模型的输出包含了每个检测目标的边界框、类别、置信度以及关键点信息。
    -   代码会遍历原始检测结果：
        -   过滤掉置信度低于 `model_confidence_threshold_` 的检测。
        -   提取每个目标的类别ID、置信度、边界框坐标和关键点坐标。
        -   应用 NMS 来去除重叠度高的冗余检测框，基于 `model_NMS_threshold_`。
        -   使用 `scale_factor_` 将边界框和关键点坐标从模型输入尺寸映射回原始图像尺寸。
    -   最终，`PostProcessing` 函数会将有效的检测结果（主要是关键点）存储在 `inference.contours` 成员变量中。`contours` 是一个 `std::vector<std::vector<cv::Point2f>>` 类型。每个内部的 `std::vector<cv::Point2f>` 代表一个检测到的目标，其结构通常是：
        -   `contours[i][0].x`: 目标的类别索引。
        -   `contours[i][0].y`: 目标的置信度。
        -   `contours[i][1]` 到 `contours[i][k]`: (k-1)个关键点的坐标。

### 2. 目标信息提取与扇叶状态表示

-   **轮廓信息解析 (`ContourInfo`)**:
    -   在 `RuneNode::rune_image_callback` 中，选取最可信的检测结果。
    -   使用检测结果中的关键点数据创建一个 `ContourInfo` 对象。
        -   `index` (扇叶类别/状态) 来自 `contour[0].x`。
        -   `conf` (置信度) 来自 `contour[0].y`。
        -   `circle_center` (R点/能量机关旋转中心) 通常是预定义的关键点之一 (例如 `contour[3]`)。
        -   `center` (扇叶的几何中心) 通过取其他几个关键点 (例如 `contour[1]`, `contour[2]`, `contour[4]`, `contour[5]`) 的平均坐标计算得到。
-   **扇叶状态更新 (`Blade`)**:
    -   `RuneNode` 中的 `target_blade_` 对象通过 `update(contour_info, current_time)` 方法更新。
        -   存储传入的 `contour_info`。
        -   计算从 `contour_info.circle_center` (R点) 指向 `contour_info.center` (扇叶中心) 的二维向量 `vector_`。
        -   使用 `atan2(vector_.y, vector_.x)` 计算该向量的角度 `angle_`。
        -   记录当前观测的时间戳 `time_`。

### 3. 运动预测 (`Prediction`)

-   **数据收集与更新 (`Prediction::update`)**:
    -   `RuneNode` 将 `target_blade_.vector_` 和当前时间戳传递给 `prediction_.update()`。
    -   `Prediction::update` 内部：
        -   从传入的 `orientation` 计算当前扇叶的原始角度 `current_radian_raw = atan2(orientation.y, orientation.x)`。
        -   `angle_of(current_radian_raw)` 函数对角度进行初步处理。
        -   将处理后的角度 `current_radian` 和对应的时间戳存储在内部的 `radians_` 和 `times_` 向量中。
-   **拟合条件判断 (`Prediction::can_fit`, `Prediction::need_fit`)**:
    -   `can_fit()`: 检查是否有足够的数据点进行拟合。
    -   `need_fit()`: 检查距离上次成功拟合的时间是否超过了配置的 `cfg.refit_delay_sec`。
-   **模型拟合 (`Prediction::fit`)**:
    -   如果满足拟合条件，则执行 `fit()`:
        -   **角度解算 (`unwrapped_radians`)**:
            -   对 `radians_` 存储的角度序列进行解算，消除 `atan2` 结果的周期性跳变，生成一个连续的角度序列。
            -   该函数内部通过乘以5、标准角度解算、再除以5的操作，适应能量机关的5重旋转对称性。
        -   **Ceres Solver 参数优化**:
            -   创建一个 `ceres::Problem`。
            -   待优化的参数是运动模型参数：`k` (角速度基线), `b` (初始相位偏移), `a` (正弦项振幅), `omega` (正弦项角频率), `phi` (正弦项相位偏移)，存储在 `params` 中。
            -   对于每个数据点 `(t, theta)`，创建 `RotationResidual` 代价函数。
            -   `RotationResidual::operator()` 定义残差：`residual = observed_theta - (k*t + b + a*sin(omega*t + phi))`。
            -   Ceres Solver 调整参数以最小化所有数据点残差的平方和。
            -   更新 `params` 并记录 `last_fit_time`。
-   **未来位置预测 (`Prediction::predict`)**:
    -   获取当前时间，并加上配置的打击延迟 `cfg.hit_delay_sec`，得到目标预测时间 `target_time_from_start`。
    -   使用当前拟合得到的 `params` 和 `target_time_from_start`，代入运动模型 `radian(time, k, b, a, omega, phi)` 来计算预测时刻的（解算后的）角度 `predicted_unwrapped_radian`。

### 4. 3D位姿估计与打击点生成

-   **预测角度应用 (`RuneNode::rune_image_callback`)**:
    -   获取 `prediction_.predict()` 返回的预测角度 `predicted_rad`。
    -   `PowerRune::predict(target_blade_.vector_, predicted_rad)`: 将 `target_blade_.vector_` (当前扇叶的朝向向量) 旋转 `predicted_rad` 角度，得到 `predicted_blade_vector_2d_`。
    -   `predicted_point_2d = target_blade_.contour_info.circle_center + predicted_blade_vector_2d_` 计算出预测时刻扇叶中心在当前图像上的2D坐标。
-   **PnP求解当前位姿**:
    -   定义能量机关扇叶的3D模型点 `object_points_`。
    -   准备当前帧检测到的扇叶的2D图像点 `rune_imagePoints` (来自 `ContourInfo` 的角点)。
    -   使用 `cv::solvePnP(object_points_, rune_imagePoints, camera_matrix_, dist_coeffs_, rvec, tvec, ...)` 计算当前扇叶相对于相机的旋转向量 `rvec` 和平移向量 `tvec`。
-   **生成3D预测点**:
    -   利用当前PnP解算得到的深度信息 (`tvec.at<double>(2)`)，将 `predicted_point_2d` (预测的2D图像点) 反投影到3D空间，得到预测打击点在相机坐标系下的3D坐标 `(X_cam, Y_cam, Z_cam)`。

### 5. 坐标变换与发布

-   **TF2变换**:
    -   将相机坐标系下的3D预测点封装成 `geometry_msgs::msg::PointStamped`，其 `header.frame_id` 设置为相机的光学坐标系名。
    -   使用 `tf2_buffer_->transform()` 将该点从相机坐标系变换到目标参考系 (例如 `"odom"`)。
-   **结果发布**:
    -   发布变换后的3D预测点到 `/rune/prediction` 主题。
    -   （可选）在原始图像上绘制调试信息，并将结果图像发布到 `rune/result_image` 主题。

