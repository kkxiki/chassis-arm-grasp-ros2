# 风险与已定决策的实施注意事项

## 第三轮铺设记录

- 新增 `object_radius_mm=0.0 mm` 与 `approach_step_check_ratio=0.6`。前者必须按物体重新量取；取 `0` 会使系统性偏近约一个物体半径，抓 500ml 瓶应填 `32.0 mm`。后者用于 S3 单步有效性判据，不满足时立即中止，禁止继续迭代。
- 固定相机 provider 新增 `camera_info_topic=""`。为空时不订阅也不补偿；配置了话题但内参尚未就绪时必须 WARN 并原样使用深度，绝不以 `m≈1` 或任何兜底内参猜测。否则边缘像素的补偿误差可误导 S6 的 `correction` 守卫。
- S2/S4 与 S6 强制复用 `compensateDepthForRadius` 和同一 `object_radius_mm`。仅补一侧会让粗、精定位引入不同偏差，使 `correction` 守卫失去意义。

## 第二轮修复记录

- S3 已改为单次 `translateDistance(α, lround(d_cmd), approach_speed_mps)` 纯平移，移除了 `rotateAngle`；新增 `approach_angle_sign`（仅 `+1/-1`）并要求总闸关闭、空载验收，禁止用增大步长规避方向错误。
- S6 已改为 `G_new=P_pos+R_P·(p_tool-o_tool)`；60 mm 守卫改为 `||G_new-G_old||₂`，首次以粗定位为 `G_old`，迭代后以上一轮结果为准。
- provider-only 已规定通过全部检查后无条件写入 `target_`，干跑闸仅抑制 `executeGrasp`，并在 `CHANGES.md` 给出完整分支。
- 安全位已改由 `GraspMission` 以 `safe_home_joints` 调 `addJointPoint("mission_safe_home", ...)` 与 `moveToPoint(...)` 实现，不再声称 `goSafeHome()` 遵守配置；每次 S3 前必须同步成功执行。
- 服务客户端节点名已改为配置派生、同进程唯一；默认固定/腕部 provider 分别使用不同节点名。
- 安全不变量已补充：`move_wait_ms` 必须是所有相关轨迹耗时的现场标定上界；张爪占位值和腕部内参兜底值均已标出必须实测。

1. `ArmDirectController` 没有位姿回读，也没有运动完成状态接口。`moveCartL` 只按传入 `wait_ms` 阻塞后返回；因此 D6 所说“等当前 MoveL 走完”在现有接口中只能等该软件等待窗口，不能从控制器确认真实完成。设计仍按 D6 不调用 `stop()`，但现场必须将 `move_wait_ms=3000 ms` 校准为所有相关轨迹的上界。

2. 现有 `goSafeHome()` 固定回 `{0,-90,0,-90,0,0}`，完全忽略配置。复合状态机不修改 `ArmDirectController`，也不调用该接口；由 `GraspMission` 使用 `safe_home_joints` 执行 `addJointPoint("mission_safe_home", ...)` 加 `moveToPoint(...)`，使配置真正生效。

3. `VisionGraspNode` 当前在 `processCandidate` 的 `execute_enabled=false` 分支于写入 `target_` 前返回。实施时必须按 `CHANGES.md` 重排：目标通过全部检查后无条件写入 `target_`；`execute_enabled` 仅抑制 `executeGrasp`。这样 provider-only 可绕开干跑早退并支持全关验坐标。

4. 当前 `VisionGraspNode` 的服务客户端节点名固定为 `vision_grasp_service_client`，且客户端服务名写死为相对 `gather_depth`。实施 `depth_service/transform_service/service_client_node_name` 配置字段；同一进程构造的每个 provider 使用由配置派生的唯一节点名。仅给外部 gather_depth 加 `/wrist` 命名空间并不足以让当前 C++ 客户端访问它。

5. D2“provider_only 照常做坐标转换”和 D5“S6 不需要 transform_xyz”对腕部 provider 存在直接冲突。本文按 D5 执行：固定相机 `transform_enabled=true`；腕部 `transform_enabled=false`，仍复用 YOLO、稳定帧和 GatherDepth，S6 在 `GraspMission` 内完成反投影与工具系换算。目标接口因此额外携带 `(pixel_u,pixel_v,depth_m)`，而不仅是基座坐标。

6. D3 规定“底盘运动前机械臂必须已在安全位”，但当前控制器无关节/末端反馈，`arm_working=false` 仅表示软件互斥锁未占用，不证明实际已到安全位。实现必须在每次 S3 前同步调用并成功返回任务内的 `moveMissionSafeHome()`，然后才检查 `arm_working=false` 和移动；这仍是基于等待时间的证明。

7. `translateDistance` 的文档仅说明“前方右侧为正”，与数学 `atan2` 的逆时针正方向可能相反。本文以 `approach_angle_sign∈{+1,-1}` 显式映射，并要求在空载、执行总闸关闭条件下验收；S3 不允许旋转。一旦方向错误，应修正该符号，不可通过增大 `max_approach_step_mm` 绕过。

8. 外部 legacy 工作区不在本仓库，无法核实 `gather_depth` 和 `yolov8_detector_node` 的 ROS 包名、可执行名、所订阅的两台相机话题，亦无法核实 D405 `CameraInfo` 实际路径。`LAUNCH.md` 使用用户提供的组件名作为包/可执行名，并给出 `ros2 pkg executables` 发现命令；实施前必须以实际工作区输出替换二元组。

9. `MoveTwoFingersGripper` 非阻塞，现有 `executeGrasp` 合爪后立即退回。本设计在 S8 强制等待 `hand_wait_ms=900 ms`；该值只是已定默认值，安全验收应以夹爪闭合和夹持稳定的实测结果确认。

10. 当前 `main.cpp` 将配置路径硬编码为 `/home/robot/robot_ws/config/config.json`（537 行），而仓库根目录是其他路径。未按任务修改代码；部署时必须确保该路径存在或在实施变更时一并以启动参数替代，否则新配置不会被读取。
