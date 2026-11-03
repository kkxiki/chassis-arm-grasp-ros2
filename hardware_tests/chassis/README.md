# 底盘单独测试

用于单独验证 AGV 底盘的调度服务(8008)与直控服务(8767)是否连通、动作是否正常。

## 依赖

- libcurl（`sudo apt install -y libcurl4-openssl-dev`）
- 复用上级工程的 `include/`、`src/`、`third_party/`（编译时按相对路径引用，目录结构请保持）

## 编译（在虚拟机上）

```bash
cd hardware_tests/chassis
cmake -B build .
cmake --build build -j
```

## 运行

```bash
./build/chassis_test
```

启动前先在 `chassis_config.json` 中确认底盘 IP（默认 `192.168.30.200`）与 robot_id（默认 `227`）。

## 常用命令

| 命令 | 说明 |
| --- | --- |
| `status` | 查询所有机器人状态（调度 8008） |
| `pose` | 查询当前位姿 x/y/angle |
| `nodes [map_id]` | 查询地图作业点坐标 |
| `move x y angle [speed]` | 直控移动（8767，不阻塞） |
| `stop` | 停止移动 |
| `rot angle [speed]` | 直控原地旋转（相对角度） |
| `stoprot` | 停止旋转 |
| `goto x y angle` | 直控巡航到位（阻塞轮询，后台线程可 `stop` 打断） |
| `home` | 返回配置的原点 |
| `speed mv rs` | 设置直线/旋转速度 |
| `q` | 退出（自动停止所有运动） |

> 安全提示：测试前确认底盘周围无障碍物、急停可触达。
