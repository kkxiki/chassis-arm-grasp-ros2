# 机械臂单独测试

用于单独验证 TCP 直连机械臂控制器（默认 `192.168.30.11:5868`）的连接与基本运动。

## 依赖

- 纯 C++17 + POSIX socket，无需额外库（`Threads`）
- 复用上级工程的 `include/`、`src/`、`third_party/`

## 编译（在虚拟机上）

```bash
cd hardware_tests/arm
cmake -B build .
cmake --build build -j
```

## 运行

```bash
./build/arm_test
```

启动前先在 `arm_config.json` 中确认机械臂 IP/端口。

## 常用命令

| 命令 | 说明 |
| --- | --- |
| `connect [ip] [port]` | 连接控制器（默认读配置） |
| `clear` | 清除故障 |
| `enable` | 上电使能 |
| `home` | 回安全零位（6轴全 0°） |
| `addj name j1..j6` | 添加关节点位（自动限位校验） |
| `move name [wait_ms]` | 运动到关节点位 |
| `addc name x y z rx ry rz` | 添加笛卡尔点位（mm/度） |
| `movej / movel name [wait_ms]` | 笛卡尔关节插补 / 直线插补 |
| `speed 1~100` | 设置全局速度倍率 |
| `raw {指令}` | 发送原始指令（如抓手 `{SetHandState --state=0 --hand_id=0}`） |
| `q` | 退出（自动回零并断开） |

> 安全提示：首次测试建议把速度倍率调低（如 `speed 20`），
> 并确保机械臂周围无障碍物、急停可触达。
