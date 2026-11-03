# 因时 Inspire 4B4C 夹爪单独测试

用于单独验证因时 4B4C 独立夹爪（串口 115200 波特率，协议 `EB 90 | id | len | opcode | payload | checksum`）。

## 依赖安装

```bash
pip3 install -r requirements.txt   # pyserial
```

## 运行

```bash
python3 gripper_test.py            # 默认 /dev/ttyUSB0
python3 gripper_test.py /dev/ttyUSB1   # 指定其他串口
```

若夹爪 ID 不是 1，可修改脚本顶部 `GRIPPER_ID`。

## 常用命令

| 命令 | 说明 |
| --- | --- |
| `open [port]` | 打开串口 |
| `clear` | 清除错误 |
| `release speed` | 松开到最大开口（速度 1~1000） |
| `grasp speed force [hold]` | 抓取（速度 1~1000，力 50~1000） |
| `move opening` | 移动到指定开口（0~1000） |
| `opening` | 查询当前开口 |
| `status` | 查询运行状态/错误码/温度 |
| `limits` / `setlimits max min` | 查询/设置开口限位 |
| `stop` | 急停 |
| `q` | 退出 |

## 快速自检流程

```text
open
clear
status        → 确认错误码为 0
release 200   → 夹爪应张开
move 500      → 夹爪应停在中间开口
opening       → 应显示约 500
grasp 150 300 → 夹爪应合拢夹持
stop          → 应急急停
```

> 安全提示：运行前确认夹爪周围无障碍物、急停可触达。
