# 贞实控制器日志查看说明

控制器：192.168.30.11（贞实 / taiKeU058T）
两个日志服务：
- **codeit-deploy**：运动内核，记录每条 RPC 命令及返回码（如 Clear / Enable / MoveBlend / MoveL）
- **RpcGateway**：RPC 网关，记录会话连接（127.0.0.1:5866）、ZMQ 状态订阅（19091）、初始化事件等

## 方法一：Web 界面（推荐）

1. 浏览器打开 `http://192.168.30.11`
2. 登录：使用控制器出厂默认账号（见设备手册）。**注意：出厂默认口令必须在上线前修改**
3. 进入「服务管理 / 节点管理」页面，找到 `codeit-deploy` 和 `RpcGateway` 两个节点
4. 点击节点旁的「查看日志」按钮，会新窗口打开：
   - `http://192.168.30.11/log?node=codeit-deploy`
   - `http://192.168.30.11/log?node=RpcGateway`
5. 页面内实时滚动显示该服务日志

## 方法二：命令行 API（无需登录，可直接 curl）

```bash
# 查看 codeit-deploy 日志
curl -X POST "http://192.168.30.11/nd-mgr/log?action=read-log" \
     -H "Content-Type: application/json" \
     -d '{"node_name":"codeit-deploy","file_name":""}'

# 查看 RpcGateway 日志
curl -X POST "http://192.168.30.11/nd-mgr/log?action=read-log" \
     -H "Content-Type: application/json" \
     -d '{"node_name":"RpcGateway","file_name":""}'
```

返回格式：`{"code":0,"logs":["行1","行2",...]}`，`logs` 为逐行日志数组。

## 日志内容示例

codeit-deploy（每条命令带返回码，0=成功）：
```
cmd 3134---{JogCartesian --coordinate=0 --direction=-1 --increase_count --moving_type=2 --speed=v80}
    3134--- subcmd_index:0return code :1
cmd 3110---{MoveTwoFingersGripper --distance=0 --interval=25}
    3110--- subcmd_index:0return code :0
```

RpcGateway（服务状态）：
```
[RPC会话管理器] connected 127.0.0.1:5866 code=0(连接成功)
[系统状态订阅] 订阅: [overall_system_rtstate] 推送周期: 8ms
[话题处理] 模型0名称: taiKeU058T_0
工作目录: /home/pi/Desktop/rpc_gateway/v1.1.2/bin
```

## 注意事项

- 日志接口目前只返回最近 100 行；查看更早历史需研究 `file_name` 参数或直接登录控制器找日志文件
- 控制器 SSH 端口 22 开放，常见默认口令均不通过，SSH 凭证需另行确认；Web 端可用出厂默认账号登录
- 日志常用于排查机械臂运动问题：codeit-deploy 里能直接看到下发的命令及返回码（如 -1003、-504）对应的执行记录
