# iSV2-RS 香橙派 RS-485 远程控制

本工程将串口与实时运动控制放在香橙派上，Windows 电脑只运行 Tkinter 上位机：

```text
Windows 上位机 192.168.2.1
        │ HTTP JSON / 250 ms 心跳
        ▼
Orange Pi 192.168.2.5:8000
        │ 80 ms JOG 刷新 / Modbus RTU 38400 8N2
        ▼
USB-RS485 ── iSV2-RS 左轮 ID 2 / 右轮 ID 1
```

香橙派断开上位机心跳 1 秒后会开始本地柔性停车。该机制不替代硬件急停、限位、驱动器使能或断电安全链路。

## 文件说明

- `isv2_tool.py`：Modbus RTU 命令行调试工具。
- `robot_control.py`：与界面无关的双轮控制、JOG 和柔停逻辑。
- `remote_client.py`：与 Tkinter 无关的 HTTP、会话和心跳客户端。
- `orange_pi_server.py`：香橙派 HTTP 控制服务，唯一持有 RS-485 串口。
- `robot_gui.py`：Windows 远程上位机。
- `deploy/`：systemd 服务、环境配置和安装脚本。

## RS-485 接线

- USB-RS485 `A/+` 接电机 CN6 pin 1 `RS485+`。
- USB-RS485 `B/-` 接电机 CN6 pin 2 `RS485-`。
- USB-RS485 `GND` 接电机 CN6 pin 3 `485_GND`。
- 总线两端各使用 120 Ω 终端电阻；使用能够自动切换收发方向的 USB-RS485 适配器。

接入适配器后在香橙派确认设备名：

```bash
ls -l /dev/serial/by-id/ /dev/ttyUSB* 2>/dev/null
```

服务配置为 `auto` 时优先选择 `/dev/serial/by-id/...`，其次选择 `/dev/ttyUSB*`。正式使用时建议在 `/etc/default/isv2-controller` 中写入固定的 `/dev/serial/by-id/...` 路径。

## 部署到香橙派

电脑与香橙派当前使用：

- Windows：`192.168.2.1/24`
- Orange Pi：`192.168.2.5/24`
- SSH 用户：`orangepi`

在本目录的 PowerShell 中执行；命令会提示输入 SSH 密码，密码不会写入工程文件：

```powershell
ssh orangepi@192.168.2.5 "mkdir -p /home/orangepi/isv2-controller/deploy"
scp .\isv2_tool.py .\robot_control.py .\remote_client.py .\orange_pi_server.py .\requirements.txt orangepi@192.168.2.5:/home/orangepi/isv2-controller/
scp .\deploy\* orangepi@192.168.2.5:/home/orangepi/isv2-controller/deploy/
ssh orangepi@192.168.2.5 "cd /home/orangepi/isv2-controller && sudo sh deploy/install_orangepi.sh"
```

如果系统提示缺少 `venv`，先执行：

```bash
sudo apt-get update
sudo apt-get install -y python3-venv
sudo sh /home/orangepi/isv2-controller/deploy/install_orangepi.sh
```

香橙派不能访问软件源时，可在 Windows 下载纯 Python wheel 后离线安装：

```powershell
python -m pip download --only-binary=:all: --no-deps pyserial==3.5 --dest $env:TEMP\isv2-wheels
ssh orangepi@192.168.2.5 "mkdir -p /home/orangepi/isv2-controller/deploy/wheels"
scp $env:TEMP\isv2-wheels\pyserial-3.5-py2.py3-none-any.whl orangepi@192.168.2.5:/home/orangepi/isv2-controller/deploy/wheels/
ssh orangepi@192.168.2.5 "cd /home/orangepi/isv2-controller && sudo sh deploy/install_orangepi.sh"
```

安装器会在缺少 `ensurepip` 时自动使用 `venv --without-pip`，并从 `deploy/wheels` 解包 `pyserial`。

服务管理：

```bash
sudo systemctl status isv2-controller
sudo systemctl restart isv2-controller
journalctl -u isv2-controller -f
```

健康检查：

```powershell
Invoke-RestMethod http://192.168.2.5:8000/api/v1/health
```

未接 USB-RS485 时，HTTP 服务仍会运行，健康状态显示 `serial_unavailable`，并每 2 秒自动尝试重新打开串口。

## Windows 上位机

安装依赖并启动：

```powershell
python -m pip install -r requirements.txt
python .\robot_gui.py
```

连接默认地址 `192.168.2.5:8000`。硬件参数由香橙派返回并只读显示，GUI 可以设置当前速度及 JOG 加减速参数。连续运动期间 RS-485 只发送控制指令；报警与手动诊断可在停车后读取。

控制方式：

```text
按住 W / ↑       前进，松开柔停
按住 S / ↓       后退，松开柔停
按住 A / ←       左转，松开柔停
按住 D / →       右转，松开柔停
Space             柔性停止
```

窗口失去焦点、主动断开或退出时，上位机会请求柔停并释放会话。上位机异常退出或网线断开时，香橙派在心跳超时后自行柔停。只有一个上位机可以取得控制会话。

诊断和单轮 JOG 仅允许在连续运动完全停止后执行，避免读取大量寄存器破坏 80 ms 的 JOG 刷新。运动方向改变时，服务端先对当前运动执行柔停，再执行新的方向命令。

辅助控制字 `0x4001/0x4002` 使用 `Pr6.04 (0x0609)` 作为 JOG 试机速度、使用 `Pr6.25 (0x0633)` 作为共享加减速时间。`Pr8.39～Pr8.41 (0x6027～0x6029)` 属于 PR 模式，不能控制该辅助 JOG；软件默认最高 1500 rpm、加减速参数 10000 ms/Krpm。

## HTTP API

除健康检查和创建会话外，请求都必须携带：

```text
X-Control-Session: <session_id>
```

接口：

- `GET /api/v1/health`
- `POST /api/v1/session`、`DELETE /api/v1/session`
- `POST /api/v1/heartbeat`
- `POST /api/v1/motion/start`、`POST /api/v1/motion/stop`
- `POST /api/v1/jog`
- `GET /api/v1/alarms`、`POST /api/v1/alarms/reset`
- `GET /api/v1/motion-status`
- `GET /api/v1/diagnostics`

错误统一返回：

```json
{
  "ok": false,
  "error": {
    "code": "invalid_parameter",
    "message": "rpm 必须在 1..1500 范围内"
  }
}
```

## 命令行串口调试

停止 systemd 服务后才能让命令行工具独占同一串口：

```bash
sudo systemctl stop isv2-controller
source /home/orangepi/isv2-controller/.venv/bin/activate
python isv2_tool.py probe --port /dev/ttyUSB0 --baud 38400
python isv2_tool.py alarm --port /dev/ttyUSB0 --baud 38400 --id 1
python isv2_tool.py jog --port /dev/ttyUSB0 --baud 38400 --id 1 --dir left --duration 0.5
sudo systemctl start isv2-controller
```

默认通信格式为 38400、8N2。若扫描不到电机，检查设备名、A/B 极性、站号、波特率、参考地和终端电阻。

## 测试与硬件验收

离线测试：

```powershell
python -m unittest -v
python -m py_compile .\isv2_tool.py .\robot_control.py .\remote_client.py .\orange_pi_server.py .\robot_gui.py .\test_isv2_tool.py .\test_remote_control.py
```

首次硬件验收：

1. 让轮子离地并确认硬件急停可用。
2. 连接 GUI，确认健康状态为 `idle`，读取两轴报警。
3. 使用“带载安全档”与 5 rpm 分别短按四个方向和单轮 JOG。
4. 确认松开按键或按钮后完成柔停。
5. 运动中关闭 GUI或断开网线，确认约 1 秒后开始柔停。
6. 恢复网络后确认电机不会自动恢复运动，必须重新取得会话并再次按下方向键。
7. 重启香橙派，确认 systemd 服务自动启动且无运动指令时电机保持停止。
