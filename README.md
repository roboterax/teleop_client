
## 遥操作服务文档

### 登录 developer

```
ssh developer@192.168.8.100 -p 2222 #密码：developer
git clone http://gitlab.robotera.com/software/rbclient.git
cd rbclient
clocon build
source install/setup.bash
python pub_client.py -h 查看命令
```

### 启动sdk
```
python pub_client.py --cmd start_sdk
```

### 初始化遥操作
```
python pub_client.py --cmd init_teleop --verify [path] --hand [xhand | lite] --mocap [vr | gamepad] --camera-type [dummy | realsense | stereo]
注意：path是授权文件路径，需要把授权文件解压
```

### 使用vr设备连接机器人
```
在网页中输入网址：https://192.168.8.100:8010。
当网页显示 "datachannle open" 后可以点击网页中 “Start AR
```

### 启动遥操作
```
python pub_client.py --cmd start_teleop
```

### 停止遥操作
```
python pub_client.py --cmd stop_teleop
```

### 停止sdk
```
python pub_client.py --cmd stop_sdk
```