# USB Video 实例

## 用法

### 软件需要

* 可播放摄像头视频流的软件

### 项目配置

### 开发板设置
```
Example Video Initialization Configuration  --->
    Select Target Development Board  --->
        (X) Customized Development Board
```

#### MIPI-CSI 设置

```
Component config  --->
    Espressif Camera Sensors Configurations  --->
        [*] OV5647  ---->
            Default format select for MIPI (RAW8 1280x720 30fps, MIPI 2lane 24M input)  --->
                (X) RAW10 1920x1080 30fps, MIPI 2-lane, 24M input

    USB Device UVC  --->
        USB Cam1 Config  --->
             UVC Default Resolution (HD 1280x720)  --->
                (X) FHD 1920x1080
            (30) Frame Rate (FPS)
            (1920) Cam1 Frame Width
            (1080) Cam1 Frame Height
```

####  视频流类型

##### JPEG

```
Component config  --->
    USB Device UVC  --->
        USB Cam1 Config  --->
             Cam1 Format (MJPEG)  --->
                (X) MJPEG
```

##### H.264

```
Component config  --->
    USB Device UVC  --->
        USB Cam1 Config  --->
             UVC Cam1 Format (H264)  --->
                (X) H264
```

