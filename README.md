# Hunting-UAV
Hunting Unmanned Aerial Vehicle 2019 全国大学生电子设计竞赛 【B题】

<p align="center">
  <a href="https://img.shields.io/badge/language-C-brigreen.svg?style=flat-square"><img src="https://img.shields.io/badge/language-C-brigreen.svg?style=flat-square" alt="C"></a>
</p>


[通信协议](./Docs/README.md)

## 简介
2019集大电协 巡线无人机

本文对巡线机器人进行了初步的研究和设计，该机器人由四旋翼飞行器与检测识别模块构成。根据四旋翼飞行器飞行原理，选择合适的无刷电机作为系统动力装置，选取了功能强大且容易开发的微处理器、传感器和相关电子元器件，并做了大量的系统软硬件调试工作，最终完成了整体设计。
本机器人采用TM4C123GH6PMI单片机为主控制器，通过六轴加速度陀螺仪ICM20602采集的四旋翼欧拉角数据进行ADRC处理以保持飞机的姿态稳定；采用SPL06气压传感器以及激光测距模块GY-53L1控制巡线机器人的飞行高度；使用OpenMV实现巡检电力线路及杆塔状态。


工具     | 描述
-------- | -----
Altium Design|PCB设计
Keil|软件编程
Solidworks|3D软件（3D打印）
123D Design|3D软件（激光切割）


| 硬件 | 描述 |
| -- | -- |
|芯片型号| TM4C123G |
|CPU| Cortex-M4 |
|主频| 80MHz |
|FLASH| 256KB |
|RAM|  32KB |
|EEPROM|  2KB |
|单元| FPU、DSP |

## 目录说明
````
+——Marine craft
|----+ Docs: 【设计文档】
|    |  tool 【调试工具】
|    |  参考文献 
|---- 3D model: 【3D模型、激光切割】
|----+ hardware:【电路设计】  
|       Controller 【飞控主控板】
|---+ software:【软件设计】
|       ├──ETA-fly【飞控程序】
````

## 3D model
- 摄像头支架

![摄像头支架](/Docs/Pictures/infrared.png "摄像头支架")

- 激光切割

![激光切割亚克力转接板](/Docs/Pictures/laser cutting.png "激光切割亚克力转接板")


## Hardware

![飞控主控](/Docs/Pictures/Controller-PCB.png "飞控主控")

![飞控主控3D](/Docs/Pictures/Controller-broad.png "飞控主控3D")

## Sensor
- 主控与外设描述

硬件     | 描述
-------- | -----
陀螺仪  | ICM-20602
加速度计  | SPL06
磁力计  | IST8310
激光测距  |GY-53L1x
光流|优象


## 通信协议
#### 》》协议表格中，若带 0x 开头代表该值是16进制，无0x开头，则为十进制《《
|编号 | 0| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |11  |12  |13  |14  |15|
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |--- |--- |
|数据包所代表 | 包头 | 包头 |模式位  | 数据长度位 |保留位  |保留位 |保留位 |保留位 | 保留位  |保留位  |保留位  | 保留位 |保留位| 保留位| 保留位|累加和校验sum|
|描述 | 0xAA| 0x55 | 1~255 | <12 |  |  |  |  |  |   |  |  |  | || sum



1. AA 55为包头
2. 模式位：巡线远程模式等等
2. 数据长度位：代表后面除校验位有几位有效数据
3. 保留位：不传输
4. 校验和：从包头开始到校验位前的所有数据和的低八位

