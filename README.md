# OPENVIO_APP

## OPENVIO APP源码。 

## **OPENVIO线上文档请点击下面链接** 

[OPENVIO线上文档](http://guanglundz.com/openvio)  

## 简介

OPENVIO 一款脱胎于OPENMV的智能摄像头。

在OPENMV硬件基础上增加了USB2.0芯片(USB3315）和IMU芯片（MPU6050），除却兼容OPENMV固件外，还可以将摄像头原始图像（未压缩图像）和IMU九轴数据高速传输至PC，可以作为SLAM单目IMU方案研究的低廉传感器方案（如港科大的VINS-MONO）.多种接口方便扩展更多功能，比如扩展超声波或激光模块后作为PX4光流模块使用（暂未实现 还在研发中）。

## 源码和资料

[新架构OPENVIO-APP源码(github)](https://github.com/guanglun/openvio)【开发环境：Vscode Makefile】  

[新架构OPENVIO-BOOTLOADER源码(github)](https://github.com/guanglun/openvio_bootloader)【开发环境：Vscode Makefile】 

[新架构OPENVIO-APP源码(gitee)](https://gitee.com/guanglunking/openvio)【开发环境：Vscode Makefile】  

[新架构OPENVIO-BOOTLOADER源码(gitee)](https://gitee.com/guanglunking/openvio_bootloader)【开发环境：Vscode Makefile】  

[OPENVIO源码](https://gitee.com/guanglunking/OPENVIO_BOARD)【开发环境：Keil5】  

[OPENVIO PC上位机](https://gitee.com/guanglunking/OPENVIO_PC)【开发环境：QT5.6.0 qt-opensource-windows-x86-mingw492-5.6.0】  

[OPENVIO ROS源码](https://gitee.com/guanglunking/OPENVIO_ROS)

[淘宝店铺](https://item.taobao.com/item.htm?id=615919130291)  

## 使用 

#### 介绍
OPENVIO板子固件程序

#### 当前问题描述

1. 上位机无法切换分辨率
2. 同时开启摄像头和IMU目前只有在MT9V034 752x480分辨率下能够使用
3. OV7725上位机和屏幕同时显示小概率花屏
4. 上位机打开CAM有概率出现问题  


## 欢迎加入 光流电子交流群  558343678  