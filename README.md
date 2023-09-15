# 基于nRF7002-DK的蓝牙HID应用

Funpack活动项目，基于nRF7002-DK实现的蓝牙键盘鼠标+NFC模拟标签。

## 项目背景

这是我在参与的第二个Funpack活动项目。任务内容如下：

* 任务一：
> 使用板卡的蓝牙连接，设计一个蓝牙鼠标+键盘复合设备，按键1作为鼠标点击，按键2作为键盘输入按下时输入“eetree”字符，电脑开启大写锁定时，板卡的LED亮起

* 任务二：
> 使用WiFi连接功能，连接网络，并实现远程控制板卡LED和读取按键信息

* 任务三：
> 使用板卡的NFC功能，模拟出一个自定义功能的卡片，使用手机靠近并能读取卡片信息

* 任务四：
> 若您针对这个板卡有更好的创意，可自命题完成（难度不能低于以上任务）


这次活动我最开始选择完成的是任务一，设计一个蓝牙鼠标+键盘复合设备，但是开发完键鼠共功能以后发现最后那个通过同步电脑大小写的状态没法做，群里大佬说是官方SDK有问题。不得已又加了任务三的内容，所以实际我完成的应该算任务四吧。


## 硬件介绍


nRF7002-DK 简介

nRF7002-DK——用于nRF7002 Wi-Fi 6双频辅助IC的开发套件。

nRF7002 DK是nRF7002 Wi-Fi 6协同IC的开发评估板，它包含了在单板上开发所需要的所有元器件。开发板采用nRF5340多协议SoC作为主处理器，配合nRF7002 Wi-Fi协同芯片。可以同时支持低功耗蓝牙和Wi-Fi 应用的开发，并实现如 OFDMA、波束成形和目标唤醒时间等多项 Wi-Fi 6 功能。

nRF7002是Nordic的Wi-Fi产品系列中的首款器件，符合802.11ax标准，可提供双频段（2.4和5GHz）连接，支持Matter中使用的全部无线协议，可以为产品中添加最新的Wi-Fi 6技术，该芯片还具有帮助保护用户数据的先进安全功能。并与Nordic现有的超低功率技术无缝结合，可延长电池使用寿命。它提供快速、可靠的连接，具有先进的安全功能，并且方便集成到各个应用当中。

板上的nRF5340是支持低功耗蓝牙、蓝牙Mesh、NFC、Thread和Zigbee的双核蓝牙5.3 SoC，并且蓝牙测向可实现所有到达角(AoA)和出发角(AoD)的测量功能。此外，它支持低功耗蓝牙音频，2 Mbps高吞吐量、广播扩展和长距离。像蓝牙Mesh、Thread和Zigbee这样的Mesh协议可以与低功耗蓝牙同时运行，从而使智能手机能够配网、入网、配置和控制Mesh节点。还支持NFC、ANT、802.15.4和2.4 GHz专有协议。

特性：

* Arduino连接器
* 两个可编程的按钮
* 搭载nRF7002 Wi-Fi协同IC
* 作为主处理器的nRF5340 SoC
* 电流测量引脚
* 2.4GHz、2.4/5 GHz和NFC天线
* 高性能的128MHz Arm Cortex-M33应用内核
* 超低功率的64MHz Arm Cortex-M33网络核心




## 项目介绍

本次项目基于[nRFConnect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/introduction.html)进行开发，它集成了[ZephyrOS](https://docs.zephyrproject.org/)操作系统，使用zephyr的多线程管理实现了同时模拟蓝牙HID输入设备和NFC标签的功能。通过接入五轴摇杆外设可以实现模拟鼠标的移动和左右键点击操作。使用手机触碰NFC天线可以实现使用NFC打开[eetree funpack项目首页](https://www.eetree.cn/page/digikey-funpack)的功能。


## 参考资料
nRF Connect SDK官方文档：https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/introduction.html
Zephyr官方文档：https://docs.zephyrproject.org/
* HID报告描述符：https://www.usbzh.com/article/detail-775.html