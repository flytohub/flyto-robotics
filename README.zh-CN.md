<div align="center">

<img src="docs/assets/flyto2-logo.png" alt="Flyto2" width="96" height="96">

# Flyto2 Robotics

**给你手上的 ROS 2 机器人装上 Physical AI。说出要做的事，机器人上什么都不用装。**

![Physical AI](https://img.shields.io/badge/Physical%20AI-ROS%202-6d5dfc) ![Nav2](https://img.shields.io/badge/Nav2-native-22314e) ![Open-RMF](https://img.shields.io/badge/Open--RMF-fleets-0a7d6e) ![License](https://img.shields.io/badge/license-Apache--2.0-blue)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · **简体中文** · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

[快速开始](#快速开始) · [特性](#特性) · [工作原理](#工作原理) · [开发者](#开发者)

</div>

## 快速开始

```bash
git clone https://github.com/flytohub/flyto-robotics && cd flyto-robotics
pip install -e .
python -m flyto_robotics.cli dry-run examples/jobs/pharmacy-to-ward.json   # a full delivery, simulated: no robot needed
```

然后指向你的机器人（`export FLYTO_ROS2_SSH_HOST=ubuntu@my-robot.local`），把它加入 Flyto2 Space，然后说：

> 去厨房拍一张照片。

## 特性

- **机器人零安装**：原厂 TurtleBot3 保持原样，机器人上没有任何 Flyto2 代码或凭据。
- **原生 ROS 2**：直接驱动机器人现有的 Nav2、里程计、激光雷达和摄像头。
- **撞上之前就刹住**：0.35 米安全距离内有东西，绝不往前开。
- **自己绕路**：路被挡住时，会找出通往同一目的地的绕行路线。
- **按名字去地点**：“去充电座”，命名地点就存在机器人地图旁边。
- **到达有证据**：Flyto2 自己核对地图位置，机器人说“我到了”不算数。
- **从一台到一整队**：Open-RMF 车队走的是完全相同的契约。
- **没有机器人也能试**：一次完整的配送模拟，连遇到障碍物停车都包括在内，笔记本上几秒就跑完。

## 工作原理

1. 你用一句话说出要做的事。
2. Flyto2 规划步骤并检查权限。
3. 本适配器通过标准 ROS 2 驱动机器人的 Nav2。
4. Flyto2 核对地图位置，确认后才算完成。

---

## 开发者

架构、安全机制、仿真与测试的细节在英文版里。 [README.md → For developers](README.md#for-developers)

License: Apache-2.0 (`LICENSE`, `NOTICE`).
