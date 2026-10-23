<div align="center">

<img src="docs/assets/flyto2-logo.png" alt="Flyto2" width="96" height="96">

# Flyto2 Robotics

**이미 가진 ROS 2 로봇에 Physical AI를. 할 일만 말하세요. 로봇에는 아무것도 설치하지 않아요.**

![Physical AI](https://img.shields.io/badge/Physical%20AI-ROS%202-6d5dfc) ![Nav2](https://img.shields.io/badge/Nav2-native-22314e) ![Open-RMF](https://img.shields.io/badge/Open--RMF-fleets-0a7d6e) ![License](https://img.shields.io/badge/license-Apache--2.0-blue)

[English](README.md) · [日本語](README.ja.md) · **한국어** · [简体中文](README.zh-CN.md) · [Español](README.es.md) · [Português (BR)](README.pt-BR.md)

[빠른 시작](#빠른-시작) · [주요 기능](#주요-기능) · [작동 방식](#작동-방식) · [개발자용](#개발자용)

</div>

## 빠른 시작

```bash
git clone https://github.com/flytohub/flyto-robotics && cd flyto-robotics
pip install -e .
python -m flyto_robotics.cli dry-run examples/jobs/pharmacy-to-ward.json   # a full delivery, simulated: no robot needed
```

그다음 로봇을 지정하고(`export FLYTO_ROS2_SSH_HOST=ubuntu@my-robot.local`), Flyto2 Space에 추가해서 이렇게 말하면 돼요:

> 주방에 가서 사진 찍어 줘.

## 주요 기능

- **로봇에 설치할 건 0개.** 순정 TurtleBot3는 순정 그대로예요. Flyto2 코드도 자격 증명도 올리지 않아요.
- **네이티브 ROS 2로 말해요.** 로봇에 이미 있는 Nav2, 오도메트리, LiDAR, 카메라를 움직여요.
- **부딪히기 전에 멈춰요.** 0.35 m 안전 거리 안에 있는 건 향해 가지 않아요.
- **알아서 돌아가요.** 길이 막히면 같은 목적지로 가는 우회로를 짜요.
- **장소를 이름으로 기억해요.** "충전 독으로 가." 이름 붙인 장소는 로봇 지도 옆에 저장돼요.
- **도착은 증거로.** Flyto2가 지도 위치를 직접 확인해요. 로봇이 "도착했어요"라고만 해서는 안 돼요.
- **한 대부터 플릿까지.** Open-RMF 플릿도 똑같은 계약으로 움직여요.
- **로봇 없이도 해 봐요.** 장애물 정지까지 포함한 배송 시뮬레이션이 노트북에서 몇 초 만에 돌아가요.

## 작동 방식

1. 원하는 걸 말해요.
2. Flyto2가 계획하고 권한을 확인해요.
3. 이 어댑터가 표준 ROS 2로 로봇의 Nav2를 움직여요.
4. Flyto2가 지도 위치를 확인한 뒤에 완료로 판정해요.

---

## 개발자용

아키텍처, 안전 장치, 시뮬레이션, 테스트 내용은 영어 버전에 있어요. [README.md → For developers](README.md#for-developers)

License: Apache-2.0 (`LICENSE`, `NOTICE`).
