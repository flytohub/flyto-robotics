<div align="center">

<img src="docs/assets/flyto2-logo.png" alt="Flyto2" width="96" height="96">

# Flyto2 Robotics

**Physical AI para el robot ROS 2 que ya tienes. Dile qué hacer. No se instala nada en el robot.**

![Physical AI](https://img.shields.io/badge/Physical%20AI-ROS%202-6d5dfc) ![Nav2](https://img.shields.io/badge/Nav2-native-22314e) ![Open-RMF](https://img.shields.io/badge/Open--RMF-fleets-0a7d6e) ![License](https://img.shields.io/badge/license-Apache--2.0-blue)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · **Español** · [Português (BR)](README.pt-BR.md)

[Inicio rápido](#inicio-rápido) · [Lo mejor](#lo-mejor) · [Cómo funciona](#cómo-funciona) · [Para desarrolladores](#para-desarrolladores)

</div>

## Inicio rápido

```bash
git clone https://github.com/flytohub/flyto-robotics && cd flyto-robotics
pip install -e .
python -m flyto_robotics.cli dry-run examples/jobs/pharmacy-to-ward.json   # a full delivery, simulated: no robot needed
```

Después apúntalo a tu robot (`export FLYTO_ROS2_SSH_HOST=ubuntu@my-robot.local`), añádelo a un Space de Flyto2 y di:

> Ve a la cocina y haz una foto.

## Lo mejor

- **Cero instalación en el robot.** Un TurtleBot3 de fábrica sigue de fábrica. Ni código de Flyto2 ni credenciales a bordo.
- **Habla ROS 2 nativo.** Mueve el Nav2, la odometría, el LiDAR y la cámara que tu robot ya tiene.
- **Frena antes de chocar.** No avanza hacia nada dentro de su margen de seguridad de 0,35 m.
- **Encuentra otro camino.** ¿Camino bloqueado? Busca un rodeo hasta el mismo destino.
- **Se sabe los lugares por nombre.** "Ve a la base de carga." Los lugares con nombre se guardan junto al mapa del robot.
- **Llegadas que se demuestran.** Flyto2 comprueba la posición en el mapa por su cuenta. Que el robot diga "ya llegué" no basta.
- **De un robot a toda una flota.** Las flotas Open-RMF usan exactamente el mismo contrato.
- **Pruébalo sin robot.** Una entrega simulada completa, parada por obstáculo incluida, corre en tu portátil en segundos.

## Cómo funciona

1. Dices lo que quieres.
2. Flyto2 lo planifica y revisa los permisos.
3. Este adaptador mueve Nav2 en tu robot con ROS 2 estándar.
4. Flyto2 comprueba la posición en el mapa antes de darlo por hecho.

---

## Para desarrolladores

La arquitectura, la seguridad, la simulación y las pruebas están en la versión en inglés. [README.md → For developers](README.md#for-developers)

License: Apache-2.0 (`LICENSE`, `NOTICE`).
