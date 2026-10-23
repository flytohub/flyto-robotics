<div align="center">

<img src="docs/assets/flyto2-logo.png" alt="Flyto2" width="96" height="96">

# Flyto2 Robotics

**Physical AI para o robô ROS 2 que você já tem. Diga o que fazer. Nada é instalado no robô.**

![Physical AI](https://img.shields.io/badge/Physical%20AI-ROS%202-6d5dfc) ![Nav2](https://img.shields.io/badge/Nav2-native-22314e) ![Open-RMF](https://img.shields.io/badge/Open--RMF-fleets-0a7d6e) ![License](https://img.shields.io/badge/license-Apache--2.0-blue)

[English](README.md) · [日本語](README.ja.md) · [한국어](README.ko.md) · [简体中文](README.zh-CN.md) · [Español](README.es.md) · **Português (BR)**

[Início rápido](#início-rápido) · [Destaques](#destaques) · [Como funciona](#como-funciona) · [Para desenvolvedores](#para-desenvolvedores)

<img src="docs/assets/demo-dry-run.gif" alt="A simulated delivery: the robot drives to the pharmacy, stops for an obstacle, then delivers to Ward A" width="760">

</div>

## Início rápido

```bash
git clone https://github.com/flytohub/flyto-robotics && cd flyto-robotics
pip install -e .
python -m flyto_robotics.cli dry-run examples/jobs/pharmacy-to-ward.json   # a full delivery, simulated: no robot needed
```

Depois aponte para o seu robô (`export FLYTO_ROS2_SSH_HOST=ubuntu@my-robot.local`), adicione a um Space do Flyto2 e diga:

> Vai até a cozinha e tira uma foto.

## Destaques

- **Zero instalação no robô.** Um TurtleBot3 de fábrica continua de fábrica. Nada de código do Flyto2 nem credenciais a bordo.
- **Fala ROS 2 nativo.** Comanda o Nav2, a odometria, o LiDAR e a câmera que o robô já tem.
- **Freia antes de bater.** Não avança em nada dentro da margem de segurança de 0,35 m.
- **Acha outro caminho.** Caminho bloqueado? Ele monta um desvio até o mesmo destino.
- **Conhece os lugares pelo nome.** "Vai até a base de carga." Lugares com nome ficam salvos ao lado do mapa do robô.
- **Chegada com prova.** O Flyto2 confere a posição no mapa por conta própria. O robô dizer "cheguei" não basta.
- **De um robô a uma frota inteira.** Frotas Open-RMF usam exatamente o mesmo contrato.
- **Teste sem robô.** Uma entrega simulada completa, com parada por obstáculo, roda no seu notebook em segundos.

## Como funciona

1. Você diz o que quer.
2. O Flyto2 planeja e confere as permissões.
3. Este adaptador comanda o Nav2 do robô por ROS 2 padrão.
4. O Flyto2 confere a posição no mapa antes de dar como feito.

---

## Para desenvolvedores

Arquitetura, segurança, simulação e testes estão na versão em inglês. [README.md → For developers](README.md#for-developers)

License: Apache-2.0 (`LICENSE`, `NOTICE`).
