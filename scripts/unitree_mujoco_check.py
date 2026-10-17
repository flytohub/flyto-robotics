"""Headless Unitree G1 locomotion check: a velocity command, then a stop, measured.

Feasibility probe for docs/UNITREE_SIM.md, not an adapter. It replicates the
control loop of unitree_rl_gym's deploy/deploy_mujoco (BSD-3-Clause, Unitree
Robotics): the published G1 policy at 50 Hz and PD torque at 500 Hz, without a
viewer, and reports how far the base moved and whether it stayed upright.
Nothing from unitree_rl_gym is copied into this repository; point the script
at a local clone.

  pip install mujoco torch pyyaml numpy
  git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym
  python -I scripts/unitree_mujoco_check.py ./unitree_rl_gym 5 3 0.5
"""
import sys

import mujoco
import numpy as np
import torch
import yaml

root, walk_s, stop_s, vx = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
with open(f"{root}/deploy/deploy_mujoco/configs/g1.yaml", encoding="utf-8") as handle:
    cfg = yaml.safe_load(handle)


def sub(path):
    return path.replace("{LEGGED_GYM_ROOT_DIR}", root)


kps, kds = np.array(cfg["kps"], np.float32), np.array(cfg["kds"], np.float32)
default = np.array(cfg["default_angles"], np.float32)
dt, dec = cfg["simulation_dt"], cfg["control_decimation"]
cmd_scale = np.array(cfg["cmd_scale"], np.float32)
n_act, n_obs = cfg["num_actions"], cfg["num_obs"]

m = mujoco.MjModel.from_xml_path(sub(cfg["xml_path"]))
d = mujoco.MjData(m)
m.opt.timestep = dt
policy = torch.jit.load(sub(cfg["policy_path"]))

def gravity(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])

action = np.zeros(n_act, np.float32)
target = default.copy()
obs = np.zeros(n_obs, np.float32)
counter = 0

def run(seconds, cmd):
    global counter, action, target
    for _ in range(int(seconds / dt)):
        d.ctrl[:] = (target - d.qpos[7:]) * kps - d.qvel[6:] * kds
        mujoco.mj_step(m, d)
        counter += 1
        if counter % dec == 0:
            phase = (counter * dt) % 0.8 / 0.8
            obs[:3] = d.qvel[3:6] * cfg["ang_vel_scale"]
            obs[3:6] = gravity(d.qpos[3:7])
            obs[6:9] = cmd * cmd_scale
            obs[9:9 + n_act] = (d.qpos[7:] - default) * cfg["dof_pos_scale"]
            obs[9 + n_act:9 + 2 * n_act] = d.qvel[6:] * cfg["dof_vel_scale"]
            obs[9 + 2 * n_act:9 + 3 * n_act] = action
            angle = 2 * np.pi * phase
            obs[9 + 3 * n_act:9 + 3 * n_act + 2] = [np.sin(angle), np.cos(angle)]
            action = policy(torch.from_numpy(obs).unsqueeze(0)).detach().numpy().squeeze()
            target = action * cfg["action_scale"] + default

start = d.qpos[:3].copy()
run(walk_s, np.array([vx, 0, 0], np.float32))
walked = d.qpos[:3].copy()
run(stop_s, np.array([0, 0, 0], np.float32))
end = d.qpos[:3].copy()
upright = gravity(d.qpos[3:7])[2]
dx, dy = walked[0] - start[0], walked[1] - start[1]
drift = np.linalg.norm(end[:2] - walked[:2])
print(f"walk {walk_s}s at vx={vx}: moved x={dx:.3f} m, y={dy:.3f} m, base z={walked[2]:.3f}")
print(f"stop {stop_s}s: drift {drift:.3f} m, base z={end[2]:.3f}, upright(g_z)={upright:.3f}")
