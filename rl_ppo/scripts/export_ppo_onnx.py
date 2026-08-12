#!/usr/bin/env python3
"""
export_ppo_onnx.py
==================
Run this on your TRAINING PC (inside the venv that has torch + SB3 +
f1tenth_gym), NOT on the car. The car has onnxruntime but no torch and no
stable_baselines3, so the policy must cross over as ONNX.

It produces two files that always travel together:

    ppo_policy.onnx        the deterministic policy: obs[1,110] -> action[1,2]
    ppo_deploy_config.json every scaling constant the car node needs

The JSON exists so that max_speed / min_speed / max_steer / lidar_clip /
num_beams / fov are read from THE TRAINING CODE ITSELF rather than retyped
on the car. Retyping is how a policy silently ends up commanding a
different speed range than it was trained for.

WHAT IT ALSO DOES — the part that matters most:
    Numerical parity check. It feeds N random observations through both
    SB3's model.predict(deterministic=True) and onnxruntime, and reports
    the max absolute difference. If that number is not ~1e-6, the export
    is broken and you must NOT deploy it.

--------------------------------------------------------------------------
USAGE (on the PC)
--------------------------------------------------------------------------
    source /data/dhyeyshah_ws/.venv/bin/activate
    cd /data/dhyeyshah_ws/training

    # normal case — reads constants live from the training wrapper
    python3 export_ppo_onnx.py \
        --model models/best/best_model.zip \
        --out-dir deploy \
        --map my_map5

    # if you cannot import the training env on this machine
    python3 export_ppo_onnx.py \
        --model models/best/best_model.zip \
        --out-dir deploy \
        --no-probe-env

Then copy both files to the car:
    scp deploy/ppo_policy.onnx deploy/ppo_deploy_config.json \
        nvidia@<car-ip>:~/f1tenth_ws/src/rl_ppo/scripts/
"""

import argparse
import json
import os
import sys

import numpy as np


def probe_training_env(map_name, train_module):
    """
    Instantiate the real training wrapper and read the constants off it.
    This is the whole point of --probe-env: the deployed scaling comes from
    the same code path that trained the policy, not from CLI defaults.
    """
    import importlib
    mod = importlib.import_module(train_module)
    env = mod.F1TenthRLWrapper(map_name=map_name)

    info = {
        'num_beams':  int(env.num_beams),
        'obs_dim':    int(env.observation_space.shape[0]),
        'lidar_clip': float(env.lidar_clip),
        'max_speed':  float(env.max_speed),
        'min_speed':  float(env.min_speed),
        'max_steer':  float(env.max_steer),
    }

    # LiDAR FOV — the gym exposes this in a few different places depending
    # on version, so try them in order and fall back to the documented
    # default (4.7 rad) with a loud warning rather than guessing silently.
    fov = None
    for getter in (
        lambda: env.env.unwrapped.sim.agents[0].scan_simulator.fov,
        lambda: env.env.unwrapped.config.lidar_config.fov,
        lambda: env.env.unwrapped.sim.agents[0].lidar_config.fov,
        lambda: env.env.unwrapped.lidar_config.fov,
    ):
        try:
            v = float(getter())
            if v > 0:
                fov = v
                break
        except Exception:
            continue
    if fov is None:
        fov = 4.7
        print('  [!] Could not read LiDAR fov from the gym — assuming 4.7 rad '
              '(269.3 deg), the f1tenth_gym default. VERIFY THIS.')
    info['lidar_fov_rad'] = fov

    # Control period — how much sim time one env.step() advances. The policy
    # acts once per step, so this sets the deployment control rate.
    dt = None
    for getter in (
        lambda: env.env.unwrapped.timestep,
        lambda: env.env.unwrapped.config.simulation_config.timestep,
        lambda: env.env.unwrapped.sim.timestep,
    ):
        try:
            v = float(getter())
            if v > 0:
                dt = v
                break
        except Exception:
            continue
    if dt is None:
        dt = 0.01
        print('  [!] Could not read the sim timestep — assuming 0.01 s (100 Hz).')
    info['sim_timestep'] = dt
    info['control_rate_hz'] = round(1.0 / dt, 3)

    env.close()
    return info


def load_vecnormalize(path, obs_dim):
    """
    train_ppo_basic.py uses norm_obs=False, so this normally returns
    'not normalized' and vecnormalize.pkl is irrelevant at inference. It is
    checked anyway because flipping norm_obs=True during a later experiment
    and forgetting it here would produce a policy that drives into a wall.
    """
    if not path or not os.path.isfile(path):
        return {'norm_obs': False}
    from stable_baselines3.common.vec_env import VecNormalize
    import pickle
    with open(path, 'rb') as f:
        vn = pickle.load(f)
    if not getattr(vn, 'norm_obs', False):
        print(f'  vecnormalize.pkl: norm_obs=False -> obs normalization NOT '
              f'needed at inference (as expected).')
        return {'norm_obs': False}
    print('  [!] vecnormalize.pkl has norm_obs=True — embedding obs stats. '
          'The car node WILL apply them.')
    return {
        'norm_obs': True,
        'obs_mean': np.asarray(vn.obs_rms.mean, dtype=np.float64).ravel().tolist(),
        'obs_var':  np.asarray(vn.obs_rms.var,  dtype=np.float64).ravel().tolist(),
        'clip_obs': float(getattr(vn, 'clip_obs', 10.0)),
    }


def main():
    p = argparse.ArgumentParser(description='Export SB3 PPO -> ONNX for the car')
    p.add_argument('--model',    default='models/best/best_model.zip')
    p.add_argument('--vecnorm',  default='models/vecnormalize.pkl')
    p.add_argument('--out-dir',  default='deploy')
    p.add_argument('--map',      default='my_map5')
    p.add_argument('--train-module', default='train_ppo_basic',
                   help='module name of your training script (no .py)')
    p.add_argument('--no-probe-env', action='store_true',
                   help='skip building the gym env; use the fallback constants below')
    p.add_argument('--opset', type=int, default=17)
    p.add_argument('--check-samples', type=int, default=2000)

    # Fallbacks, only used with --no-probe-env. These mirror the defaults in
    # F1TenthRLWrapper.__init__ — if you changed them there, change them here.
    p.add_argument('--num-beams',  type=int,   default=108)
    p.add_argument('--lidar-clip', type=float, default=10.0)
    p.add_argument('--max-speed',  type=float, default=6.0)
    p.add_argument('--min-speed',  type=float, default=1.5)
    p.add_argument('--max-steer',  type=float, default=0.4189)
    p.add_argument('--fov',        type=float, default=4.7)
    p.add_argument('--control-rate', type=float, default=100.0)
    args = p.parse_args()

    import torch
    import torch.nn as nn
    from stable_baselines3 import PPO

    os.makedirs(args.out_dir, exist_ok=True)

    print('=' * 66)
    print('  SB3 PPO -> ONNX export')
    print('=' * 66)

    # ── 1. Constants ────────────────────────────────────────────────────
    if args.no_probe_env:
        print('  Using CLI fallback constants (--no-probe-env).')
        info = {
            'num_beams':     args.num_beams,
            'obs_dim':       args.num_beams + 2,
            'lidar_clip':    args.lidar_clip,
            'max_speed':     args.max_speed,
            'min_speed':     args.min_speed,
            'max_steer':     args.max_steer,
            'lidar_fov_rad': args.fov,
            'sim_timestep':  round(1.0 / args.control_rate, 6),
            'control_rate_hz': args.control_rate,
        }
    else:
        print(f'  Probing training env (map={args.map}) for exact constants...')
        info = probe_training_env(args.map, args.train_module)

    for k, v in info.items():
        print(f'    {k:16s} = {v}')

    # ── 2. Load the policy ──────────────────────────────────────────────
    print(f'\n  Loading {args.model} ...')
    model = PPO.load(args.model, device='cpu')
    model.policy.set_training_mode(False)
    sb3_obs_dim = int(model.observation_space.shape[0])
    if sb3_obs_dim != info['obs_dim']:
        raise SystemExit(
            f'  ABORT: the saved model expects obs_dim={sb3_obs_dim} but the '
            f'training env reports {info["obs_dim"]}. You are exporting a '
            f'model that does not match this env config.')
    print(f'    obs_dim={sb3_obs_dim}  act_dim={model.action_space.shape[0]}')

    # ── 3. Wrap + export ────────────────────────────────────────────────
    class DeterministicPolicy(nn.Module):
        """
        _predict(deterministic=True) is the same call SB3's own
        model.predict() makes internally, minus the numpy/clipping wrapper.
        Clipping to the action space is done on the car instead, so this
        graph stays a pure MLP with no version-specific behaviour baked in.
        """

        def __init__(self, policy):
            super().__init__()
            self.policy = policy

        def forward(self, observation):
            return self.policy._predict(observation, deterministic=True)

    wrapper = DeterministicPolicy(model.policy).eval()
    dummy = torch.zeros(1, sb3_obs_dim, dtype=torch.float32)
    onnx_path = os.path.join(args.out_dir, 'ppo_policy.onnx')

    torch.onnx.export(
        wrapper, dummy, onnx_path,
        input_names=['observation'],
        output_names=['action'],
        dynamic_axes={'observation': {0: 'batch'}, 'action': {0: 'batch'}},
        opset_version=args.opset,
        do_constant_folding=True,
    )
    print(f'\n  Wrote {onnx_path} ({os.path.getsize(onnx_path) / 1024:.1f} KB)')

    # ── 4. Parity check — the gate on whether this is deployable ────────
    print(f'\n  Parity check over {args.check_samples} random observations...')
    import onnxruntime as ort
    sess = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])

    rng = np.random.default_rng(0)
    # Sampled from the true observation domain: everything is in [0,1] by
    # construction in F1TenthRLWrapper._process_obs, so a uniform sample
    # covers the space the policy will actually be queried on.
    obs = rng.random((args.check_samples, sb3_obs_dim)).astype(np.float32)

    sb3_act, _ = model.predict(obs, deterministic=True)
    onnx_act = sess.run(None, {'observation': obs})[0]
    onnx_act = np.clip(onnx_act, -1.0, 1.0)   # car clips too; compare like for like

    diff = np.abs(sb3_act - onnx_act)
    max_diff = float(diff.max())
    print(f'    max |SB3 - ONNX| = {max_diff:.3e}   (mean {diff.mean():.3e})')
    if max_diff > 1e-4:
        raise SystemExit(
            '  ABORT: export does not reproduce the SB3 policy. Do NOT deploy '
            'this file. Check the SB3/torch versions and the policy type.')
    print('    PASS — the ONNX graph reproduces the SB3 policy.')

    # ── 5. Config sidecar ───────────────────────────────────────────────
    cfg = dict(info)
    cfg.update(load_vecnormalize(args.vecnorm, sb3_obs_dim))
    cfg.update({
        'source_model': os.path.abspath(args.model),
        'map':          args.map,
        'act_dim':      int(model.action_space.shape[0]),
        'opset':        args.opset,
        'parity_max_abs_diff': max_diff,
        'note': ('Consumed by ppo_deploy_node.py on the car. Every scaling '
                 'constant here came from the training code, not by hand.'),
    })
    cfg_path = os.path.join(args.out_dir, 'ppo_deploy_config.json')
    with open(cfg_path, 'w') as f:
        json.dump(cfg, f, indent=2)
    print(f'  Wrote {cfg_path}')

    print('\n' + '=' * 66)
    print('  Copy BOTH files to the car:')
    print(f'    scp {onnx_path} {cfg_path} \\')
    print('        nvidia@<car-ip>:~/f1tenth_ws/src/rl_ppo/scripts/')
    print('=' * 66)


if __name__ == '__main__':
    sys.exit(main())
