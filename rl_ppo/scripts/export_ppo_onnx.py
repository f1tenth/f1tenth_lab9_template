#!/usr/bin/env python3
"""
export_ppo_onnx.py
==================
Run this on your TRAINING PC (needs torch, onnx, onnxruntime; stable_baselines3
is optional, only used to double-check vecnormalize.pkl), NOT on the car. The car has onnxruntime but no torch, so the policy must cross
over as ONNX.

It produces two files that always travel together:

    ppo_policy.onnx        the deterministic policy: obs[1,N] -> action[1,2]
    ppo_deploy_config.json every scaling constant the car node needs

The policy is the from-scratch ActorCritic trained in
PPO_from_Scratch_LearnToDrive_Student.ipynb, saved there via:

    torch.save(net.state_dict(), 'models_scratch/ppo_scratch.pt')

This script rebuilds the same ActorCritic class, loads the weights into it, and exports the
deterministic (mean) action head to ONNX.

The JSON config exists so that max_speed / min_speed / max_steer /
lidar_clip / num_beams / fov are read from ONE place rather than retyped on the car.
Retyping is how a policy silently ends up commanding a different speed
range than it was trained for. 

WHAT IT ALSO DOES — the part that matters most:
    Numerical parity check. It feeds N random observations through both
    the PyTorch net (deterministic action) and onnxruntime, and reports
    the max absolute difference. If that number is not ~1e-6, the export
    is broken and you must NOT deploy it.


"""

import argparse
import json
import os
import sys

import numpy as np
import torch
import torch.nn as nn


def load_vecnormalize(path):
    """
    The notebook's make_training_env() always uses norm_obs=False (lidar is
    already in [0,1] by construction) — so this normally returns
    'not normalized' and vecnormalize.pkl only carries reward-normalization
    stats, which are irrelevant at inference. It is checked anyway because
    flipping norm_obs=True during a later experiment and forgetting it here
    would produce a policy that drives into a wall.
    """
    if not path or not os.path.isfile(path):
        return {'norm_obs': False}
    import pickle
    try:
        # Unpickling needs stable_baselines3 installed (the pkl holds a VecNormalize).
        with open(path, 'rb') as f:
            vn = pickle.load(f)
    except ImportError:
        print('  vecnormalize.pkl: stable_baselines3 not installed, skipping check. '
              'Assuming norm_obs=False (the notebook default). '
              'pip install stable-baselines3 to verify.')
        return {'norm_obs': False}
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


def build_actor_critic():
    """
    Mirrors the ActorCritic class defined in
    PPO_from_Scratch_LearnToDrive_Student.ipynb. Kept in sync by hand —
    if you change the network architecture in the notebook, change it here
    too, or state_dict loading below will fail with a key/shape mismatch.
    """
    class ActorCritic(nn.Module):
        def __init__(self, obs_dim, act_dim, hidden=64):
            super().__init__()
            self.actor = nn.Sequential(
                nn.Linear(obs_dim, hidden), nn.Tanh(),
                nn.Linear(hidden, hidden),  nn.Tanh(),
            )
            self.actor_mean = nn.Linear(hidden, act_dim)

            self.critic_body = nn.Sequential(
                nn.Linear(obs_dim, hidden), nn.Tanh(),
                nn.Linear(hidden, hidden),  nn.Tanh(),
            )
            self.critic = nn.Linear(hidden, 1)
            self.log_std = nn.Parameter(torch.zeros(act_dim))

        def forward(self, obs):
            mean  = self.actor_mean(self.actor(obs))
            value = self.critic(self.critic_body(obs)).squeeze(-1)
            return mean, value

    return ActorCritic


def infer_dims(state_dict):
    """Read obs_dim / act_dim / hidden straight off the checkpoint tensors
    instead of hardcoding them — stays correct even if the notebook's
    hidden size or beam count changes."""
    hidden, obs_dim = state_dict['actor.0.weight'].shape
    act_dim = state_dict['actor_mean.weight'].shape[0]
    return int(obs_dim), int(act_dim), int(hidden)


def main():
    p = argparse.ArgumentParser(
        description='Export the from-scratch ActorCritic checkpoint -> ONNX for the car')
    p.add_argument('--model',    default='models_scratch/ppo_scratch.pt',
                   help='path to the state_dict saved by the notebook (torch.save(net.state_dict(), ...))')
    p.add_argument('--vecnorm',  default='models_scratch/vecnormalize.pkl')
    p.add_argument('--out-dir',  default='deploy')
    p.add_argument('--opset', type=int, default=17)
    p.add_argument('--check-samples', type=int, default=2000)

    # These mirror F1TenthRLWrapper's defaults in the notebook — override
    # only if you changed the wrapper's config there.
    p.add_argument('--num-beams',  type=int,   default=108)
    p.add_argument('--lidar-clip', type=float, default=10.0)
    p.add_argument('--max-speed',  type=float, default=6.0)
    p.add_argument('--min-speed',  type=float, default=1.5)
    p.add_argument('--max-steer',  type=float, default=0.4189)
    p.add_argument('--fov',        type=float, default=4.7)
    p.add_argument('--control-rate', type=float, default=100.0)
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    print('=' * 66)
    print('  From-scratch PPO (ActorCritic state_dict) -> ONNX export')
    print('=' * 66)

    # ── 1. Constants — mirror the training wrapper (no probing: training is
    #        a notebook, not an importable module) ─────────────────────────
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
    for k, v in info.items():
        print(f'    {k:16s} = {v}')

    # ── 2. Load the checkpoint ──────────────────────────────────────────
    print(f'\n  Loading {args.model} ...')
    state_dict = torch.load(args.model, map_location='cpu')
    obs_dim, act_dim, hidden = infer_dims(state_dict)
    if obs_dim != info['obs_dim']:
        raise SystemExit(
            f'  ABORT: the checkpoint expects obs_dim={obs_dim} but --num-beams '
            f'implies obs_dim={info["obs_dim"]} ({args.num_beams} + 2). Pass the '
            f'--num-beams value used when the notebook\'s F1TenthRLWrapper was built.')

    ActorCritic = build_actor_critic()
    net = ActorCritic(obs_dim, act_dim, hidden=hidden)
    net.load_state_dict(state_dict)
    net.eval()
    print(f'    obs_dim={obs_dim}  act_dim={act_dim}  hidden={hidden}')

    # ── 3. Wrap + export ─────────────────────────────────────────────────
    class DeterministicPolicy(nn.Module):
        """
        The deterministic action is dist.mean, i.e. actor_mean(actor(obs)) —
        the same value net.act(obs, deterministic=True) returns in the
        notebook, minus the Normal() distribution wrapper (ONNX can't export
        torch.distributions objects, so we skip straight to the mean).
        """

        def __init__(self, net):
            super().__init__()
            self.actor = net.actor
            self.actor_mean = net.actor_mean

        def forward(self, observation):
            return self.actor_mean(self.actor(observation))

    wrapper = DeterministicPolicy(net).eval()
    dummy = torch.zeros(1, obs_dim, dtype=torch.float32)
    onnx_path = os.path.join(args.out_dir, 'ppo_policy.onnx')

    export_kwargs = dict(
        input_names=['observation'],
        output_names=['action'],
        dynamic_axes={'observation': {0: 'batch'}, 'action': {0: 'batch'}},
        opset_version=args.opset,
        do_constant_folding=True,
    )
    try:
        # Force the legacy TorchScript exporter — dynamic_axes is its API.
        # Newer torch defaults to the "dynamo" exporter, which needs the
        # extra `onnxscript` package most training environments won't have.
        # Older torch versions don't know this kwarg at all, hence the fallback.
        torch.onnx.export(wrapper, dummy, onnx_path, dynamo=False, **export_kwargs)
    except TypeError:
        torch.onnx.export(wrapper, dummy, onnx_path, **export_kwargs)
    print(f'\n  Wrote {onnx_path} ({os.path.getsize(onnx_path) / 1024:.1f} KB)')

    # ── 4. Parity check — the gate on whether this is deployable ────────
    print(f'\n  Parity check over {args.check_samples} random observations...')
    import onnxruntime as ort
    sess = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])

    rng = np.random.default_rng(0)
    # Sampled from the true observation domain: everything is in [0,1] by
    # construction in F1TenthRLWrapper._process_obs, so a uniform sample
    # covers the space the policy will actually be queried on.
    obs = rng.random((args.check_samples, obs_dim)).astype(np.float32)

    with torch.no_grad():
        torch_act = wrapper(torch.from_numpy(obs)).numpy()
    torch_act = np.clip(torch_act, -1.0, 1.0)   # car clips too; compare like for like
    onnx_act = sess.run(None, {'observation': obs})[0]
    onnx_act = np.clip(onnx_act, -1.0, 1.0)

    diff = np.abs(torch_act - onnx_act)
    max_diff = float(diff.max())
    print(f'    max |PyTorch - ONNX| = {max_diff:.3e}   (mean {diff.mean():.3e})')
    if max_diff > 1e-4:
        raise SystemExit(
            '  ABORT: export does not reproduce the trained policy. Do NOT deploy '
            'this file. Check the torch/onnx versions and the ActorCritic architecture.')
    print('    PASS — the ONNX graph reproduces the trained policy.')

    # ── 5. Config sidecar ───────────────────────────────────────────────
    cfg = dict(info)
    cfg.update(load_vecnormalize(args.vecnorm))
    cfg.update({
        'source_model': os.path.abspath(args.model),
        'act_dim':      act_dim,
        'hidden':       hidden,
        'opset':        args.opset,
        'parity_max_abs_diff': max_diff,
        'note': ('Consumed by ppo_deploy_node.py on the car. Every scaling '
                 'constant here should match F1TenthRLWrapper in the training '
                 'notebook, not be retyped by hand.'),
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
