"""Fit a phase-indexed joint trajectory to HF demonstrations and color-conditioned Isaac demonstrations."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


def image_features(image, instruction):
    colors = [color for color in ('red','blue') if color in instruction.lower().split()]
    if len(colors) != 1:
        raise ValueError('Expected one red or blue cube in the instruction')
    rgb = np.asarray(image.convert('RGB'), dtype=float)
    axis = 0 if colors[0] == 'red' else 2
    chosen = rgb[:,:,axis]
    others = np.delete(rgb, axis, axis=2).max(axis=2)
    y, x = np.nonzero((chosen > 45) & (chosen > 1.6*others))
    if len(x) < 50:
        raise ValueError('Selected block missing from the starting RGB')
    # A 20 mm cube looks smaller when it is farther from the pinhole camera.
    y0, y1 = np.percentile(y,[1,99])
    width_scale = max(y1-y0,1.)
    return np.array([100/width_scale, (np.median(x)-320)/width_scale])


class VLAFeatures:
    """Reuse the HF-trained VLA encoder and action head's learned hidden layer."""
    def __init__(self, parent):
        import torch
        from policy import Policy
        self.policy = Policy(parent, 'cuda' if torch.cuda.is_available() else 'cpu')
        if self.policy.kind != 'vla':
            raise ValueError('Expected the pretrained visual-language policy')
        self.hidden = self.policy.head[:-1]
        self.digest = hashlib.sha256(parent.read_bytes()).hexdigest()

    def __call__(self, image, instruction, state):
        import torch
        with torch.inference_mode():
            values = self.hidden(self.policy.encoder(image,instruction,state)).cpu().numpy()
        return values / max(np.linalg.norm(values),1e-6)


class ColorPolicy:
    def __init__(self, checkpoint, parent):
        self.vla = VLAFeatures(parent)
        with np.load(checkpoint, allow_pickle=False) as model:
            if self.vla.digest != str(model['parent_sha256']):
                raise ValueError('HF VLA parent checkpoint hash mismatch')
            self.mean, self.scale, self.weights = (model[k] for k in ('mean','scale','weights'))

    def plan(self, image, instruction, state):
        geometry = (image_features(image,instruction)-self.mean)/self.scale
        design = np.r_[1., 1., geometry, self.vla(image,instruction,state)]
        return np.einsum('i,itj->tj',design,self.weights)


def resample(actions, steps):
    old = np.linspace(0,1,len(actions))
    new = np.linspace(0,1,steps)
    return np.stack([np.interp(new,old,column) for column in np.asarray(actions).T],axis=1)


def train():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('hf', type=Path)
    p.add_argument('demos', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--extra-demos', type=Path, action='append', default=[],
                   help='Include successful Isaac demonstrations from another seed (repeatable)')
    p.add_argument('--parent-vla',type=Path,required=True)
    p.add_argument('--dataset',type=Path,help='Create a local mixed dataset compatible with policy.py')
    args = p.parse_args()
    if args.dataset:
        args.dataset.mkdir(parents=True,exist_ok=False)
    encoder = VLAFeatures(args.parent_vla)
    hidden = []
    features, trajectories, sources = [], [], []
    steps = None
    seeds = []
    for demo_root in [args.demos, *args.extra_demos]:
        report = json.loads((demo_root/'results.json').read_text())
        if report['controller'] != 'teacher':
            raise ValueError('Provide a completed teacher run')
        if steps is not None and steps != report['decisions']:
            raise ValueError('Demonstration horizons differ')
        steps = report['decisions']
        if report['seed'] in seeds:
            raise ValueError('Duplicate demonstration seed')
        seeds.append(report['seed'])
        for row in report['results']:
            if not row['success']:
                continue
            directory = demo_root / f"{row['layout']:02d}-{row['selected']}"
            rows = [json.loads(line) for line in (directory/'trace.jsonl').read_text().splitlines()]
            with Image.open(directory/'start.png') as rgb:
                hidden.append(encoder(rgb,row['instruction'],rows[0]['state']))
                features.append(image_features(rgb,row['instruction']))
            actions = [r['action'] for r in rows]
            if args.dataset:
                from actions import SCHEMA, NAMES
                episode = args.dataset/(f'isaac-{report["seed"]}-'+directory.name if args.extra_demos else 'isaac-'+directory.name)
                episode.mkdir()
                (episode/'images').symlink_to(directory.resolve(), target_is_directory=True)
                (episode/'episode.json').write_text(json.dumps(dict(schema=SCHEMA,state=NAMES,action=NAMES,
                    instruction=row['instruction'],source='Isaac Sim 6.1',seed=report['seed'],initial=row['initial'])))
                with (episode/'frames.jsonl').open('w') as frames:
                    for r in rows:
                        frames.write(json.dumps(dict(image=f"images/{r['step']:03d}.jpg",
                            state=r['state'],action=r['action'],time=r['step']*.25))+'\n')
            if len(actions) != steps:
                raise ValueError('Incomplete color trajectory')
            trajectories.append(actions)
            sources.append(dict(episode=directory.name, seed=report['seed'], start_joints=rows[0]['state'][2:], initial=row['initial'], instruction=row['instruction']))
    if len(sources) < 4 or len({r['instruction'] for r in sources}) < 2:
        raise ValueError('Need at least four complete demonstrations spanning both instructions')
    mean = np.mean(features,axis=0)
    scale = np.maximum(np.std(features,axis=0),1e-6)
    # Shared phase weights + a simulation correction + visible target geometry.
    design = [np.r_[1.,1.,(feature-mean)/scale] for feature in features]
    hf_sources = []
    for path in sorted(args.hf.glob('*/episode.json')):
        meta = json.loads(path.read_text())
        if meta.get('repo') != 'Hadolking/openmanipulator-pickplace-dataset-1114ep':
            continue
        raw = (path.parent/'frames.jsonl').read_bytes()
        actions = [json.loads(line)['action'] for line in raw.splitlines()]
        trajectories.append(resample(actions,steps))
        first = json.loads(raw.splitlines()[0])
        with Image.open(path.parent/first['image']) as rgb:
            hidden.append(encoder(rgb,meta['instruction'],first['state']))
        if args.dataset:
            (args.dataset/path.parent.name).symlink_to(path.parent.resolve(),target_is_directory=True)
        design.append([1.,0.,0.,0.])
        hf_sources.append(dict(episode=meta['source_episode'],revision=meta['revision'],
                               sha256=hashlib.sha256(raw).hexdigest(), frames=len(actions)))
    if not hf_sources:
        raise ValueError('No HF robot demonstrations in input')
    x = np.column_stack((design,hidden))
    y = np.asarray(trajectories).reshape(len(x),-1)
    penalty = np.diag(np.r_[1e-6,.004,.04,.04,np.full(x.shape[1]-4,.5)])
    weights = np.linalg.solve(x.T@x+penalty,x.T@y).reshape(x.shape[1],steps,7)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('xb') as file:
        np.savez(file,mean=mean,scale=scale,weights=weights,parent_sha256=encoder.digest)
    manifest = dict(method='phase-conditioned ridge behavioral cloning', steps=steps,
                    checkpoint_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                    seed=seeds[0],seeds=seeds,parent_sha256=encoder.digest, hf=hf_sources, color_demos=sources)
    args.output.with_suffix('.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Trained on {len(hf_sources)} HF and {len(sources)} successful color episodes.')


if __name__ == '__main__':
    train()
