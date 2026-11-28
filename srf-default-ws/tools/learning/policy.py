"""Fit a continuous behavioral clone or a frozen SmolVLM with discrete action heads."""
import argparse
import json
from pathlib import Path
from functools import lru_cache
import numpy as np
from PIL import Image
import torch
from torch import nn
from actions import SCHEMA, LOW, HIGH, normalized, physical

MODEL='HuggingFaceTB/SmolVLM2-500M-Video-Instruct'
REV='4cde9d208e19519c36917aa170fffc7e381aa0ad'
BINS=257  # Odd count preserves an exact zero base command.


class Encoder:
    def __init__(self, kind, device):
        self.kind,self.device=kind,device
        self.size=7
        if kind=='vla':
            from transformers import AutoProcessor, SmolVLMModel
            self.processor=AutoProcessor.from_pretrained(MODEL,revision=REV)
            # Use one global frame per observation to keep inference inside the ROS control period.
            self.processor.image_processor.do_image_splitting=False
            self.processor.image_processor.size={'longest_edge':512}
            dtype=torch.float16 if torch.device(device).type=='cuda' else torch.float32
            self.model=SmolVLMModel.from_pretrained(MODEL,revision=REV,dtype=dtype).to(device).eval()
            self.model.requires_grad_(False)
            self.size+=self.model.config.text_config.hidden_size

    @torch.inference_mode()
    def __call__(self,image,task,state):
        state=torch.as_tensor(normalized(state),device=self.device,dtype=torch.float32)
        if self.kind=='bc':
            return state
        message=[{'role':'user','content':[{'type':'image'},{'type':'text','text':task}]}]
        prompt=self.processor.apply_chat_template(message,add_generation_prompt=True,tokenize=False)
        batch=self.processor(text=prompt,images=[image.convert('RGB')],return_tensors='pt').to(self.device)
        hidden=self.model(**batch.to(dtype=self.model.dtype),use_cache=False).last_hidden_state[0,-1].float()
        return torch.cat((state,hidden))


def head(width, kind):
    return nn.Sequential(nn.LayerNorm(width) if kind=='vla' else nn.Identity(),
                         nn.Linear(width,128),nn.SiLU(),nn.Linear(128,7*BINS if kind=='vla' else 7))


def decode(pred,kind):
    if kind=='vla':
        return pred.reshape(-1,7,BINS).argmax(-1)*2/(BINS-1)-1
    return pred.tanh()


class Policy:
    def __init__(self,path,device):
        data=torch.load(path,map_location='cpu',weights_only=True)
        if data['schema']!=SCHEMA: raise ValueError('Action schema mismatch')
        self.kind=data['kind']
        self.constant_mask=data.get('constant_mask',torch.zeros(7,dtype=torch.bool)).numpy()
        self.constant_value=data.get('constant_value',torch.zeros(7)).numpy()
        if self.kind=='vla' and data['encoder_revision']!=REV: raise ValueError('Encoder revision mismatch')
        if self.kind=='vla' and data.get('image_preprocessing')!='single-512':raise ValueError('Image preprocessing mismatch')
        self.encoder=Encoder(self.kind,device)
        self.head=head(self.encoder.size,self.kind).to(device).eval()
        self.head.load_state_dict(data['weights'])

    @torch.inference_mode()
    def predict(self,image,task,state):
        features=self.encoder(image,task,state)
        pred=decode(self.head(features[None]),self.kind)[0].cpu().numpy()
        return physical(np.where(self.constant_mask,self.constant_value,pred))


def episode_rows(paths):
    rows=[]
    for path in paths:
        meta=json.loads((path/'episode.json').read_text())
        if meta['schema']!=SCHEMA: raise ValueError(f'Incompatible episode {path}')
        for line in (path/'frames.jsonl').read_text().splitlines():
            frame=json.loads(line)
            rows.append((path/frame['image'],meta['instruction'],frame['state'],frame['action']))
    if not rows: raise ValueError('No frames in supplied episodes')
    return rows


def train():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('data',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--kind',choices=['bc','vla'],default='bc')
    p.add_argument('--web',type=Path,help='Flickr8k captions.jsonl directory for auxiliary image/text matching')
    p.add_argument('--web-weight',type=float,default=.15)
    p.add_argument('--epochs',type=int,default=20);p.add_argument('--batch-size',type=int,default=32)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--device',default='cuda' if torch.cuda.is_available() else 'cpu')
    args=p.parse_args()
    episodes=sorted(x.parent for x in args.data.glob('*/episode.json'))
    if len(episodes)<2: p.error('Provide at least two separate demonstrations for episode-level holdout')
    np.random.default_rng(args.seed).shuffle(episodes)
    split=max(1,len(episodes)//5)
    torch.manual_seed(args.seed)
    encoder=Encoder(args.kind,args.device)
    @lru_cache(maxsize=4096)
    def features(path,task,state):
        with Image.open(path) as im:
            return encoder(im,task,state).cpu()
    def dataset(paths):
        rows=episode_rows(paths)
        x=torch.stack([features(path,task,tuple(s)) for path,task,s,_ in rows])
        y=torch.tensor(normalized(np.array([a for _,_,_,a in rows])),dtype=torch.float32)
        return torch.utils.data.TensorDataset(x,y)
    train_data=dataset(episodes[split:])
    test_data=dataset(episodes[:split])
    model=head(encoder.size,args.kind).to(args.device)
    auxiliary=nn.Linear(128,1).to(args.device)
    optimizer=torch.optim.AdamW([*model.parameters(),*auxiliary.parameters()],lr=.001)
    web_features=None
    if args.web:
        if args.kind!='vla':p.error('--web requires --kind vla')
        pairs=[json.loads(line) for line in (args.web/'captions.jsonl').read_text().splitlines()]
        if len(pairs)<2:raise ValueError('At least two different captions required')
        vectors=[];labels=[]
        for i,pair in enumerate(pairs):
            for match,caption in [(1.,pair['text']),(0.,pairs[(i+1)%len(pairs)]['text'])]:
                vectors.append(features(args.web/pair['image'],caption,tuple((LOW+HIGH)/2)))
                labels.append(match)
        web_features=torch.stack(vectors).to(args.device)
        web_labels=torch.tensor(labels,device=args.device)
    targets=train_data.tensors[1]
    constant_mask=(targets.max(0).values-targets.min(0).values)<1e-6
    constant_value=targets[0].clone()
    for _ in range(args.epochs):
        for x,y in torch.utils.data.DataLoader(train_data,batch_size=args.batch_size,shuffle=True):
            pred=model(x.to(args.device)); y=y.to(args.device)
            if args.kind=='vla':
                targets=((y+1)*(BINS-1)/2).round().long()
                loss=nn.functional.cross_entropy(pred.reshape(-1,BINS),targets.flatten())
            else:
                loss=nn.functional.smooth_l1_loss(pred.tanh(),y)
            if web_features is not None:
                idx=torch.randint(len(web_labels),(min(16,len(web_labels)),),device=args.device)
                logits=auxiliary(model[:-1](web_features[idx])).squeeze(-1)
                match_loss=nn.functional.binary_cross_entropy_with_logits(logits,web_labels[idx])
                loss=loss+args.web_weight*match_loss
            optimizer.zero_grad(); loss.backward(); optimizer.step()
    with torch.no_grad():
        x,y=test_data.tensors
        predictions=torch.cat([decode(model(b.to(args.device)),args.kind).cpu() for b in x.split(args.batch_size)])
        predictions[:,constant_mask]=constant_value[constant_mask]
        error=(predictions-y).abs().mean(0)
    print('held-out normalized MAE per action:',error.tolist(),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    torch.save(dict(schema=SCHEMA,kind=args.kind,weights=model.state_dict(),encoder_revision=REV,image_preprocessing='single-512',
                    constant_mask=constant_mask,constant_value=constant_value,
                    web_source=json.loads((args.web/'source.json').read_text()) if args.web else None,
                    holdout=[p.name for p in episodes[:split]]),args.output)


if __name__=='__main__':
    train()
