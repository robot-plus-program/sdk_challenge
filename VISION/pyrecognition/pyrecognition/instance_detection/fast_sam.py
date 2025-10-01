import argparse

import cv2, numpy as np
from docutils.nodes import image
from fastsam import FastSAM, FastSAMPrompt
import ast
import torch, os
from PIL import Image
# from ketisdk.utils.proc_utils import download_gdrive_file
from pathlib import Path
from pyrecognition.recognizer import  InsDetector
from pyrecognition.utils import crop_image
from pyinterfaces.instances import CustomInstances

root_dir = Path(__file__).parent.parent.parent
ckpt_dir = os.path.join(root_dir, 'ckpts')
os.makedirs(ckpt_dir, exist_ok=True)
__CKPT__='FastSAM-x.pt'
__CKPTS_IDS__ = {
    'FastSAM-x.pt': '1m1sjY4ihXBU1fZXdQ-Xdj-mDltW-2Rqv'}
device = 'cuda' if torch.cuda.is_available() else 'cpu'

def download_gdrive_file(gdrive_id, filepath):
    import gdown
    if os.path.exists(filepath):
        print(f'{filepath} existed ...')
        return
    os.makedirs(os.path.split(filepath)[0], exist_ok=True)
    gdown.download(f'https://docs.google.com/uc?export=download&id={gdrive_id}', filepath)
    print(f'File {filepath} downloaded ...')

def get_net(ckpt=__CKPT__):
    ckpt_path = os.path.join(ckpt_dir, ckpt)
    download_gdrive_file(__CKPTS_IDS__[ckpt], ckpt_path)
    return FastSAM(ckpt_path)


def fastsam_exec(model, rgb, crop_roi=None, high_resolution=True, imgsz=1024, conf=0.4, iou=0.5, **kwargs):
    img = Image.fromarray(crop_image(rgb, crop_roi=crop_roi, keep_size=True))
    return model(img, device=device, retina_masks=high_resolution, imgsz=imgsz, conf=conf, iou=iou)


def fastsam_prompt_process(rgb, everything_results, crop_roi=None):
    img = Image.fromarray(crop_image(rgb, crop_roi=crop_roi, keep_size=True))
    return FastSAMPrompt(image=img, results=everything_results, device=device)


class InstanceDetector(InsDetector):
    def __init__(self, ckpt=__CKPT__, **kwargs):
        self.model = get_net(ckpt=ckpt)

    def run(self, rgb, crop_roi=None, text_prompt=None, box_prompt=None, min_mass=0, max_mass=200000,
            min_ratio=0.01, max_ratio=1,**kwargs):
        everything_results = fastsam_exec(self.model, rgb=rgb, crop_roi=crop_roi, **kwargs)
        if everything_results is None:
            return None
        prompt_process =  fastsam_prompt_process(rgb=rgb, everything_results=everything_results, crop_roi=crop_roi)
        
        ret = prompt_process.results[0]
        # if text_prompt is None and box_prompt is None:
        #     ret = prompt_process.results[0]
        # elif text_prompt is not None:
        #     ret = prompt_process.text_prompt(text=text_prompt)
        # else:
        #     ret = prompt_process.box_prompt(bboxes=box_prompt)
        
        masks, boxes = (ret.masks.data.cpu().numpy()>0).astype('uint8'), ret.boxes.xyxy.data.cpu().numpy().astype('int')
        scores = ret.boxes.conf.data.cpu().numpy()
        ins = CustomInstances(masks=masks, boxes= boxes, scores=scores)
        ins = ins.clip_mass(min_mass=min_mass, max_mass=max_mass)
        ins = ins.clip_ratio(min_ratio=min_ratio, max_ratio=max_ratio)

        return  ins
        






if __name__=='__main__':
    detector = InstanceDetector()
    detector.demo_single(
        # rgb_path='/media/keti/workdir/projects/data/care_robot/logs(1)/vision/20241220102327856283_20241220102503831801_rgb_hand.png',
        show_steps=True, 
        # text_prompt='cup'
    )
    # detector.demo_multi(
    #     rgb_format='/media/keti/workdir/projects/data/care_robot/logs(1)/vision/*rgb*',
    # )
    

