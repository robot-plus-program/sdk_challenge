from pyrecognition.recognizer import InsDetector
from pyrecognition.vlm.grounding_dino import GroundingDino, __CKPT__ as __DINO_CKPT__, __CONFIG__ as __DINO_CONFIG__
from pyrecognition.instance_detection.fast_sam import InstanceDetector, __CKPT__ as __FASTSAM_CKPT__
from pathlib import Path
import os, cv2
from pyconnect.run_client import get_client

root_dir = Path(__file__).parent.parent.parent
log_dir = os.path.join(root_dir, 'logs/groundedsam')
os.makedirs(log_dir, exist_ok=True)

DINO_ARGS = {'ckpt': __DINO_CKPT__, 'config':__DINO_CONFIG__, 'client_url': None}
FASTSAM_ARGS = {'ckpt': __FASTSAM_CKPT__, 'client_url': None}

class GroundedSam(InsDetector):

    def __init__(self, dino_args=DINO_ARGS, fastsam_args=FASTSAM_ARGS, **kwargs):
        if dino_args['client_url'] is None:
            detector0 = GroundingDino(**dino_args)
            self.groundingdino_run = lambda inputs: detector0.run(**inputs)
        else:
            client0 = get_client(*(el.strip() for el in dino_args['client_url'].split(':')))
            self.groundingdino_run = client0.send
            

        if fastsam_args['client_url'] is None:
            detector1 = InstanceDetector(**fastsam_args)
            self.fastsam_run = lambda inputs: detector1.run(**inputs)
        else:
            client1 = get_client(*(el.strip() for el in fastsam_args['client_url'].split(':')))
            self.fastsam_run = client1.send
        
        

    def run(self, rgb, caption='item', crop_roi=None, min_mass=0, max_mass=200000, min_ratio=0, max_ratio=1,
             box_threshold=0.1, text_threshold=0.15, iou_threshold=0.5, save_steps=False, do_clustering=False, **kwargs):
        inputs = {'rgb':rgb, 'crop_roi':crop_roi, 'min_mass':min_mass, 'max_mass':max_mass, 
                  'min_ratio':min_ratio, 'max_ratio':max_ratio}
        fastsam_ins = self.fastsam_run(inputs)
        inputs = {'rgb':rgb, 'caption':caption, 'crop_roi':crop_roi, 
                  'box_threshold':box_threshold,'text_threshold':text_threshold, 'iou_threshold':iou_threshold,
                  'min_ratio':min_ratio, 'max_ratio':max_ratio, 'do_clustering':do_clustering}
        dino_ins = self.groundingdino_run(inputs)
        # dino_ins.from_maskall(fastsam_ins.mask_all)
        dino_ins.import_masks(masks=fastsam_ins.masks)
        if save_steps:
            cv2.imwrite(os.path.join(log_dir, 'fast_sam.png'), fastsam_ins.visualize(rgb=rgb)[...,::-1])
            cv2.imwrite(os.path.join(log_dir, 'fast_sam_maskall.png'), fastsam_ins.visualize_maskall(rgb=rgb)[...,::-1])
            cv2.imwrite(os.path.join(log_dir, 'dino.png'), dino_ins.visualize(rgb=rgb)[...,::-1])
            cv2.imwrite(os.path.join(log_dir, 'dino_maskall.png'), dino_ins.visualize_maskall(rgb=rgb)[...,::-1])


        return dino_ins

if __name__=='__main__':
    detector =  GroundedSam()
    pred, out = detector.demo_single(save_steps=True)    
        
        

