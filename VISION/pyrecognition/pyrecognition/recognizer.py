from abc import ABC, abstractmethod
from pyrecognition.utils import strftime, visualize_insdetect, show_mask_on_rgb
from glob import glob
import cv2, os, numpy as np
from pathlib import Path

root_dir = Path(__file__).parent.parent
out_dir = os.path.join(root_dir, 'data/outputs')
os.makedirs(out_dir, exist_ok=True)

test_dir = os.path.join(root_dir, 'test_images')

def make_demo_args(**kwargs):
    if 'rgb_path' in kwargs:
            kwargs['rgb'] = cv2.imread(kwargs['rgb_path'])[...,::-1]
    if 'depth_path' in kwargs:
        kwargs['depth'] = cv2.imread(kwargs['depth_path'], cv2.IMREAD_UNCHANGED)
    if 'cam_params_path' in kwargs:
        with open(kwargs['cam_params_path'], 'r') as f: 
            kwargs['cam_params'] = [float(el) for el in f.read().split(',')]
    if 'rgb' not in kwargs and 'depth' not in kwargs:
        kwargs["rgb_path"] = os.path.join(test_dir, 'rgb.png')
        kwargs["depth_path"] = os.path.join(test_dir, 'depth.png')
        kwargs["cam_params_path"] = os.path.join(test_dir, 'cam_params')
        kwargs['rgb'] = cv2.imread(kwargs["rgb_path"] )[...,::-1]
        kwargs['depth'] = cv2.imread(kwargs["depth_path"], cv2.IMREAD_UNCHANGED)
        with open(kwargs["cam_params_path"], 'r') as f: 
            kwargs['cam_params'] = [float(el) for el in f.read().split(',')]
    if 'rgb_path' in kwargs:
        print(f'{"="*5} RGB path: {kwargs["rgb_path"]}')
    if 'depth_path' in kwargs:
        print(f'{"="*5} Depth path: {kwargs["depth_path"]}')
    if 'cam_params_path' in kwargs:
        print(f'{"="*5} Cam params path: {kwargs["cam_params_path"]}')
    
    return kwargs

        
class Recognizer(ABC):

    @abstractmethod
    def __init__(self, **kwargs):
        NotImplementedError

    @abstractmethod
    def run(self, **kwargs):
        NotImplementedError

    @abstractmethod
    def visualize(self, **kwargs):
        NotImplementedError

    def run_and_visualize(self,**kwargs):
        pred = self.run(**kwargs)
        out = self.visualize(pred=pred, **kwargs)
        try:
            pred_out = pred.todict()
        except:
            pred_out = {}
            print('Cannot convert pred to dict')

        return pred_out, out
        
        
    def demo_single(self, **kwargs):
        kwargs = make_demo_args(**kwargs)
            
        pred = self.run(**kwargs)
        out = self.visualize(pred=pred, **kwargs)
        return pred, out
        
    def demo_multi(self, rgb_format=None, depth_format=None, cam_params_path=None, **kwargs):
        assert rgb_format is not None or depth_format is not None
        
        im_dir = Path(rgb_format).parent if rgb_format is not None else Path(depth_format).parent
        out_dir = os.path.join(im_dir, 'outputs')
        os.makedirs(out_dir, exist_ok=True)
        
        rgb_paths = sorted(glob(rgb_format)) if rgb_format is not None else None
        depth_paths = sorted(glob(depth_format)) if depth_format is not None else None
        
        rgb_paths = len(depth_paths) * [None] if rgb_paths is None else rgb_paths
        depth_paths = len(rgb_paths) * [None] if depth_format is None else depth_paths
        
        assert len(rgb_paths)==len(depth_paths)
        num_im = len(rgb_paths)
        
        cam_params_path = os.path.join(test_dir, 'cam_params') if cam_params_path is None else cam_params_path
        with open(cam_params_path, 'r') as f: 
                cam_params = [float(el) for el in f.read().split(',')]
        
        for j, (rgb_path, depth_path) in enumerate(zip(rgb_paths, depth_paths)):
            print(f'[{j+1}/{num_im}]{rgb_path}--{depth_path}')
            rgb = cv2.imread(rgb_path)[...,::-1] if rgb_path is not None else None
            depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED) if depth_path is not None else None
            pred = self.run(rgb=rgb, depth=depth, cam_params=cam_params, **kwargs)
            out = self.visualize(pred=pred, rgb=rgb, **kwargs)
            
            name = strftime()
            cv2.imwrite(os.path.join(out_dir, f'{name}_rgb.png'), out[...,::-1])
            
    
class InsDetector(Recognizer):

    def visualize(self, rgb, pred, show_steps=False, **kwargs):
        try:
            return pred.visualize(rgb=rgb)
        except:
            return visualize_insdetect(rgb=rgb, pred=pred, show_steps=show_steps)


    def find_suggest(self, obj_name, rgb=None, instances=None, **kwargs):
        from pyplanner.utils.chatgpt import matchObjToCategory
        assert rgb is not None or instances is not None

        ins = self.run(rgb=rgb) if instances is None else instances
        if ins is None:
            print(f'Instance is None. Exit ...')
            return None
        target_ind, alt_obj_name = None, obj_name
        # find alternative
        if obj_name in ins['label_indexes']:
            target_ind = ins['label_indexes'][obj_name][0]
        else:
            alt_obj_name = matchObjToCategory(obj_name=obj_name, cat_list=list(ins['label_indexes'].keys()))
            target_ind = None if alt_obj_name is None else ins['label_indexes'][alt_obj_name][0]
            if alt_obj_name is not None:
                print(f'Detected {alt_obj_name} instead  of {obj_name}')

        if target_ind is not None:
            left, top, right, bottom = ins['boxes'][target_ind, :].astype('int')
            # ins['im'] = cv2.rectangle(ins['im'], (left, top), (right, bottom), (255,0,0), 2)

        ins['target_ind'] = target_ind
        ins['alt_obj_name'] = alt_obj_name

        return ins

    def find_centered_object(self, rgb=None, instances=None, **kwargs):
        assert rgb is not None or instances is not None

        instances = self.run(rgb=rgb) if instances is None else instances
        if instances is None:
            return None
        if len(instances['masks'])==0:
            return None
        h,w = instances['masks'][0].shape[:2]
        dX = [abs((x0+x1)//2 - w/2) for x0,_,x1,_ in instances['boxes']]
        instances['target_ind'] = int(np.argmin(dX))
        return instances

    
class GraspDetect(Recognizer):

    def visualize(self, rgb, pred, topn=None, **kwargs):
        try:
            if hasattr(self, 'suction_mask'):
                out = show_mask_on_rgb(rgb=rgb, mask=self.suction_mask)
            else:
                out = self.instances.visualize(rgb=rgb)
        except:
            out = rgb.copy()
        return out if pred is None else pred.disp(out, topn=topn, **kwargs)
            



