from pyrecognition.recognizer import Recognizer
from pyrecognition.vlm.grounded_sam import GroundedSam
from pyrecognition.grasp.kgrasp import mask2grasps
from pyrecognition.utils import show_text_on_rgb

class InstanceGrasp(Recognizer):
    def __init__(self, **kwargs):
        self.instance_detector = GroundedSam()
    
    def run(self, rgb, depth=None, cam_params=None, caption='item', crop_roi=None, min_mass=200, max_ratio=0.6, nfingers=2, dmin=10, dmax=200):
        
        caption = caption.replace(',', '.')
        ins = self.instance_detector.run(rgb=rgb, crop_roi=crop_roi, caption=caption,  min_mass=min_mass, max_ratio=max_ratio)

        obj_names = [caption.strip(),] if '.' not in caption else [el.strip() for el in caption.split('.')]
        out = {}
        for k in obj_names:
            if k not in ins.label_clusters:
                continue
            v = ins.label_clusters[k]
            out[k] = mask2grasps(v.masks[0], nfingers=nfingers, dmin=dmin, dmax=dmax, obj_score=v.scores.flatten()[0])
        return out
    
    def visualize(self, rgb, pred):
        rgb_out = rgb.copy()
        if pred is None:
            return rgb_out
                
        for k, gg in pred.items():
            rgb_out = gg.disp(rgb_out, topn=5)
            x,y = gg.centers_2d[0, :]
            rgb_out =  show_text_on_rgb(rgb_out, k, (x,y), color=(255,0,0), thick=2)
        return rgb_out
            
        

        

