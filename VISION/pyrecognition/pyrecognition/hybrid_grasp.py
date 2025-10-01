from pyrecognition.recognizer import Recognizer
from pyrecognition.instances import CustomInstances
from pyrecognition.suction.detector import SuctionDetector
from pyrecognition.grasp.kgrasp import KGraspDetector
from pyrecognition.utils import n2colormap
import numpy as np, cv2

class RPPInstances(CustomInstances):
    
    def split(self, mass_threshs=[5000, 27000], suction_cup=(31,31)):
        kernel = np.ones(tuple(suction_cup), 'uint8')
        masses_erode = [cv2.erode(m, kernel=kernel) for m in self.masks]
        masses_erode = [np.sum(m) for m in masses_erode]

        splits = [[] for _ in range(len(mass_threshs) + 1)]
        for i,(mass, mass_erode) in enumerate(zip(self.masses, masses_erode)):
            isobtained = False
            for j, th in enumerate(sorted(mass_threshs)):
                if mass_erode < 10:
                    splits[0].append(i)
                    isobtained = True
                    break
                elif mass<th:
                    splits[j].append(i)
                    isobtained = True
                    break
            if not isobtained:
                splits[-1].append(i)
        return  [self.select(inds=inds) for inds in splits]
    
class HybridGraspGroup():
    def __init__(self, suction, grasp, grasp3, ins_splits):
        self.suction, self.grasp, self.grasp3, self.ins_splits = suction, grasp, grasp3, ins_splits

    def todict(self,):
        return {
            'ins_splits': [el.todict() for el in self.ins_splits],
            'suction': None if self.suction is None else self.suction.todict(),
            'grasp': None if self.grasp is None else self.grasp.todict(),
            'grasp3': None if self.grasp3 is None else self.grasp3.todict()
        }
        

class HybridGraspDetector(Recognizer):
    def __init__(self, ins_detector=None, **kwargs):
        self.ins_detector = ins_detector
        
    def run(self, rgb, depth, cam_params=None, crop_roi=None, target_ind=None, min_mass=2000, 
            max_ratio=0.5, mass_threshs=[5000, 27000], suction_cup=(31,31), df=100, **kwargs):
        self.instances = self.ins_detector.run(rgb=rgb, crop_roi=crop_roi, min_mass=min_mass, max_ratio=max_ratio)
        self.instances = RPPInstances(issorted=True, **self.instances.todict())
        
        splits = self.instances.split(mass_threshs=mass_threshs, suction_cup=suction_cup)
        # return splits

        suction = SuctionDetector().run(rgb=rgb,depth=depth, cam_params=cam_params, 
                                              target_ind=target_ind, instances=splits[1])
        grasp = KGraspDetector().run(rgb=rgb,depth=depth, cam_params=cam_params, 
                                              target_ind=target_ind, instances=splits[0], nfingers=2)
        grasp3 = KGraspDetector().run(rgb=rgb,depth=depth, cam_params=cam_params, 
                                              target_ind=target_ind, instances=splits[2], nfingers=3, df=df)
        return HybridGraspGroup(suction=suction, grasp=grasp, grasp3=grasp3, ins_splits=splits)

    def visualize(self, rgb, pred, topn=10, **kwargs):
        out = rgb.copy()
        ins_splits = pred.ins_splits
        colors = n2colormap(3)
        for color, gtype, ins in zip(colors, ['grasp', 'suction', 'grasp3'], ins_splits):
            out = ins.visualize(rgb=out, colors=[color]*len(ins))
            
            gg = getattr(pred, gtype)
            out = out if gg is None else gg.disp (rgb=out, topn=topn)
        return out
    

    

    