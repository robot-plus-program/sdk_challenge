import numpy as np
from pyrecognition.utils.ram_labelling import ram_exec, load_model as load_ram_model
from pyrecognition.utils.dino_grounding import get_grounding_output, load_model as load_dino_model
from pyrecognition.instance_detection.fast_sam import get_net as  get_fastsam_net, fastsam_prompt_process, fastsam_exec
from pyrecognition.recognizer import  InsDetector
from pyplanner.utils.chatgpt import matchObjToCategory, getObjCharacter

import cv2,  os, torch
from ketisdk.vision.utils.image_processing import show_masks_on_rgb, show_mask_on_rgb
from pathlib import Path


device = 'cuda' if torch.cuda.is_available() else 'cpu'

root_dir = Path(__file__).parent.parent.parent
out_dir = os.path.join(root_dir, 'test_images')
os.makedirs(out_dir, exist_ok=True)


class InstanceDetector(InsDetector):
    def init(self):
        self.ram_model = load_ram_model()
        self.dino_model = load_dino_model()
        self.sam_model = get_fastsam_net()


    def run(self, rgb, crop_roi=None):
        # try:
        ram_ret = ram_exec(self.ram_model, rgb, crop_roi=crop_roi, multiscale=False)
        # ram_ret = 'alcohol, beverage, bottle, box, cardboard box, smartphone, container, control, table, electronic, fruit, juice, lime, orange, phone, remote, shelf, sit, soda'
        boxes_tensor, scores_tensor, phrases, label_indexes = get_grounding_output(
            self.dino_model, rgb, caption=ram_ret, crop_roi=crop_roi, skip_large_objs=True)

        boxes = boxes_tensor.numpy().astype('int').tolist()
        sam_ret = fastsam_exec(self.sam_model, rgb=rgb, crop_roi=crop_roi)
        prompt_process = fastsam_prompt_process(rgb=rgb, everything_results=sam_ret, crop_roi=crop_roi)
        masks = prompt_process.box_prompt(bboxes=boxes)
        ret = {'boxes': np.array(boxes),
               'masks': masks,
               'label_indexes': label_indexes,
               'labels': phrases,
               'scores': scores_tensor.numpy()}
        # out = show_masks_on_rgb(rgb, ret['masks'])
        # except Exception as e:
        #     print(e)
        #     ret = None
        return ret

if __name__=='__main__':
    detector = InstanceDetector()
    detector.init()
    detector.demo(
        # rgb_path='/home/keti/ros2_ws/src/data/outputs/image_processor/20241122172457_rgb_hand.png',
        # rgb_path='/home/keti/ros2_ws/src/data/outputs/image_processor/20241126164759_rgb.png',
        # rgb_path='/home/keti/Downloads/2024-11-29 08_51_10.983.png',
        # rgb_path='/home/keti/Downloads/2024-11-29 08_50_59.452.png',
        rgb_path='/media/keti/workdir/projects/pyrecognition/data/logs/vision/20241216160408599135_20241216161453438375_rgb.png',
        # rgb_path='/media/keti/workdir/projects/pyrecognition/data/logs/vision/20241216160408599135_20241216162209680870_rgb.png',
        show_steps=True,
        crop_roi=[180, 50, 1000, 690]
    )


