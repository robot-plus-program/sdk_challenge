from pyrecognition.utils.ram_labelling import ram_exec, load_model as load_ram_model
from pyrecognition.utils.dino_grounding import get_grounding_output, load_model as load_dino_model
from pyrecognition.utils.sam_segment import sam_exec, load_predictor
from pyrecognition.recognizer import  InsDetector
from pyplanner.utils.chatgpt import matchObjToCategory, getObjCharacter

import cv2,  os
from ketisdk.vision.utils.image_processing import show_masks_on_rgb, show_mask_on_rgb
from pathlib import Path

root_dir = Path(__file__).parent.parent.parent
out_dir = os.path.join(root_dir, 'test_images')
os.makedirs(out_dir, exist_ok=True)


class InstanceDetector(InsDetector):
    def init(self):
        self.ram_model = load_ram_model()
        self.dino_model = load_dino_model()
        self.sam_predict = load_predictor()


    def run(self, rgb, crop_roi=None):
        try:
            ram_ret = ram_exec(self.ram_model, rgb, crop_roi=crop_roi)
            boxes_tensor, scores_tensor, phrases, label_indexes = get_grounding_output(
                self.dino_model, rgb, caption=ram_ret, crop_roi=crop_roi, skip_large_objs=False)
            masks = sam_exec(self.sam_predict, rgb, boxes_tensor, crop_roi=crop_roi)
            ret = {'boxes': boxes_tensor.numpy().astype('int'),
                   'masks': masks.cpu().numpy()[:, 0, ...],
                   'label_indexes': label_indexes,
                   'labels': phrases, 'scores': scores_tensor.numpy()}
            # ret.update({'im': self.visualize(rgb, ret, show_steps=True)})
            out = show_masks_on_rgb(rgb, ret['masks'])
            # cv2.imwrite(os.path.join(out_dir, 'out_instances.png'), out[..., ::-1])
        except:
            ret = None
        return ret

if __name__=='__main__':
    detector = InstanceDetector()
    detector.init()
    detector.demo(
        rgb_path='/home/keti/ros2_ws/src/data/outputs/image_processor/20241122172457_rgb_hand.png',
        show_steps=True
    )


