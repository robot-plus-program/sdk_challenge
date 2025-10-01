from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from pyrecognition.recognizer import InsDetector
from ketisdk.utils.proc_utils import download_gdrive_file
from ketisdk.vision.utils.image_processing import show_masks_on_rgb
import os, cv2, numpy as np
from pathlib import Path

gdrive_ids = {'detectron2_maskrcnn_box_object.pth':'1cBVF1GPAkMHg_zUObHCQDA5v_yqutHjZ',
              'mask_rcnn_R_101_FPN_3x.yaml': '1-S42vomeumQGmqHc8I2ddYKXxN4F-Gwd',
              'Base-RCNN-FPN.yaml':'1Fa2RBuGX4IcI7_-5GA-Fh-kffn0NvZ-g'}

root_dir = Path(__file__).parent.parent.parent
ckpt_dir = os.path.join(root_dir, 'ckpts')
config_dir = os.path.join(root_dir, 'configs')
os.makedirs(ckpt_dir, exist_ok=True)
out_dir = os.path.join(root_dir, 'test_images')
os.makedirs(out_dir, exist_ok=True)

name = 'Base-RCNN-FPN.yaml'
download_gdrive_file(gdrive_ids[name], os.path.join(config_dir, name))


class InstanceDetector(InsDetector):
    def init(self, ckpt='detectron2_maskrcnn_box_object.pth', config='mask_rcnn_R_101_FPN_3x.yaml',
             num_classes=1, score_thresh=0.5):
        ckpt_path, config_path = os.path.join(ckpt_dir, ckpt), os.path.join(config_dir, config)
        download_gdrive_file(gdrive_ids[ckpt], ckpt_path)
        download_gdrive_file(gdrive_ids[config], config_path)

        cfg = get_cfg()
        cfg.merge_from_file(config_path)

        cfg.MODEL.RETINANET.SCORE_THRESH_TEST = score_thresh
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = score_thresh
        cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = score_thresh
        cfg.MODEL.WEIGHTS = ckpt_path
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = num_classes
        # cfg.MODEL.RPN.NMS_THRESH = 0.5
        cfg.freeze()

        self.model = DefaultPredictor(cfg)
        print(f'{ckpt} loaded ...')

    def run(self, rgb, thresh=None, classes=None, crop_box=None):
        h,w = rgb.shape[:2]
        left, top, right, bottom = (0, 0, w, h) if crop_box is None else crop_box
        im = np.zeros_like(rgb)
        im[top:bottom, left:right, :] = rgb[top:bottom, left:right, :]

        pred = self.model(im)

        ret = dict()
        if 'instances' in pred:
            inst = pred["instances"].to("cpu")
            if inst.has("pred_boxes"): ret.update({'boxes': inst.pred_boxes.tensor.numpy()})
            if inst.has("scores"): ret.update({'scores': inst.scores.numpy().reshape((-1, 1))})
            if inst.has("pred_classes"): ret.update({'labels': inst.pred_classes.numpy()})
            if inst.has("pred_keypoints"): ret.update({'keypoints': inst.pred_keypoints.numpy()})
            if inst.has("pred_masks"): ret.update({'masks': inst.pred_masks.numpy()})
        if 'sem_seg' in pred:
            sem_seg = pred["sem_seg"].argmax(dim=0).to("cpu").numpy()
            ret.update({'sem_seg': sem_seg})

        out = show_masks_on_rgb(rgb, ret['masks'])
        cv2.imwrite(os.path.join(out_dir, 'out_instances.png'), out[..., ::-1])

        return ret

if __name__=='__main__':
    detector = InstanceDetector()
    detector.init()
    detector.demo()