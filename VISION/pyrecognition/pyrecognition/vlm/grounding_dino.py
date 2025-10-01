from PIL import Image
import cv2, os, torch, torchvision, numpy as np

import groundingdino.datasets.transforms as T
from groundingdino.models import build_model
from groundingdino.util.slconfig import SLConfig
from groundingdino.util.utils import clean_state_dict, get_phrases_from_posmap
from pyrecognition.utils import get_device, ckpt_dir, config_dir, get_device, wget_file
from pathlib import Path
from pyrecognition.recognizer import InsDetector
from pyinterfaces.instances import CustomInstances
from pyrecognition.utils import crop_image
from sklearn.cluster import KMeans

device = get_device()
ckpt_links = {'groundingdino_swint_ogc.pth':'https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth'}
__CKPT__ = 'groundingdino_swint_ogc.pth'
__CONFIG__ = 'GroundingDINO_SwinT_OGC.py'

transform = T.Compose(
        [
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ]
    )
def preprocess_image(rgb, crop_roi=None, keep_size=True):
    # h, w = rgb.shape[:2]
    # left, top, right, bottom = (0, 0, w, h) if crop_roi is None else crop_roi
    # inp = np.zeros_like(rgb)
    # inp[top:bottom, left:right, :] = rgb[top:bottom, left:right, :]

    image_pil = Image.fromarray(crop_image(rgb, crop_roi, keep_size=keep_size))
    image, _ = transform(image_pil, None)  # 3, h, w
    return image.to(device)

def load_model(ckpt=__CKPT__,
               cfg_path=__CONFIG__):
    ckpt_path = os.path.join(ckpt_dir, ckpt)
    cfg_path = os.path.join(config_dir, cfg_path)
    wget_file(filelink=ckpt_links[ckpt], filepath=ckpt_path)

    args = SLConfig.fromfile(cfg_path)
    args.device = device
    model = build_model(args)
    checkpoint = torch.load(ckpt_path, map_location="cpu")
    load_res = model.load_state_dict(clean_state_dict(checkpoint["model"]), strict=False)
    print(load_res)
    _ = model.eval()
    print(f'{ckpt_path} loaded ...')
    return model.to(device)

def preprocess_caption(caption):
    if not isinstance(caption, str):
        caption = '. '.join(caption)
    caption = caption.lower()
    caption = caption.strip()
    if not caption.endswith("."):
        caption = caption + "."
    return caption
def get_grounding_output(model, rgb, caption, box_threshold=0.1, text_threshold=0.15,iou_threshold=0.5,
                         crop_roi=None, skip_large_objs=False):
    # preprocess inputs
    caption = preprocess_caption(caption)
    image = preprocess_image(rgb, crop_roi=crop_roi, keep_size=True)

    # inferecne
    with torch.no_grad():
        outputs = model(image[None], captions=[caption])
    logits = outputs["pred_logits"].cpu().sigmoid()[0]  # (nq, 256)
    boxes = outputs["pred_boxes"].cpu()[0]  # (nq, 4)
    logits.shape[0]

    # filter output
    logits_filt = logits.clone()
    boxes_filt = boxes.clone()
    filt_mask = logits_filt.max(dim=1)[0] > box_threshold
    logits_filt = logits_filt[filt_mask]  # num_filt, 256
    boxes_filt = boxes_filt[filt_mask]  # num_filt, 4
    logits_filt.shape[0]

    H, W = rgb.shape[:2]
    for i in range(boxes_filt.size(0)):
        boxes_filt[i] = boxes_filt[i] * torch.Tensor([W, H, W, H])
        boxes_filt[i][:2] -= boxes_filt[i][2:] / 2
        boxes_filt[i][2:] += boxes_filt[i][:2]

    # get phrase
    tokenlizer = model.tokenizer
    tokenized = tokenlizer(caption)
    # build pred
    pred_phrases = []
    scores = []
    inds = []
    for i,(logit, box) in enumerate(zip(logits_filt, boxes_filt)):
        if skip_large_objs and ((box[2]-box[0]) > 0.7*W or (box[3]-box[1])> 0.7*H):
            continue
        pred_phrase = get_phrases_from_posmap(logit > text_threshold, tokenized, tokenlizer)
        # pred_phrases.append(pred_phrase + f"({str(logit.max().item())[:4]})")
        if len(pred_phrase)==0:
            continue
        inds.append(i)
        pred_phrases.append(pred_phrase)
        scores.append(logit.max().item())
    scores = torch.Tensor(scores)
    boxes_filt = boxes_filt[inds, :]


    # MNS
    boxes_filt = boxes_filt.cpu()
    # use NMS to handle overlapped boxes
    print(f"Before NMS: {boxes_filt.shape[0]} boxes")
    nms_idx = torchvision.ops.nms(boxes_filt, scores, iou_threshold).numpy().tolist()
    boxes_filt = boxes_filt[nms_idx]
    pred_phrases = [pred_phrases[idx]for idx in nms_idx]
    scores = scores[nms_idx]
    print(f"After NMS: {boxes_filt.shape[0]} boxes")
    # tags_chinese = check_tags_chinese(tags_chinese, pred_phrases)
    # print(f"Revise tags_chinese with number: {tags_chinese}")

    # Group detected object into same labels
    label_indexes = dict()
    for i, ph in enumerate(pred_phrases):
        for lb in ph.split(' '):
            label_indexes.update({lb: label_indexes[lb]+[i,] if lb in label_indexes else [i,]})

    return boxes_filt, scores, pred_phrases, label_indexes


class GroundingDino(InsDetector):
    def __init__(self, ckpt=__CKPT__, config=__CONFIG__, **kwargs):
        self.model = load_model(ckpt=ckpt, cfg_path=config)
    
    def run(self, rgb, caption='item', crop_roi=None, box_threshold=0.1, min_ratio=0, max_ratio=1,
            text_threshold=0.15,iou_threshold=0.5, do_clustering=False, **kwargs):
        boxes_filt, scores, pred_phrases, label_indexes= get_grounding_output(
            model=self.model, rgb=rgb, crop_roi=crop_roi, caption=caption, box_threshold=box_threshold,
            text_threshold=text_threshold, iou_threshold=iou_threshold
        )
        scores = scores.numpy()

        ins = CustomInstances(boxes=boxes_filt.numpy().astype('int'), imsize=rgb.shape[:2][::-1],
                              scores=scores, labels=pred_phrases)
        ins = ins.clip_ratio(min_ratio=min_ratio, max_ratio=max_ratio)
        return  self.clustering(rgb, ins) if do_clustering else ins 
    
    def clustering(self, rgb, instances, **kwargs): 
        import torchvision.models as models
        from hdbscan import HDBSCAN

        resnet = models.resnet50(pretrained=True).eval().to(device)
        backbone = torch.nn.Sequential(*list(resnet.children())[:-1])
        # backbone = self.model.backbone

        # cropped_tensors = [preprocess_image(rgb, box, keep_size=False) for box in instances.boxes]
        cropped_tensors = [preprocess_image(rgb, box, keep_size=False).unsqueeze(0) for box in instances.boxes]

        # extract features
        rois_feature = []
        for ts in cropped_tensors:
            with torch.no_grad():
                feat = backbone.forward(ts)
                rois_feature.append(feat.flatten().cpu().numpy())
        features_array = np.array(rois_feature)
        

        # # k-cluster
        num_clusters = 3  # Adjust based on the expected number of groups
        kmeans = KMeans(n_clusters=num_clusters, random_state=0)
        clusters = kmeans.fit_predict(features_array).tolist()
        
        # hdbscan = HDBSCAN(
        #     min_cluster_size=2,        # Minimum size of clusters
        #     min_samples=1,            # How conservative the clustering is
        #     cluster_selection_method='eom',  # 'eom' (Excess of Mass) or 'leaf'
        #     allow_single_cluster=True  # Allow single cluster if data supports it
        # )
        # clusters = hdbscan.fit_predict(features_array).tolist()

        instances.labels = clusters
        return instances
        



if __name__=='__main__':
    detector =  GroundingDino()
    
    caption = 'cup'
    detector.demo_single(caption=caption, show_steps=True)