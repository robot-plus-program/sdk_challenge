import os, torch, numpy as np
from segment_anything import build_sam, build_sam_hq, SamPredictor
from pathlib import Path

ckpt_links = {'sam_vit_h_4b8939.pth':'https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth'}

device = 'cuda' if torch.cuda.is_available() else 'cpu'
root_dir = Path(__file__).parent.parent.parent
ckpt_dir = os.path.join(root_dir, 'ckpts')
os.makedirs(ckpt_dir, exist_ok=True)

def load_predictor(ckpt='sam_vit_h_4b8939.pth', use_sam_hq=False):

    ckpt_path = os.path.join(ckpt_dir, ckpt)
    if not os.path.exists(ckpt_path):
        print(f'{ckpt_path} doesnot exist. Downloading ....')  # check this again
        os.system(f'wget {ckpt_links[ckpt]} -o {ckpt_path}')

    # initialize SAM
    if use_sam_hq:
        print("Initialize SAM-HQ Predictor")
        return SamPredictor(build_sam_hq(checkpoint=ckpt_path).to(device))
    return SamPredictor(build_sam(checkpoint=ckpt_path).to(device))



def sam_exec(predictor, rgb, boxes_tensor, crop_roi=None):
    h, w = rgb.shape[:2]
    left, top, right, bottom = (0, 0, w, h) if crop_roi is None else crop_roi
    inp = np.zeros_like(rgb)
    inp[top:bottom, left:right, :] = rgb[top:bottom, left:right, :]

    predictor.set_image(inp)
    transformed_boxes = predictor.transform.apply_boxes_torch(boxes_tensor, rgb.shape[:2]).to(device)
    masks, _, _ = predictor.predict_torch(
        point_coords=None,
        point_labels=None,
        boxes=transformed_boxes.to(device),
        multimask_output=False,
    )
    return masks

if __name__=='__main__':
    load_predictor()