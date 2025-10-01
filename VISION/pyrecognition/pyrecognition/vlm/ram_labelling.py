from ram.models import ram, ram_plus
from ram import inference_ram
import torch, torchvision, os, sys, cv2
from PIL import Image
from pathlib import Path
from pyrecognition.utils.utils import crop_image

root_dir = Path(__file__).parent.parent.parent
ckpt_dir = os.path.join(root_dir, 'ckpts')
os.makedirs(ckpt_dir, exist_ok=True)

device = 'cuda' if torch.cuda.is_available() else 'cpu'
normalize = torchvision.transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                     std=[0.229, 0.224, 0.225])
transform = torchvision.transforms.Compose([
                    torchvision.transforms.Resize((384, 384)),
                    torchvision.transforms.ToTensor(), normalize
                ])
ckpt_links = {
    'ram_swin_large_14m.pth':'https://huggingface.co/spaces/xinyu1205/Tag2Text/resolve/main/ram_swin_large_14m.pth',
    'ram_plus_swin_large_14m.pth':'https://huggingface.co/xinyu1205/recognize-anything-plus-model/resolve/main/ram_plus_swin_large_14m.pth'}

def preprocess_image(rgb, crop_roi=None):
    h,w = rgb.shape[:2]
    left, top, right, bottom = (0,0,w,h) if crop_roi is None else crop_roi
    image_pil = Image.fromarray(rgb[top:bottom, left:right, :])

    raw_image = image_pil.resize((384, 384))
    return transform(raw_image).unsqueeze(0).to(device)


def load_model(checkpoint='ram_plus_swin_large_14m.pth'):
    ckpt_path = os.path.join(ckpt_dir, checkpoint)
    if not os.path.exists(ckpt_path):
        print(f'{ckpt_path} doesnot exist. Downloading ....')   # check this again
        os.system(f'wget {ckpt_links[checkpoint]} -o {ckpt_path}')

    model = ram_plus(pretrained=ckpt_path, image_size=384, vit='swin_l')
    model.eval()
    model.to(device)
    print(f'{ckpt_path} loaded ...')
    return model




def ram_exec(ram_model, rgb, crop_roi=None, multiscale=False):
    rgb_crop = crop_image(rgb, crop_roi=crop_roi, keep_size=False)
    ins = [rgb_crop]
    res = []
    if multiscale:
        h, w = rgb_crop.shape[:2]
        dx, dy = w//4, h//4
        ins.append(cv2.resize(rgb_crop[dy:-dy, dx:-dx, ...], dsize=(w,h), interpolation=cv2.INTER_CUBIC))
    for im in ins:
        raw_image = preprocess_image(im)
        res += inference_ram(raw_image, ram_model)[0].split(' | ')
    return  ', '.join(list(set(res)))

def demo(image_path):
    ram_model = load_model('ram_plus_swin_large_14m.pth')
    rgb = cv2.imread(image_path)[...,::-1]
    ret = ram_exec(ram_model, rgb)

    print(ret)
    cv2.imshow('rgb', rgb[...,::-1])
    cv2.waitKey()



if __name__=='__main__':
    demo(
        # image_path=os.path.join(root_dir,'test_images/rgb.png'),
        image_path='/home/keti/Downloads/2024-11-29 08_51_10.983.png',
    )
