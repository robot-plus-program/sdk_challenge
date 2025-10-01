import os, gdown, cv2, numpy as np, re, sys, json
from time import time
from datetime import datetime
from pathlib import Path
from scipy.ndimage import median_filter


root_dir = Path(__file__).parent.parent
ckpt_dir = os.path.join(root_dir, 'ckpts')
os.makedirs(ckpt_dir, exist_ok=True)
config_dir = os.path.join(root_dir, 'configs')
os.makedirs(config_dir, exist_ok=True)
data_dir = os.path.join(root_dir, 'data')
os.makedirs(data_dir, exist_ok=True)
test_image_dir = os.path.join(root_dir, 'test_images')
os.makedirs(test_image_dir, exist_ok=True)

def get_device(): 
    import torch
    return 'cuda' if torch.cuda.is_available() else 'cpu'



def download_gdrive_file(gdrive_id, filepath):
    import gdown
    if os.path.exists(filepath):
        print(f'{filepath} existed ...')
        return
    os.makedirs(os.path.split(filepath)[0], exist_ok=True)
    gdown.download(f'https://docs.google.com/uc?export=download&id={gdrive_id}', filepath)
    print(f'File {filepath} downloaded ...')
    
def wget_file(filelink, filepath):
    if os.path.exists(filepath):
        print(f'{filepath} existed ...')
        return
    print(f'{filepath} doesnot exist. Downloading ....')   # check this again
    os.system(f'wget {filelink} -o {filepath}')


def show_mask_on_rgb(rgb, mask):
    mask = mask if isinstance(mask, np.ndarray) else mask.detach().cpu().numpy()
    rgb = rgb if isinstance(rgb, np.ndarray) else rgb.detach().cpu().numpy()
    locs = np.where(mask>0)
    mask = np.repeat(mask[..., np.newaxis], 3, axis=-1)
    heatmap = 255-cv2.applyColorMap((255*mask).astype('uint8'), cv2.COLORMAP_JET)
    out = rgb.copy()
    out[locs] = 0.6*out[locs] + 0.4*heatmap[locs]
    return out
    # return (0.7*rgb + 0.3*heatmap).astype('uint8')
    # return (np.multiply(rgb, 1-mask) + np.multiply(heatmap, mask)).astype('uint8')

def n2colormap(n):
    return cv2.applyColorMap((np.arange(n) / n * 255).astype('uint8'), cv2.COLORMAP_JET)

def show_masks_on_rgb(rgb, masks, colors=None):
    if masks is None:
        return rgb
    if len(masks)==0:
        return rgb
    rgb = rgb if isinstance(rgb, np.ndarray) else rgb.detach().cpu().numpy()
    masks = [m if isinstance(m, np.ndarray) else m.detach().cpu().numpy() for m in masks]
    colors = n2colormap(len(masks)) if colors is None else colors
    out = rgb.copy()
    for i, (m, color) in enumerate(zip(masks, colors)):
        loc = np.where(m>0)
        out[loc] = 0.6*out[loc] + tuple((0.4*color).tolist())
        
        Y,X = loc
        # xc, yc = int(np.mean(X)), int(np.mean(Y))
        # out  = cv2.putText(out, f'{i}', (xc, yc), cv2.FONT_HERSHEY_COMPLEX, 0.4, (0,255,0), thickness=1)
    return out

def show_box_on_rgb(rgb, box, color=(0,255,0), thick=1):
    x0, y0, x1, y1 = box
    return cv2.rectangle(rgb, (x0, y0), (x1,y1), color, thick)

def show_boxes_on_rgb(rgb, boxes, color=(0,255,0), thick=1):
    out = rgb.copy()
    for box in boxes:
        out = show_box_on_rgb(out, box, color=color, thick=thick)
    return out

def show_text_on_rgb(rgb, text, org, size=0.4, color=(0,255,0), thick=1):
    return cv2.putText(rgb.copy(), text, org, cv2.FONT_HERSHEY_COMPLEX, size, color, thick)

def show_texts_on_rgb(rgb, texts, orgs, size=0.4, color=(0,255,0), thick=1):
    out  = rgb.copy()
    for text, org in zip(texts, orgs):
        out = show_text_on_rgb(out, text, org, size, color, thick)
    return out

def crop_image(im, crop_roi=None, keep_size=False):
    if crop_roi is None:
        return im
    x0, y0, x1, y1 = crop_roi
    if not keep_size:
        return im[y0:y1, x0:x1,...]
    out = np.zeros_like(im)
    out[y0:y1, x0:x1,...] = im[y0:y1, x0:x1,...]
    return out


def matto3Dmat( mat_in, Y_in, X_in, ry, rx):
    mat = np.copy(mat_in)
    X = np.copy(X_in)
    Y = np.copy(Y_in)
    h, w = mat.shape[:2]
    y_end = ry - 1
    if ry - 1 == 0: y_end = h
    x_end = rx - 1
    if x_end == 0: x_end = w

    mat = np.expand_dims(mat[ry + 1:y_end, rx + 1:x_end], axis=2)
    X = np.expand_dims(X[ry + 1:y_end, rx + 1:x_end], axis=2)
    Y = np.expand_dims(Y[ry + 1:y_end, rx + 1:x_end], axis=2)
    return np.concatenate((X, Y, mat), axis=2)

def repmat( mat, ntimes):
    size = mat.shape
    mat_dim = len(size)
    out_dim = len(ntimes)
    assert mat_dim <= out_dim
    size += (1,) * (out_dim - mat_dim)
    out = np.copy(mat)
    for dim in range(out_dim):
        if dim >= mat_dim: out = np.expand_dims(out, axis=dim)
        ntime = ntimes[dim]
        out1 = np.copy(out)
        for i in range(ntime - 1): out1 = np.concatenate((out1, out), axis=dim)
        out = np.copy(out1)
    return out

def get_3point_normals_mat(mat0, mat1, mat2):
    v1 = mat1 - mat0
    v2 = mat2 - mat0
    v = np.cross(v1, v2, axis=2)
    v_norm = np.linalg.norm(v, axis=2) +0.00001
    # v_norm = np.expand_dims(v_norm, axis=2)
    v_norm = repmat(v_norm, (1, 1, 3))
    v = np.divide(v, v_norm)
    return v

def get_mat_normal_map( mat_in, nb_mean=False):
    mat = mat_in.astype('float32')
    h, w = mat.shape[:2]
    X, Y = np.meshgrid(np.arange(0, w), np.arange(0, h))

    if nb_mean:
        M00 = matto3Dmat(mat, Y, X, 0, 0)
        Mm1m1 = matto3Dmat(mat, Y, X, -1, -1)
        M0m1 = matto3Dmat(mat, Y, X, 0, -1)
        M1m1 = matto3Dmat(mat, Y, X, 1, -1)
        M10 = matto3Dmat(mat, Y, X, 1, 0)
        M11 = matto3Dmat(mat, Y, X, 1, 1)
        M01 = matto3Dmat(mat, Y, X, 0, 1)
        Mm11 = matto3Dmat(mat, Y, X, -1, 1)
        Mm10 = matto3Dmat(mat, Y, X, -1, 0)

        v = np.zeros((h - 2, w - 2, 3, 8), 'float32')
        v[:, :, :, 0] = get_3point_normals_mat(M00, Mm1m1, M0m1)
        v[:, :, :, 1] = get_3point_normals_mat(M00, M0m1, M1m1)
        v[:, :, :, 2] = get_3point_normals_mat(M00, M1m1, M10)
        v[:, :, :, 3] = get_3point_normals_mat(M00, M10, M11)
        v[:, :, :, 4] = get_3point_normals_mat(M00, M11, M01)
        v[:, :, :, 5] = get_3point_normals_mat(M00, M01, Mm11)
        v[:, :, :, 6] = get_3point_normals_mat(M00, Mm11, Mm10)
        v[:, :, :, 7] = get_3point_normals_mat(M00, Mm10, Mm1m1)
        v_mean = np.mean(v, axis=3)

        v_norm = np.linalg.norm(v_mean, axis=2)
        v_norm = repmat(v_norm, (1, 1, 3))
        v_norm = np.divide(v_mean, v_norm)


    else:
        Mm1m1 = matto3Dmat(mat, Y, X, -1, -1)
        M1m1 = matto3Dmat(mat, Y, X, 1, -1)
        M01 = matto3Dmat(mat, Y, X, 0, 1)
        v_norm = get_3point_normals_mat(M01, Mm1m1, M1m1)

    v = np.zeros((h, w, 3), 'float32')
    v[1:-1, 1:-1, :] = np.copy(v_norm)

    v[0, 1:-1, :] = np.copy(v_norm[0, :, :])  # copy to boundary
    v[1:-1, 0, :] = np.copy(v_norm[:, 0, :])
    v[-1, 1:-1, :] = np.copy(v_norm[-1, :, :])
    v[1:-1, -1, :] = np.copy(v_norm[:, -1, :])
    v[(0, 0)], v[(0, -1)], v[(-1, 0)], v[(-1, -1)] = v_norm[(0, 0)], v_norm[(0, -1)], v_norm[(-1, 0)], v_norm[(-1, -1)]
    return v

def get_mat_normal_map_U8( mat_in, nb_mean=False):
    return np.abs(255 * get_mat_normal_map(mat_in=mat_in, nb_mean=nb_mean)).astype('uint8')

class Timer():
    def __init__(self):
        self.reset()

    def reset(self):
        self.times = [time(),]
        self.labels = []

    def len(self):
        return len(self.times)

    def pin_time(self, label='NaN'):
        self.add_time(label=label)
        return self.times[-1] - self.times[-2]

    def add_time(self, label='NaN'):
        self.times.append(time())
        self.labels.append(label)

    def run_time(self):
        if self.len()<2: self.add_time()
        return self.times[-1] - self.times[0]

    def fps(self):
        return 1/self.run_time()

    def run_time_str(self):
        runtime=self.run_time()
        if self.run_time() < 1: return'Total:%dms' % (1000*runtime)
        else:  return 'Total:%.2fs' %runtime

    def pin_times_str(self):
        if self.len()<2: return self.run_time_str()
        s = ''
        for i in range(self.len()-1):
            label  = self.labels[i]
            if label == 'NaN': continue
            dtime = self.times[i+1]-self.times[i]
            if dtime < 1: s += '%s:%dms-' %(self.labels[i], 1000*dtime)
            else: s += '%s:%.2fs-' % (self.labels[i], dtime)

        # if self.run_time() < 1:s+='Total:%dms' % (1000*self.run_time())
        # else:s+='Total:%.2fs' % self.run_time()
        s += self.run_time_str()

        return s
    

def twopoints2theta(P0, P1):
    '''
    V0, V1: nxd ndarray
    '''
    V = P1-P0    
    return np.arctan2(np.roll(V, -1), V)

def findNumdersInString(sentence):
    return [eval(el) for el in re.findall(r'\d+', sentence)]
    
def strftime():
    return datetime.now().strftime('%Y%m%d%H%M%S%f')

def evaluate(v):
    v = v.replace(' ', '')
    if ',' not in v:
        try:  return eval(v) 
        except:  return v
    return [evaluate(el) for el in v.split(',') if len(el)>0]
    
def parse_keys_values(optional_args={}):
    
    
    args = sys.argv[1:] 
    args = [arg.split('=') for arg in args  if '=' in arg]
    args = {k: evaluate(v) for k, v in args}
    
    # optional_args = [arg.split('=') for arg in optional_args  if '=' in arg]
    # optional_args = {k: evaluate(v) for k, v in optional_args}
    
    optional_args.update(args)
    print(f'{"="*10} Optional args: {", ".join([f"{k}={v}" for k,v in optional_args.items()])}')
    
    return optional_args

def visualize_insdetect(rgb, pred, show_steps=False, colors=None):
    out = rgb.copy()
    if pred is None:
        return out

    colors = n2colormap(len(pred))
    for i,(lb, ins) in enumerate(pred.items()):
        if len(ins['boxes'])==0:
            return out
        has_mask = False
        if 'masks' in ins:
            has_mask = ins['masks'] is not None
        out = show_masks_on_rgb(rgb=out, masks=ins['masks'], colors=[colors[i]]*len(ins)) if has_mask else show_boxes_on_rgb(rgb=out, boxes=ins['boxes'])
        
        orgs = [[(x0+x1)//2, (y0+y1)//2] for x0,y0,x1,y1 in ins['boxes']]
        texts = [f'{i}:{lb}:{sc:.2f}' if show_steps else f'{i}' for i,sc in enumerate(ins['scores'])]
        out = show_texts_on_rgb(rgb=out, texts=texts, orgs=orgs, size=0.4, thick=1)
        
        target_ind =  ins['target_ind'] if 'target_ind' in pred else 0
        if target_ind is not None:
            # if not has_mask:
            out = show_box_on_rgb(rgb=out, box=ins['boxes'][target_ind][:4].astype('int'),color=(255,0,0), thick=3)
            out = show_text_on_rgb(rgb=out, text=f'{target_ind}:{lb}', 
                                    org=orgs[target_ind], size=0.6, color=(255,0,0), thick=1)
    return out


def depth2xyz(depth, cam_params, crop_roi=None, denoiseKSize=None):
    fx, fy, xc, yc = cam_params[:4]
    h, w = depth.shape[:2]

    X, Y = np.meshgrid(range(w), range(h))
    X = X.astype('float32') - xc
    Y = Y.astype('float32') - yc
    Z = depth.astype('float32')
    if denoiseKSize is not None:
        left, top, right, bottom = (0,0,w,h) if crop_roi is None else crop_roi
        Z_crop = Z[top:bottom, left:right]
        invalid_locs = np.where(Z_crop < 10)
        Z_crop_med = median_filter(Z_crop, size=denoiseKSize)
        Z_crop[invalid_locs] = Z_crop_med[invalid_locs]
        Z[top:bottom, left:right] = Z_crop

    X_, Y_ = np.multiply(X, Z) / fx, np.multiply(Y, Z) / fy
    XYZ = np.concatenate((X_[..., np.newaxis], Y_[..., np.newaxis], Z[..., np.newaxis]), axis=2)
    return XYZ

def xyz2normal( xyz, r=3):
    h,w = xyz.shape[:2]

    xyz_mrmr = xyz[:-2*r, :-2*r, :]
    xyz_rmr = xyz[2*r:, :-2*r, :]
    xyz_0r = xyz[r:-r, 2*r:, :]
    V0, V1 = xyz_mrmr - xyz_0r, xyz_rmr - xyz_0r

    V2 = np.cross(V0, V1, axis=-1)
    V2 = np.divide(V2, np.linalg.norm(V2, axis=-1, keepdims=True) + 1e-10)
    V2 = np.pad(V2, pad_width=((r,r), (r,r), (0,0)),mode='edge')
    return V2


def depth2normal(depth, cam_params, crop_roi=None, denoiseKSize=None, r=3):
    xyz = depth2xyz(depth=depth, cam_params=cam_params, crop_roi=crop_roi, denoiseKSize=denoiseKSize)
    normal = xyz2normal(xyz=xyz, r=r)
    return normal

def xyz2Ixy(x,y,z, cam_params, eps=1e-10):
    fx, fy, xc, yc = cam_params[:4]
    Ix =  np.divide(x, z + eps) * fx + xc
    Iy =  np.divide(y, z + eps) * fy + yc
    return int(Ix), int(Iy)

def Ixy2xyz(Ix, Iy, Z, cam_params):
    fx, fy, xc, yc = cam_params[:4]
    Ix, Iy = Ix - xc, Iy - yc
    return np.multiply(Ix, Z) / fx, np.multiply(Iy, Z) / fy, Z

def calc_normalvector(points):
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    cov_matrix = np.cov(centered_points, rowvar=False)
    _, _, vh = np.linalg.svd(cov_matrix)
    normal_vector = vh[-1]  # The last singular vector is the normal
    return normal_vector

def write_json(filepath, adict):
    assert os.path.splitext(filepath)[-1]=='.json'
    with open(filepath, 'w') as f:
        json.dump(adict, f)
        
def read_json(filepath):
    assert os.path.splitext(filepath)[-1]=='.json'
    with open(filepath) as f:
        return json.load(f)

def calc_rect_area(rect):
    return (rect[2] - rect[0]) * (rect[3] - rect[1])

def calc_rect_areas(rects):
    return (rects[:, 2] - rects[:, 0]) * (rects[:, 3] - rects[:, 1])

get_2rects_intersect = lambda rect0, rect1: [
    max(rect0[0], rect1[0]), max(rect0[1], rect1[1]), 
    min(rect0[2], rect1[2]), min(rect0[3], rect1[3])]

def get_1xn_rect_intersets(rect, rects):
    """Calculates the intersection rectangles between a single rectangle and multiple rectangles."""
    x1 = np.maximum(rect[0], rects[:, 0])
    y1 = np.maximum(rect[1], rects[:, 1])
    x2 = np.minimum(rect[2], rects[:, 2])
    y2 = np.minimum(rect[3], rects[:, 3])

    intersection_rects = np.stack([x1, y1, x2, y2], axis=-1)
    return intersection_rects

def calc_iou(rect0, rect1):
    area0, area1 = calc_rect_area(rect0), calc_rect_area(rect1)
    areai = calc_rect_area(get_2rects_intersect(rect0, rect1))
    return area1/(area0+area1-areai + 1e-10)

def calc_iou_1xn(rect, rects):
    area, areas = calc_rect_area(rect), calc_rect_areas(rects)

    intersection_rects = get_1xn_rect_intersets(rect, rects)
    intersection_widths = np.maximum(0, intersection_rects[:, 2] - intersection_rects[:, 0])
    intersection_heights = np.maximum(0, intersection_rects[:, 3] - intersection_rects[:, 1])
    intersection_areas = intersection_widths * intersection_heights

    union_areas = area + areas - intersection_areas
    iou = intersection_areas / (union_areas + 1e-10) 
    return iou    


if __name__=='__main__':
    # P0 = np.array([[0,0,0], [0,0,0]])
    # P1 = np.array([[1,1,1], [1, np.sqrt(3), 1]])    
    # thetas = twopoints2theta(P0, P1)
    # print(thetas * 180 / np.pi)
    
    # ret = findNumdersInString('Found 2 cups')
    # print(findNumdersInString('No found cup'))
    print(strftime())
