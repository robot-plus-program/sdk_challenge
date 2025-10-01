from ketisdk.utils.proc_utils import argwhere_sampling, Timer
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
from ketisdk.vision.utils.image_processing import show_mask_on_rgb, show_masks_on_rgb
from pyrecognition.grasppose import GraspGroup
from itertools import combinations
import torch, cv2, numpy as np, os
from pyrecognition.grasppose import RGBDtensor
from pathlib import Path

root_dir = Path(__file__).parent.parent.parent
out_dir = os.path.join(root_dir, 'test_images')
os.makedirs(out_dir, exist_ok=True)

use_torch = False
device = 'cuda' if torch.cuda.is_available() else 'cpu'

func_anorm = lambda x: np.linalg.norm(x, axis=-1, keepdims=True) if not use_torch else torch.norm(x, dim=-1,
                                                                                                  keepdim=True)
func_asum = lambda x: np.sum(x, axis=-1, keepdims=True) if not use_torch else torch.sum(x, dim=-1, keepdim=True)
func_amin = lambda x: np.amin(x, axis=-1, keepdims=True) if not use_torch else torch.amin(x, dim=-1, keepdim=True)
func_amean = lambda x: np.mean(x, axis=-1, keepdims=True) if not use_torch else torch.mean(x, dim=-1, keepdim=True)
func_amean0 = lambda x: np.mean(x, axis=0, keepdims=True) if not use_torch else torch.mean(x, dim=0, keepdim=True)
func_concat = lambda x: np.concatenate(x, axis=-1) if not use_torch else torch.cat(x, dim=-1)
func_multiply = np.multiply if not use_torch else torch.mul
func_min = np.amin if not use_torch else torch.amin
func_max = np.amax if not use_torch else torch.amax
func_divide = np.divide if not use_torch else torch.divide
func_where = np.where if not use_torch else torch.where
func_argsort = np.argsort if not use_torch else torch.argsort
func_ones_like = np.ones_like if not use_torch else torch.ones_like
func_reshape = np.reshape if not use_torch else torch.reshape
func_logical_and = np.logical_and if not use_torch else torch.logical_and
func_clip = np.clip if not use_torch else torch.clip


def optimization(xyz, normal_vector, pixel_locs, contact_map, collision_map, nfingers=2,
                 friction_coef=1., dmin=10, dmax=80, obj_ind=0, eps=1e-10):
    timer = Timer()
    n = xyz.shape[0]
    inds = list(combinations(range(n), nfingers)) if not use_torch else torch.combinations(torch.arange(n),
                                                                                           nfingers).to(device)
    timer.pin_time('indexing')

    V, N = xyz[inds, :], normal_vector[inds, :]

    points = [V[:, i, :] for i in range(nfingers)]
    normals = [N[:, i, :] for i in range(nfingers)]

    locs = pixel_locs[inds, :]
    pixel_locs = [locs[:, i, :] for i in range(nfingers)]

    timer.pin_time('fingers')

    # calculate distances between fingers
    finger_pair_inds = list(combinations(range(nfingers), 2))
    Ds = [points[i] - points[j] for i, j in finger_pair_inds]
    Ds = [func_anorm(el) for el in Ds]
    # Dmean = sum(Ds)/nfingers
    # Dmean /= np.amax(Dmean)
    timer.pin_time('finger-distances')

    # calculate forces
    Xmean = sum(points) / nfingers
    DD = [el - Xmean for el in points]
    Fs = [func_divide(el, func_anorm(el) + 1e-10) for el in DD]

    # Xmean = sum(points) / nfingers
    # DD = [el-Xmean for el in points]
    # # calculate distances between fingers
    # Ds = [np.linalg.norm(el, axis=-1, keepdims=True) for el in DD]
    # Dmean = sum(Ds) / nfingers
    # Dmean /= np.amax(Dmean)
    #
    # # calculate forces
    # Fs = [np.divide(el, np.linalg.norm(el, axis=-1, keepdims=True) + 1e-10) for el in DD]

    # calculate force closure
    Ins = [func_asum(func_multiply(f, n)) for f, n in zip(Fs, normals)]
    # Ins = np.amin(np.concatenate([np.multiply(f, n) for f,n in zip(Fs, normals)], axis=-1), axis=-1, keepdims=True)
    timer.pin_time('force-closure')

    # optimization
    M = True
    for el in Ds:
        M &= (dmin < el) & (el < dmax)

    kf = np.cos(np.arctan(friction_coef))
    for el in Ins:
        M &= el > kf

    inds = func_where(M)[0].tolist()
    if len(inds) == 0:
        return None
    timer.pin_time('check-closure')

    # min distances between fingers
    Dmin = func_amin(func_concat([el[inds, :] for el in Ds]))
    Dmin /= (func_max(Dmin) + eps)

    # average distances of contact points and center
    Dmean = sum(func_asum(el[inds, :]) for el in DD) / nfingers
    Dmean /= (func_max(Dmean) + eps)

    # center scores
    center = func_amean0(points[0])
    center_scores = func_anorm(Xmean[inds, :] - center)
    center_scores = 1 - center_scores / (func_max(center_scores) + eps)

    # contact score, collision score
    locs = [el[inds, :] for el in pixel_locs]
    locs = [(el[:, 0], el[:, 1]) for el in locs]
    if use_torch:
        contact_map, collision_map = torch.tensor(contact_map).to(device), torch.tensor(collision_map).to(device)
    # center_scores = sum(center_score_map[el] for el in locs).reshape(-1,1)/nfingers
    contact_scores = sum(contact_map[el] for el in locs).reshape(-1, 1) / nfingers
    collision_scores = 1 - sum(collision_map[el] for el in locs).reshape(-1, 1) / nfingers

    # contact_scores = func_reshape(sum(contact_map[el] for el in locs), (-1, 1)) / nfingers
    # collision_scores = 1 - func_reshape(sum(collision_map[el] for el in locs), (-1, 1)) / nfingers

    Score = (0.6 * sum(Ins)[inds, :] / nfingers + 0.1 * (1 - Dmean) + 0.05 * Dmin +
             0.1 * center_scores + 0.05 * contact_scores + 0.1 * collision_scores)

    # sort and get topn
    # argsort = func_argsort(Score.flatten()).tolist()[::-1]
    # inds = func_where(Score>thresh)[0].tolist()
    Score = func_clip(Score, 0,1)
    Xs = [el[inds, :] for el in points]
    surface_normals = [el[inds, :] for el in normals]

    # # get grasp group
    # Obj_ind = obj_ind * func_ones_like(Score)
    # Xs = [el[inds, :][argsort, :] for el in points]
    #
    # gg_arr = func_concat([Obj_ind,] + Xs + [Score,])
    # gg = GraspGroup(arr=gg_arr if not use_torch else gg_arr.detach().cpu().numpy())

    contact_points, surface_normals = func_concat(Xs),  func_concat(surface_normals)
    if use_torch:
        Score = Score.detach().cpu().numpy()
        contact_points = contact_points.detach().cpu().numpy()
        surface_normals = surface_normals.detach().cpu().numpy()
    Obj_ind = obj_ind * np.ones_like(Score).astype('int')

    gg = GraspGroup(contact_points=contact_points, surface_normals=surface_normals,
                    obj_ids=Obj_ind, scores=Score)

    timer.pin_time('optimize')

    print(timer.pin_times_str())

    return gg


def get_grasp_single(xyz, normal_vector, contact_map, collision_map, pixel_locs, nfingers=2,
                     dmin=10, dmax=80, deg_stride=3, friction_coef=1., eps=3, obj_ind=0):
    # lefttop = np.array(lefttop)

    n = xyz.shape[0]
    if n < 5:
        print('Empty ...')
        return None

    gg = optimization(xyz, normal_vector, pixel_locs=pixel_locs, nfingers=nfingers, contact_map=contact_map,
                      collision_map=collision_map, friction_coef=friction_coef, dmin=dmin, dmax=dmax, obj_ind=obj_ind)

    return gg


def preprocess_data(rgb, depth, masks, cam_params):
    masks_dilate = [cv2.dilate((m>0).astype('uint8'), np.ones((11, 11), 'uint8')) for m in masks]
    collision_map = cv2.blur((sum(masks_dilate) >=2).astype('float32'), (21, 21))
    cv2.imwrite(os.path.join(out_dir, 'collision_map.png'), show_mask_on_rgb(rgb, collision_map)[...,::-1])
    if use_torch:
        collision_map = torch.tensor(collision_map).to(device)
        # contact_map = torch.tensor(contact_map).to(device)
        masks_dilate = [torch.tensor(m).to(device) for m in masks_dilate]

    if not use_torch:
        rgbd = RGBD(rgb=rgb, depth=depth)
        xyz = rgbd.xyz(intr_params=cam_params)
        normal_map = rgbd.normal_vector_map(xyz=xyz)
    else:
        rgbd_tensor = RGBDtensor(rgb=rgb, depth=depth, device=device)
        xyz = rgbd_tensor.xyz(cam_params=cam_params)
        normal_map = rgbd_tensor.normal_map(xyz=xyz)

    cv2.imwrite(os.path.join(out_dir, 'out_collision_map.png'), show_mask_on_rgb(
        rgb, collision_map if not use_torch else collision_map.detach().cpu().numpy())[..., ::-1])
    # cv2.imwrite(os.path.join(out_dir, 'out_instances.png'), show_masks_on_rgb(rgb=rgb, masks=masks)[...,::-1])
    return xyz, normal_map, collision_map, masks_dilate


def get_contact_map(mask, ins_mask, kernel_size=(15, 15), rgb=None):
    kernel = np.ones(kernel_size, 'uint8')
    contact_map = cv2.dilate(mask, kernel) - cv2.erode(mask, kernel)
    if use_torch:
        contact_map = torch.tensor(contact_map).to(device)
    # contacts_sampling = [argwhere_sampling(func_logical_and(m > 0, contact_map > 0), deg=5, stride=5)
    #                      for m in ins_masks]
    # pixel_locs_list = [func_concat([el.reshape(-1, 1) for el in loc]) for loc in contacts_sampling]
    contact_sampling = argwhere_sampling(func_logical_and(ins_mask > 0, contact_map > 0), deg=5, stride=5)
    pixel_locs = func_concat([el.reshape(-1, 1) for el in contact_sampling])

    if rgb is not None:
        cv2.imwrite(os.path.join(out_dir, 'out_contact_map.png'), show_mask_on_rgb(rgb, contact_map)[..., ::-1])
    return contact_map, contact_sampling, pixel_locs

def get_2d_normals_from_mask(mask, r=3):
    kernel = np.ones((3,3), 'uint8')
    mask = (mask>0).astype('uint8')
    contact_map = cv2.dilate(mask, kernel) - cv2.erode(mask, kernel)

    ones_x, zeros_x = np.ones((2*r+1, r), 'float32'), np.zeros((2*r+1, 1), 'float32')
    ones_y,zeros_y = ones_x.transpose(), zeros_x.transpose()

    kernel_x = np.concatenate((-ones_x, zeros_x, ones_x),axis=-1)
    kernel_y = np.concatenate((-ones_y, zeros_y, ones_y),axis=0)

    Gx = cv2.filter2D(contact_map, ddepth=-1, kernel=kernel_x)
    Gy = cv2.filter2D(contact_map, ddepth=-1,  kernel=kernel_y)

    G =  np.concatenate((Gx[...,np.newaxis], Gy[...,np.newaxis]), axis=-1)
    G = np.divide(G, np.linalg.norm(G, axis=-1, keepdims=True) +  1e-10)

    Y,X = argwhere_sampling(contact_map, deg=5, stride=1)
    normals = G[(Y,X)]
    points0 = np.concatenate([X.reshape(-1,1), Y.reshape(-1,1)], axis=-1)
    points1 = (points0 + 10*normals).astype('int')

    out = cv2.cvtColor(255*contact_map, cv2.COLOR_GRAY2BGR)
    for p0, p1 in zip(points0, points1):
        out = cv2.line(out, tuple(p0), tuple(p1), (0,255,0), 2)

    cv2.imwrite(os.path.join(out_dir, 'gradient.png'), out)





if __name__=='__main__':
    instances = np.load(os.path.join(out_dir, 'instances.npy'), allow_pickle=True).tolist()
    masks = instances['masks']
    get_2d_normals_from_mask(masks[5])





