import cv2, numpy as np, torch, os
from itertools import combinations
from pyinterfaces.grasppose import GraspGroup
from pyrecognition.recognizer import GraspDetect
from pyrecognition.utils import show_masks_on_rgb, crop_image
from pathlib import Path

root_dir = Path(__file__).parent.parent.parent
out_dir = os.path.join(root_dir, "data/outputs")
os.makedirs(out_dir, exist_ok=True)

test_dir = os.path.join(root_dir, "test_images")

device = "cuda" if torch.cuda.is_available() else "cpu"

torch_arctan2 = lambda y, x: torch.arctan(torch.divide(y, x + 1e-10)) + (
    x < 0
) * torch.pi * torch.sign(y)


def mask2normalmap(mask, r=2, eps=1e-10):

    mask = mask.astype("float32")
    ones = np.ones((r, 2 * r + 1), "float32")
    zeros = np.zeros((1, 2 * r + 1), "float32")

    kernel_y = np.concatenate((ones, zeros, -ones), axis=0)
    kernel_x = kernel_y.transpose()

    gx = cv2.filter2D(mask, ddepth=-1, kernel=kernel_x)
    gy = cv2.filter2D(mask, ddepth=-1, kernel=kernel_y)
    g = np.concatenate((gx[..., np.newaxis], gy[..., np.newaxis]), axis=-1)

    gnorm = np.linalg.norm(g, axis=-1, keepdims=True) + eps

    return np.divide(g, gnorm)


def show_normmalmap(rgb, normalmap, dr=20):
    out = rgb.copy()
    gx, gy = normalmap[..., 0], normalmap[..., 1]
    valid_locs = np.where((gx != 0) | (gy != 0))
    Y, X = valid_locs

    for x, y, ax, ay in zip(X, Y, gx[valid_locs], gy[valid_locs]):
        x1, y1 = int(x + ax * dr), int(y + ay * dr)
        cv2.line(out, (x, y), (x1, y1), (0, 255, 0), 1)

    return out


def draw_masks():

    mask0 = np.zeros((400, 600), "uint8")
    mask0 = cv2.ellipse(
        mask0,
        (150, 100),
        (100, 80),
        0,
        startAngle=0,
        endAngle=360,
        color=(1, 1, 1),
        thickness=-1,
    )

    mask1 = np.zeros_like(mask0)
    mask1 = cv2.rectangle(mask1, (300, 200), (450, 300), (1, 1, 1), thickness=-1)

    mask_all = np.bitwise_or(mask0, mask1)
    return [mask0, mask1], mask_all


def demo_normalmap():
    masks, mask_all = draw_masks()

    out = cv2.cvtColor(255 * mask_all, cv2.COLOR_GRAY2BGR)
    for mask in masks:
        normalmap = mask2normalmap(mask)
        out = show_normmalmap(out, normalmap=normalmap)

    cv2.imshow("normal map", out)
    cv2.waitKey()


def sampling(X, deg=5, stride=5):
    Xc = torch.mean(X, dim=0, keepdim=True)
    X -= Xc
    Rho = torch.norm(X, dim=1)
    Phi = torch_arctan2(X[:, [1]], X[:, [0]])
    Phi = Phi * 180 / np.pi + 180

    loc = torch.where(Phi % deg < 1)[0]
    Rho, Phi = Rho[loc].reshape(-1, 1), Phi[loc].reshape(-1, 1)

    loc = torch.where(Rho % stride < 1)[0]
    Rho, Phi = Rho[loc].reshape(-1, 1), Phi[loc].reshape(-1, 1)

    Phi = (Phi - 180) * np.pi / 180
    cosp, sinp = torch.cos(Phi), torch.sin(Phi)
    D = torch.cat((cosp, sinp), dim=-1)

    X = Xc + torch.multiply(Rho, D)

    return X.to(torch.int64)


def mask2grasps(mask=None,normalmap=None, contact_map=None, nfingers=2, friction_coef=1, dmin=10,
    dmax=150, df=80, eps=1e-10,  obj_ind=-1,  obj_score=1.0,):
    assert mask is not None or normalmap is not None
    if normalmap is None: 
        normalmap = torch.from_numpy(mask2normalmap(mask)).to(device)

    normalmap_norm = torch.norm(normalmap, dim=-1)
    locs = torch.nonzero(normalmap_norm)[:, [1, 0]].to(torch.float32)
    locs = sampling(locs)
    X, Y = locs[..., 0], locs[..., 1]
    n = X.shape[0]

    normalmap = normalmap[(Y, X)]
    contact_scores = contact_map[(Y, X)] if contact_map is not None else None

    locs = locs.to(torch.float32)
    inds = torch.combinations(torch.arange(n), nfingers).to(device)

    V, N = locs[inds, :], normalmap[inds, :]
    contact_scores = contact_scores[inds, :] if contact_map is not None else None

    points = [V[:, i, :] for i in range(nfingers)]
    normals = [N[:, i, :] for i in range(nfingers)]
    contact_scores = (
        [contact_scores[:, i, :] for i in range(nfingers)]
        if contact_map is not None
        else None
    )

    # calculate distances between fingers
    finger_pair_inds = list(combinations(range(nfingers), 2))
    Ds = [points[i] - points[j] for i, j in finger_pair_inds]
    Ds = [torch.norm(el, dim=-1, keepdim=True) for el in Ds]

    # calculate forces
    Xmean = sum(points) / nfingers
    DD = [el - Xmean for el in points]
    Fs = [torch.divide(el, torch.norm(el, dim=-1, keepdim=True) + 1e-10) for el in DD]


    # calculate force closure
    Ins = [
        torch.sum(torch.mul(f, n), dim=-1, keepdim=True) for f, n in zip(Fs, normals)
    ]

    # condition 1:  force closure
    M = True
    kf = np.cos(np.arctan(friction_coef))
    for el in Ins:
        M &= el > kf

    # condition 2: gripper width within a range
    for el in Ds:
        M &= (dmin < el) & (el < dmax)

    # condition 3: distances between other fingers to finger 0 should similar
    for el in Ds[1:nfingers-1]:
        M &= torch.abs(el - Ds[0]) < 10

    # condition 4: distance betweeen 2fingers (not including thumb) should be small
    for el in Ds[nfingers-1:]:
        M &= ((df-10)<el) & (el < (df+10))
    

    inds = torch.where(M)[0].tolist()
    if len(inds) == 0:
        return None

    Ins_scores = sum([el[inds, :] for el in Ins]) / nfingers

    # min distances between fingers
    Dmin = torch.amin(
        torch.cat([el[inds, :] for el in Ds], dim=-1), dim=-1, keepdim=True
    )
    Dmin /= torch.amax(Dmin) + eps

    # Distances of grasp centers and object center
    center = torch.mean(locs, dim=0, keepdim=True)
    Dcenter = torch.norm(Xmean[inds, :] - center, dim=-1, keepdim=True)
    Dcenter = Dcenter / (torch.amax(Dcenter) + eps)

    # contact scores
    contact_scores = contact_scores[inds, :] if contact_map is not None else 1    

    # orientation score
    orientation_score = 0.4 + 0.6*torch.abs(Fs[0][inds, :][:,[0]])

    # Score = 0.7*Ins_scores + 0.1*(1-Dmean) + 0.1 * (1-Dmin) + 0.1 * center_scores
    Score = 0.4 * Ins_scores + 0.3 * contact_scores + 0.15 * obj_score + 0.05 * (1 - Dmin) + 0.05 * (1 - Dcenter) + 0.05*orientation_score
    Score = torch.clip(Score, 0, 1)
    contact_points = torch.cat([el[inds, :] for el in points], dim=-1)
    surface_normals = torch.cat([el[inds, :] for el in normals], dim=-1)

    Score, contact_points, surface_normals = (
        Score.cpu().numpy(),
        contact_points.cpu().numpy(),
        surface_normals.cpu().numpy(),
    )
    Obj_ind = obj_ind * np.ones_like(Score).astype("int")

    gg = GraspGroup(contact_points=contact_points.astype("int"), surface_normals=surface_normals, 
                    obj_ids=Obj_ind, scores=Score, is2d=True,
    )

    return gg


def masks2grasps(masks=None,normalmaps=None, contact_map=None, nfingers=2,friction_coef=1,
    dmin=10, dmax=150, df=80, eps=1e-10, obj_scores=None):
    assert masks is not None or normalmaps is not None
    masks = [None] * len(normalmaps) if masks is None else masks
    normalmaps = [None] * len(masks) if normalmaps is None else normalmaps

    obj_scores = [1.0] * len(masks) if obj_scores is None else obj_scores
    gg = None
    for j, (mask, score) in enumerate(zip(masks, obj_scores.flatten())):
        gg_ = mask2grasps(mask=mask, contact_map=contact_map, nfingers=nfingers, friction_coef=friction_coef,
            dmin=dmin, dmax=dmax, df=df, eps=eps, obj_ind=j, obj_score=score,
        )
        if gg_ == None:
            continue
        if gg == None:
            gg = gg_
        else:
            gg.append(gg_)
    return gg


def demo_grasp_simple():
    masks, mask_all = draw_masks()

    gg = masks2grasps(masks=masks, dmin=20, dmax=160)

    gg.show(cv2.cvtColor(255 * mask_all, cv2.COLOR_GRAY2BGR), topn=100)


class KGraspDetector(GraspDetect):

    def __init__(self, instance_detector=None):
        self.instance_detector = instance_detector

    def run( self,rgb, depth=None, crop_roi=None, cam_params=None, target_ind=None, contactmap=None, 
            instances=None, min_mass=0, max_mass=500000,max_ratio=1, 
            nfingers=2, dmin=10,dmax=150, df=50,
            show_grasp=False, **kwargs):

        assert instances is not None or self.instance_detector is not None
        if instances is None:
            instances = self.instance_detector.run(
                crop_image(rgb,crop_roi=crop_roi, keep_size=True), 
                min_mass=min_mass, max_ratio=max_ratio,max_mass=max_mass) 
        if instances is None or len(instances)==0:
            return None

        self.instances = instances
        masks, scores = instances.masks,instances.scores
        if target_ind is not None:
            masks, scores = [masks[target_ind]], scores[target_ind, :]
        

        gg = masks2grasps(masks=masks, contact_map=contactmap,  nfingers=nfingers, obj_scores=scores, 
                          dmin=dmin, dmax=dmax, df=df)

        if gg is None:
            print("No grasp pose detected ...")
            return

        if show_grasp:
            print(
                f'Target: center: {gg.centers[0,...]}, angle: {gg.inplane_thetas[0,...]*180/np.pi}, width: {gg.widths[0,...]}, contacts: {gg.contact_points[0,...]}'
            )
            out = show_masks_on_rgb(rgb=rgb, masks=masks)
            gg.show(out, topn=50)
            

        return gg

class talos_KGraspDetector(KGraspDetector):

   def run( self,rgb, depth=None, crop_roi=None, cam_params=None, target_ind=None, contactmap=None, 
            instances=None, min_mass=0, max_mass=500000,max_ratio=1, 
            nfingers=2, dmin=10,dmax=150, df=50,
            show_grasp=False, **kwargs):
        assert instances is not None or self.instance_detector is not None
        if instances is None:
            instances = self.instance_detector.run(
                crop_image(rgb,crop_roi=crop_roi, keep_size=True), 
                min_mass=min_mass, max_ratio=max_ratio,max_mass=max_mass) 
        if instances is None or len(instances)==0:
            return None
        
        self.instances = instances
        masks, scores = instances.masks,instances.scores
        if target_ind is not None:
            masks, scores = [masks[target_ind]], scores[target_ind, :]
        

        gg = masks2grasps(masks=masks, contact_map=contactmap,  nfingers=nfingers, obj_scores=scores, 
                          dmin=dmin, dmax=dmax, df=df)
        gg.boxes=self.instances.boxes
        gg.masks=masks
        if gg is None:
            print("No grasp pose detected ...")
            return

        if show_grasp:
            print(
                f'Target: center: {gg.centers[0,...]}, angle: {gg.inplane_thetas[0,...]*180/np.pi}, width: {gg.widths[0,...]}, contacts: {gg.contact_points[0,...]}'
            )
            out = show_masks_on_rgb(rgb=rgb, masks=masks)
            gg.show(out, topn=50)
            
        print(gg)
        return gg

#    def angle_detection(self,rgb,depth=None):
#        print("angle_detection")
#        return [{"mode":"angle_detection"},rgb]
def demo_grasp():
    from pyrecognition.instance_detection.fast_sam import InstanceDetector

    NO_MODEL = False

    ins_detector = None if NO_MODEL else InstanceDetector() 
    detector = KGraspDetector(instance_detector=ins_detector)

    detector.demo_single(
        rgb_path="/media/keti/workdir/projects/data/app_rpp/test_images/20241108/20241107_165852_rgb.png",
        # depth_path=depth_path,
        # cam_params_path=cam_params_path,
        target_ind=None,
        no_model=NO_MODEL,
        topn=None,
        crop_roi=(280, 80, 1035,670)
    )
    
    # detector.demo_multi(
    #     rgb_format='/media/keti/workdir/projects/data/app_rpp/test_images/20241108/*rgb*',
    #     # rgb_format='/media/keti/workdir/projects/data/app_rpp/test_images/20241108/20241107_165852_rgb.png',
    #     depth_format='/media/keti/workdir/projects/data/app_rpp/test_images/20241108/*depth*',
    #     crop_roi=(280, 80, 1035,670),
    # )


if __name__ == "__main__":
    # demo_grasp_simple()
    demo_grasp()
