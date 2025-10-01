import numpy as np, cv2, os
from pyinterfaces.utils import write_json, visualize_insdetect, show_mask_on_rgb, crop_image, xyz2Ixy, Ixy2xyz, calc_iou_1xn, calc_normalvector
from pathlib import Path
root_dir = Path(__file__).parent.parent
log_dir = os.path.join(root_dir, 'logs/instances')
os.makedirs(log_dir, exist_ok=True)

def masks2boxes(masks):
    masks_loc = [np.where(mask>0) for mask in masks]
    return np.array([[np.amin(X), np.amin(Y), np.amax(X), np.amax(Y)] for Y,X in masks_loc])
    

class CustomInstances():
    def __init__(self, imsize=None, masks=None, boxes=None, labels=None, scores=None, issorted=False):
        assert imsize is not None or masks is not None
        assert masks is not None or boxes is not None
        self.masks = masks
        if self.masks is not None:
            self.masks = [(m>0).astype('uint8') for m in self.masks]
        self.boxes = masks2boxes(masks) if boxes is None else boxes
        self.imsize = masks[0].shape[:2][::-1] if imsize is None else imsize
        self.labels = [f'obj{el}' for el in range(len(self))] if labels is None else labels
        self.scores = np.ones((len(self), 1), 'float32') if scores is None else scores.reshape(-1,1)
        if not issorted:
            self.sort()

    def __len__(self):
        return len(self.masks) if self.masks is not None else len(self.boxes) 
    
    @property
    def im_height(self):
        return self.imsize[1]
    
    @property
    def centers(self):
        X, Y = (self.boxes[:,[0]] + self.boxes[:,[2]]) //2, (self.boxes[:,[1]] + self.boxes[:,[3]]) //2
        return np.concatenate((X,Y), axis=-1)

    @property
    def im_width(self):
        return self.imsize[0]
    
    @property
    def box_heights(self):
        return self.boxes[:, [3]] - self.boxes[:, [1]]
    
    @property
    def box_widths(self):
        return self.boxes[:, [2]] - self.boxes[:, [0]]
    
    @property
    def masses(self):
        return [np.sum(m) for m in self.masks]
    
    @property
    def mask_all(self):
        if self.masks is None:
            return None
        out = None
        for m in self.masks:
            out = m if out is None else np.bitwise_or(out, m)
        return out if out is None else out.astype('uint8') 
    
    @property
    def centers(self):
        Xc, Yc = (self.boxes[:,[0]] +self.boxes[:,[2]])//2, (self.boxes[:,[1]] +self.boxes[:,[3]])//2
        return np.concatenate((Xc, Yc), axis=-1)
    
    @property
    def label_clusters(self):
        out = {}
        for index, label in enumerate(self.labels):
            out.setdefault(label, []).append(index)
        for k, inds in out.items():
            out[k] = self.select(inds=inds, issorted=False)
        return out
    
    def mask_all_blur(self, kernel_size=(21,21)):
        out = np.zeros((self.im_height, self.im_width), 'float32')
        kernel = np.ones(kernel_size, 'uint8')
        # kernel_normal = kernel.astype('float32')/np.sum(kernel)
        for m in self.masks:
            m = cv2.erode(m, kernel)
            out += cv2.blur(m.astype('float32'), ksize=kernel_size)
        return np.clip(out, 0, 1)
    
    def shrink(self, kernel_size=None):
        if kernel_size is None:
            return self
        kernel = np.ones(kernel_size, 'uint8')
        masks = [cv2.erode(m, kernel=kernel) for m in self.masks]
        inds = [i for i,m in enumerate(masks) if np.sum(m)>10]
        masks = [masks[i] for i in  inds]
        scores = np.array([self.scores[i] for i in  inds]).reshape(-1,1)
        labels = [self.labels[i] for i in  inds]
        locs = [np.where(m) for m in masks]
        boxes = np.array([(np.amin(X), np.amin(Y), np.amax(X), np.amax(Y)) for Y,X in locs])
        return CustomInstances(masks=masks, boxes=boxes, imsize=self.imsize, scores=scores, labels=labels)
    
    def expand(self, kernel_size=None):
        if kernel_size is None:
            return self
        kernel = np.ones(kernel_size, 'uint8')
        masks = [cv2.dilate(m, kernel=kernel) for m in self.masks]
        locs = [np.where(m) for m in masks]
        boxes = np.array([(np.amin(X), np.amin(Y), np.amax(X), np.amax(Y)) for Y,X in locs])
        return CustomInstances(masks=masks, boxes=boxes, imsize=self.imsize, scores=self.scores, labels=self.labels)
        
    
    def select(self, inds, issorted=False):
        scores = self.scores[inds, :]
        masks = [self.masks[i] for i in inds] if self.masks is not None else None
        labels = [self.labels[i] for i in inds]
        boxes = self.boxes[inds, :]
        return CustomInstances(masks=masks, boxes=boxes, labels=labels, scores=scores, 
                               issorted=issorted, imsize=self.imsize)

    def sort(self):
        inds = np.argsort(self.scores.flatten()).tolist()[::-1]
        self = self.select(inds=inds, issorted=True)

    def clip_score(self, vmin=0.5, vmax=1):
        inds = np.where((vmin<=self.scores.flatten()) & (self.scores.flatten()<=vmax))[0].tolist()
        return self.select(inds=inds, issorted=True)
    
    def clip_mass(self, min_mass=0, max_mass=None):
        if self.masks is None:
            return self
        inds = [i for i,m in enumerate(self.masses) if min_mass<=m and (True if max_mass is None else m<=max_mass)]
        return self.select(inds=inds, issorted=True)
    
    def clip_ratio(self, min_ratio=0, max_ratio=1):
        H, W = self.box_heights.flatten(), self.box_widths.flatten()
        hmin, hmax =  min_ratio*self.im_height, max_ratio*self.im_height
        wmin, wmax = min_ratio*self.im_width, max_ratio*self.im_width
        inds = np.where((hmin<=H) & (H<=hmax) & (wmin<=W) & (W<=wmax))[0].tolist()
        return self.select(inds=inds,issorted=True)
    
    def todict(self):
        if len(self) ==0:
            return {}
        if len(self.label_clusters)<=1:
            return {self.labels[0]:{'masks': self.masks, 'boxes': self.boxes, 'scores': self.scores, 'mask_all': self.mask_all}}
        out = {}
        for label, ins in self.label_clusters.items():
            out.update(ins.todict())
        return out
        
       
    def visualize(self, rgb, colors=None):
        return visualize_insdetect(rgb=rgb, pred=self.todict(), colors=colors)   
    
    def visualize_maskall(self, rgb):
        return show_mask_on_rgb(rgb=rgb, mask=self.mask_all)
    
    def split_mass(self,mass_threshs=[5000,]):
        splits = [[] for _ in range(len(mass_threshs) + 1)]
        for i,mass in enumerate(self.masses):
            isobtained = False
            for j, th in enumerate(sorted(mass_threshs)):
                if mass<th:
                    splits[j].append(i)
                    isobtained = True
                    break
            if not isobtained:
                splits[-1].append(i)
        return [self.select(inds=inds) for inds in splits]
    
    def from_maskall(self, mask_all):
        assert self.boxes is not None
        self.masks = [crop_image(mask_all, crop_roi=box, keep_size=True) for box in self.boxes]

    def import_masks(self, masks):
        '''
        1 box to n masks
        '''
        assert self.boxes is not None
        if len(self)==0:
            return
        zeros = np.zeros((self.im_height, self.im_width), 'uint8')
        self.masks = [zeros.copy() for _ in range(len(self))]
        
        # for mask in masks:
            # Y,X = np.where(mask)
            # box = np.array([[np.amin(X), np.amin(Y), np.amax(X), np.amax(Y)]])
            # ious = calc_iou_1xn(box.flatten(), self.boxes)
            # if len(ious)==0:
            #     continue
            # argmax = int(np.argmax(ious))
            # if ious[argmax]<0.5:
            #     continue
            # self.masks[argmax] = np.bitwise_or(self.masks[argmax], mask)
        masks = [(m>0).astype('uint8') for m in masks]
        for mask in masks:
            mass = np.sum(mask)
            masks_inner = np.array([crop_image(mask, crop_roi=bbox, keep_size=True) for bbox in self.boxes])

            mass_inner_ratios = np.array([np.sum(m)/mass for m in masks_inner])
            argmax = int(np.argmax(mass_inner_ratios))
            if mass_inner_ratios[argmax]<0.8:
                continue
            self.masks[argmax] = np.bitwise_or(self.masks[argmax], masks_inner[argmax])
            
    def get_normalvector(self, depth, cam_params, target_ind=0):
        assert len(self)>0
        # assert self.masks is not None

        valid_depth = depth>0
        try:
            mask = cv2.erode(self.masks[target_ind], kernel=np.ones((11,11), 'uint8'))
            Iy, Ix = np.where((mask>0) & valid_depth)
        except:
            Iy, Ix = np.where(valid_depth)
        if len(Ix)==0:
            return None
        
        Z =  depth[(Iy, Ix)].astype('float32')

        X,Y,Z = Ixy2xyz(Ix=Ix.astype('float32'), Iy=Iy.astype('float32'), Z=Z, cam_params=cam_params)
        points = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=-1)

        return calc_normalvector(points)
        
    def log_normalvector(self, depth, cam_params, target_ind=0):
        assert len(self)>0
        # assert self.masks is not None

        mask = cv2.erode(self.masks[target_ind], kernel=np.ones((11,11), 'uint8'))
        Iy, Ix = np.where((mask>0) & (depth>0))
        
        if len(Ix)==0:
            return None
        Z =  depth[(Iy, Ix)].astype('float32')

        X,Y,Z = Ixy2xyz(Ix=Ix.astype('float32'), Iy=Iy.astype('float32'), Z=Z, cam_params=cam_params)
        points = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=-1)
        normal = calc_normalvector(points)
        data = {'points': points.tolist(), 'normal': normal.tolist()}
        write_json(os.path.join(log_dir, f'normalvector{target_ind}.json'), data)

    def get_depthvalue(self, depth, target_ind=0):
        valid_depth = depth > 0
        try:
            mask = cv2.erode(self.masks[target_ind], kernel=np.ones((11,11), 'uint8'))
            locs = np.where((mask>0) & valid_depth)
        except:
            locs = np.where(valid_depth)
        if len(locs[0])==0:
            return None
        
        return np.median(depth[locs])
        
    def visualize_normalvectors(self, rgb, depth, cam_params, target_ind=None):
        out = self.visualize(rgb=rgb)
        target_inds = list(range(len(self))) if target_ind is None else [target_ind,]
        for target_ind in target_inds:
            normal = self.get_normalvector(depth=depth, cam_params=cam_params, target_ind=target_ind)
            if normal is None:
                continue
            Ix, Iy = self.centers[target_ind, :]
            z0 = depth[(Iy, Ix)]

            point0 =  np.array(Ixy2xyz(Ix, Iy, z0, cam_params))
            point1 = point0 + 50*normal

            Ix, Iy = xyz2Ixy(point0[0], point0[1], point0[2], cam_params=cam_params)
            Ix1, Iy1 = xyz2Ixy(point1[0], point1[1], point1[2], cam_params=cam_params)
            out = cv2.drawMarker(out, (Ix, Iy), (255,0,0), cv2.MARKER_TILTED_CROSS, 10, 2)
            out =  cv2.line(out, (Ix, Iy), (Ix1, Iy1), (255,0,0), 2)
        return out
    
    

    def visualize3d(self,rgb, depth, cam_params, normal_vector=None, target_ind=0):
        import open3d as o3d

        fx, fy, cx, cy = cam_params
        h, w = depth.shape
        
        # Generate pixel grid
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        u, v = u.flatten(), v.flatten()
        
        # Flatten depth and mask
        mask = self.masks[target_ind]
        mask = cv2.dilate(mask, np.ones((21,21), 'uint8'))
        depth_flat = depth.flatten()
        mask_flat = mask.flatten()
        
        # Filter valid depth and mask
        valid = (depth_flat > 0) & (mask_flat > 0)
        u, v, depth_flat = u[valid], v[valid], depth_flat[valid]
        
        # Compute 3D points
        x = (u - cx) * depth_flat / fx
        y = (v - cy) * depth_flat / fy
        z = depth_flat
        points = np.stack((x, y, z), axis=-1)
        
        # Get corresponding colors
        colors = rgb[v, u] / 255.0
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Create a line representing the normal vector
        normal_vector = self.get_normalvector(depth=depth, cam_params=cam_params, target_ind=target_ind) if normal_vector is None else normal_vector
        normal_start = np.mean(points, axis=0)  # Center of the object
        normal_end = normal_start + 50*normal_vector  # End of normal vector
        
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector([normal_start, normal_end])
        line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
        line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0]])  # Red line
        
        # Visualize
        o3d.visualization.draw_geometries([pcd, line_set])
        
    

        

        
        
            
            
        

                
            

        
        
        
        
            
    

    
    
        
        