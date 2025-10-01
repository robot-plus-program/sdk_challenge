import cv2
import numpy as np
import copy
from itertools import combinations
import torch
from pyinterfaces.utils import twopoints2theta

class RGBDtensor:
    def __init__(self, rgb=None, depth=None, device="cpu"):
        self.rgb = (
            torch.tensor(rgb.astype("float32")).to(device) if rgb is not None else None
        )
        self.depth = (
            torch.tensor(depth.astype("float32")).to(device)
            if depth is not None
            else None
        )
        self.device = device

    @property
    def height(self):
        return self.rgb.shape[0] if self.rgb is not None else self.depth.shape[0]

    @property
    def width(self):
        return self.rgb.shape[1] if self.rgb is not None else self.depth.shape[1]

    def xyz(self, cam_params):
        fx, fy, cx, cy = cam_params[:4]
        Py, Px = torch.meshgrid(torch.arange(self.height), torch.arange(self.width))
        Px, Py = Px.to(self.device) - cx, Py.to(self.device) - cy
        X, Y = torch.multiply(Px, self.depth) / fx, torch.multiply(Py, self.depth) / fy
        return torch.cat((X[..., None], Y[..., None], self.depth[..., None]), dim=-1)

    def normal_map(self, cam_params=None, r=3, xyz=None):
        if xyz is None:
            xyz = self.xyz(cam_params=cam_params)

        xyz_mrmr = xyz[: -2 * r, : -2 * r, :]
        xyz_rmr = xyz[2 * r :, : -2 * r, :]
        xyz_0r = xyz[r:-r, 2 * r :, :]
        V0, V1 = xyz_mrmr - xyz_0r, xyz_rmr - xyz_0r

        V2 = torch.cross(V0, V1, dim=-1)
        V2 = torch.divide(V2, torch.norm(V2, dim=-1, keepdim=True) + 1e-10)
        return torch.nn.functional.pad(V2, pad=(0, 0, r, r, r, r), mode="constant")


class PointGroup:
    def __init__(self, arr=None, cam_params=(927.16, 927.36, 927.36, 349.62)):
        """
        :param arr:  Nx3 float32 [object_id, 3x3D points, score]
        :param cam_params: intrinsic params
        """
        self.arr = arr
        self.cam_params = cam_params

    @property
    def xs(self):
        return self.arr[:, [0]]

    @property
    def ys(self):
        return self.arr[:, [1]]

    @property
    def zs(self):
        return self.arr[:, [2]]

    @property
    def xs_2d(self):
        fx, fy, cx, cy = self.cam_params[:4]
        return (np.divide(self.xs, self.zs + 1e-10) * fx + cx).astype("int")

    @property
    def ys_2d(self):
        fx, fy, cx, cy = self.cam_params[:4]
        return (np.divide(self.ys, self.zs + 1e-10) * fy + cy).astype("int")

    def from_2d(self, V):
        fx, fy, cx, cy = self.cam_params[:4]
        Px, Py, Z = V[:, [0]], V[:, [1]], V[:, [2]]
        X, Y = np.multiply(Px - cx, Z) / fx, np.multiply(Py - cy, Z) / fy
        self.arr = np.concatenate((X, Y, Z), axis=-1)

    def disp(self, rgb):
        out = rgb.copy()
        for x, y in zip(self.xs_2d.flatten(), self.ys_2d.flatten()):
            out = cv2.drawMarker(
                out, (x, y), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 5, 2
            )
        return out


class VectorGroup:
    def __init__(self, arr=None, cam_params=(927.16, 927.36, 927.36, 349.62)):
        """
        :param arr:  Nx3 float32 [object_id, 3x3D points, score]
        :param cam_params: intrinsic params
        """
        self.arr = arr
        self.cam_params = cam_params

    @property
    def xs(self):
        return self.arr[:, [0]]

    @property
    def ys(self):
        return self.arr[:, [1]]

    @property
    def zs(self):
        return self.contact_points[:, [2]]

    @property
    def norm(self):
        return np.divide(self.arr, np.linalg.norm(self.arr, axis=-1, keepdims=True))

    def inner(self, V):
        return np.sum(np.multiply(self.arr, V), axis=-1, keepdims=True)

    def disp(self, rgb, org, l):
        """
        Parameters
        ----------
        rgb:  HxWx3 image
        org: Origin of vector (px, py, z)
        l: length of vector

        Returns
        -------

        """
        P0 = PointGroup(arr=np.array(org).reshape(-1, 3), cam_params=self.cam_params)
        P1 = PointGroup(arr=l * self.norm + P0.arr, cam_params=self.cam_params)

        out = P0.disp(rgb)
        for x0, y0, x1, y1 in zip(
            P0.xs_2d.flatten(),
            P0.ys_2d.flatten(),
            P1.xs_2d.flatten(),
            P1.ys_2d.flatten(),
        ):
            out = cv2.line(out, (x0, y0), (x1, y1), (0, 255, 0), 2)
        return out


class GraspGroup:
    def __init__(
        self,
        contact_points,
        surface_normals=None,
        obj_ids=None,
        scores=None,
        cam_params=(927.16, 927.36, 927.36, 349.62),
        is2d=False,
    ):
        """
        :param arr:  Nx14 float32 [object_id, 3x3D points, 3D normals, score]
        :param dmin: min distance of 2 contacting points
        """
        self.is2d = is2d
        self.scores = np.ones((len(self), 1), "int") if scores is None else scores
        self.contact_points = contact_points
        self.surface_normals = (
            (0, 0, 1) * np.ones((len(self), 1), "float32")
            if surface_normals is None
            else surface_normals
        )
        self.object_ids = (
            np.zeros((len(self), 1), "int") if obj_ids is None else obj_ids
        )
        self.sort()

        self.cam_params = cam_params
        self.boxes=None
        self.masks=None

    def __len__(self):
        return self.contact_points.shape[0]

    def sort(self):
        inds = np.argsort(self.scores.flatten()).tolist()[::-1]
        self.scores = self.scores[inds, :]
        self.contact_points = self.contact_points[inds, :]
        self.object_ids = self.object_ids[inds, :]
        self.surface_normals = self.surface_normals[inds, :]

    def append(self, gg):
        self.contact_points = np.concatenate(
            (self.contact_points, gg.contact_points), axis=0
        )
        self.surface_normals = np.concatenate(
            (self.surface_normals, gg.surface_normals), axis=0
        )
        self.object_ids = np.concatenate((self.object_ids, gg.object_ids), axis=0)
        self.scores = np.concatenate((self.scores, gg.scores), axis=0)
        self.sort()

    def set_scores(self, scores):
        self.scores = scores
        self.sort()

    @property
    def contact_points_2d(self):
        if self.is2d:
            return self.contact_points
        V = self.contact_points
        V1 = [
            self.convert3dto2d(
                z=V[:, [i * 3 + 2]], x=V[:, [i * 3]], y=V[:, [i * 3 + 1]]
            )
            for i in range(self.nfingers)
        ]
        return np.concatenate(V1, axis=-1)

    @property
    def centers(self):
        n = 2 if self.is2d else 3
        return np.concatenate(
            [
                np.mean(self.contact_points[:, i::n], axis=-1, keepdims=True)
                for i in range(n)
            ],
            axis=-1,
        )

    @property
    def centers_2d(self):
        if self.is2d:
            return self.centers.astype("int")
        V = self.centers
        return self.convert3dto2d(z=V[:, [2]], x=V[:, [0]], y=V[:, [1]])

    @property
    def nfingers(self):
        n = 2 if self.is2d else 3
        return self.contact_points.shape[-1] // n

    @property
    def force_list(self):
        if self.nfingers == 1:
            return [
                self.surface_normals,
            ]
        return [
            VectorGroup(self.finger(i) - self.centers, self.cam_params).norm
            for i in range(self.nfingers)
        ]

    @property
    def forces(self):
        return np.concatenate(self.force_list, axis=-1)

    @property
    def inplane_thetas(self):
        if self.nfingers == 1:
            return np.zeros((len(self), 1), "float32")
        P0 = self.finger(0) if self.nfingers == 2 else self.centers
        P1 = self.finger(1) if self.nfingers == 2 else self.finger(0)
        return twopoints2theta(P0, P1)

    @property
    def inplane_thetas_2d(self):
        if self.nfingers == 1:
            return np.zeros((len(self), 1), "float32")
        P0 = self.finger_2d(0) if self.nfingers == 2 else self.centers_2d
        P1 = self.finger_2d(1) if self.nfingers == 2 else self.finger_2d(0)
        return twopoints2theta(P0, P1)

    @property
    def widths(self):
        if self.nfingers == 1:
            return np.zeros((len(self), 1), "float32")
        P0 = self.finger(0) if self.nfingers == 2 else self.centers
        P1 = self.finger(1) if self.nfingers == 2 else self.finger(0)
        return np.linalg.norm((P0[0] - P1[0], P0[1] - P1[1], P0[2] - P1[2]), axis=-1, keepdims=True)

    @property
    def widths_2d(self):
        if self.is2d:
            return self.widths
        if self.nfingers == 1:
            return np.zeros((len(self), 1), "float32")
        P0 = self.finger_2d(0) if self.nfingers == 2 else self.centers_2d
        P1 = self.finger_2d(1) if self.nfingers == 2 else self.finger_2d(0)
        return np.linalg.norm((P0[0] - P1[0], P0[1] - P1[1]))

    @property
    def approach_vectors(self):
        if self.nfingers <= 2:
            return -self.surface_normals
        P0, P1, P2 = self.finger(0), self.finger(1), self.finger(3)
        P10, P20 = P1 - P0, P2 - P0
        V = np.cross(P20, P10, dim=-1)
        V = np.divide(V, np.linalg.norm(V, dim=-1, keepdim=True) + 1e-10)
        return V

    @property
    def each_obj_inds(self):
        uniq_ids = sorted(np.unique(self.object_ids))
        return [np.where(self.object_ids==i)[0].tolist() for i in uniq_ids]
    
    @property
    def each_obj_target_ind(self):
        return [inds[int(np.argmax(self.scores[inds,...].flatten()))] for inds in self.each_obj_inds ]

            
    def finger(self, index=0):
        n = 2 if self.is2d else 3
        return self.contact_points[:, n * index : n * index + n]

    def finger_2d(self, index=0):
        # finger = self.finger(index=index)
        # return self.convert3dto2d(z=finger[:, [2]], x=finger[:, [0]], y=finger[:, [1]])
        return self.contact_points_2d[:, 2 * index : 2 * index + 2]

    def convert3dto2d(self, z, x=None, y=None, cam_params=None):
        assert x is not None or y is not None
        fx, fy, cx, cy = self.cam_params[:4] if cam_params is None else cam_params[:4]
        ret, eps = [], 1e-10
        if x is not None:
            ret.append((np.divide(x, z + eps) * fx + cx).astype("int"))
        if y is not None:
            ret.append((np.divide(y, z + eps) * fy + cy).astype("int"))
        return np.concatenate(ret, axis=-1) if len(ret) > 1 else ret[0]
    

    def topn(self, n=None):
        if n is None:
            return self
        n = min(n, len(self))
        return GraspGroup(
            contact_points=self.contact_points[:n, :],
            surface_normals=self.surface_normals[:n, :],
            scores=self.scores[:n, :],
            obj_ids=self.object_ids[:n, :],
            cam_params=self.cam_params,
            is2d=self.is2d,
        )
    
    def disp_each_objs(self, rgb):
        out = rgb.copy()
        finger_pair_inds = list(combinations(range(self.nfingers), 2)) if self.nfingers>1 else None
        finger_2d_list = [self.finger_2d(i) for i in range(self.nfingers)]
        for k in self.each_obj_target_ind:
            x, y = self.centers_2d[k, :]
            if self.nfingers==1:
                out = cv2.drawMarker(out, (x, y), (0,255,0), cv2.MARKER_CROSS, 10, 1)
            else:
                for i, j in finger_pair_inds:
                    x0, y0 = finger_2d_list[i][k, :]
                    x1, y1 = finger_2d_list[j][k, :]
                    out = cv2.line(out, (x0, y0), (x1, y1), (0,255,0), 1)
        return out
            
        

    def disp(self, rgb, topn=None, **kwargs):   
        out = rgb.copy()     
        n = len(self) if topn is None else topn
        gg = self.topn(n)
        
        finger_pair_inds = list(combinations(range(gg.nfingers), 2)) if gg.nfingers>1 else None
        finger_2d_list = [gg.finger_2d(i) for i in range(gg.nfingers)]
        for k in np.roll(range(len(gg)), -1):
            x, y = gg.centers_2d[k, :]
            # x1, y1 = gg.finger_2d(0)[k,:]
            color = (0,255, 0) if k == 0 else (0, 0, 255)
            thick = 3 if k == 0 else 1
            if self.nfingers==1:
                out = cv2.drawMarker(out, (x, y), color, cv2.MARKER_CROSS, 10, thick)
            else:
                for i, j in finger_pair_inds:
                    x0, y0 = finger_2d_list[i][k, :]
                    x1, y1 = finger_2d_list[j][k, :]
                    out = cv2.line(out, (x0, y0), (x1, y1), color, thick)
            # x0, y0 = finger_2d_list[0][k, :]
            # out = cv2.drawMarker(out, (x0, y0), color, cv2.MARKER_TILTED_CROSS, 10, thick)
            if k==0:
                out = cv2.putText(out, f"{100*gg.scores[(0,0)]:.1f}", (x, y),cv2.FONT_HERSHEY_COMPLEX, 0.5, color)
        out = self.disp_each_objs(rgb=out)

        return out

    def show(self, rgb, topn=None, **kwargs):
        out = self.disp(rgb, topn=topn)
        cv2.imshow("grasp", out[..., ::-1])
        if cv2.waitKey() == 27:
            exit()
        return out

    def show_3d(self, xyz, rgb=None, mask=None, normal_map=None, topn=5, **kwargs):
        from ketisdk.vision.utils.image_processing import (
            get_clouds,
            show_cloud,
            get_lineset_cloud,
        )

        # surface
        clouds = get_clouds(xyz, rgb, mask, scale=1000.0, show_normal_map=False)

        # normal vector
        if normal_map is not None:
            points0 = np.concatenate(
                [self.finger(i).astype("float32") / 1000 for i in range(self.nfingers)]
            )
            # normals = np.concatenate([normal_map[loc].reshape(-1,3)[:n, :] for loc in self.loc_2d_list], axis=0)
            normals = np.concatenate(
                [self.surface_normals[:, i : i + 3] for i in range(self.nfingers)]
            )
            points2 = points0 + normals * 0.015
            clouds.append(get_lineset_cloud(points0, points2, color=[0, 0, 1]))
            # print(f'{np.sum(np.multiply(forces, normals), axis=-1, keepdims=True)}')

        gg = self.topn(topn)
        points0 = np.concatenate(
            [gg.finger(i).astype("float32") / 1000 for i in range(gg.nfingers)]
        )
        forces = np.concatenate([el.astype("float32") for el in gg.force_list], axis=0)
        points1 = points0 + forces * 0.020
        clouds.append(get_lineset_cloud(points0, points1, color=[1, 0, 0]))

        show_cloud(clouds)

    def todict(self, topn=10):
        gg = self.topn(n=topn)
        return {'centers_2d': gg.centers_2d, 
                'widths_2d': gg.widths_2d,
                'inplane_thetas_2d': gg.inplane_thetas_2d,
                'scores': gg.scores}