import os.path

import numpy as np
from ketisdk.utils.proc_utils import CFG, ProcUtils
from scipy.ndimage import rotate
import cv2
from ketisdk.gui.default_config import default_args
from PIL import Image
import random
from scipy.optimize import curve_fit


def get_default_image_processing_config():
    BASE_CONFIGS = default_args()

    BASE_CONFIGS.save = CFG()
    BASE_CONFIGS.save.aug_dirs = ['data/aug0']
    BASE_CONFIGS.save.extend = '.png'
    BASE_CONFIGS.save.compress_level = 0

    BASE_CONFIGS.rotate = CFG()
    BASE_CONFIGS.rotate.angles = [0, 30, 60, 120, 150]
    BASE_CONFIGS.rotate.reshape = True
    BASE_CONFIGS.rotate.order = 3
    BASE_CONFIGS.rotate.aug_angle_step = 4
    BASE_CONFIGS.rotate.random = False

    BASE_CONFIGS.color = CFG()
    BASE_CONFIGS.color.brightnesss = [0.3, ]
    BASE_CONFIGS.color.contrasts = [0.3, ]
    BASE_CONFIGS.color.saturations = [0.3, ]
    BASE_CONFIGS.color.hues = [0.2, ]
    BASE_CONFIGS.color.include_org = True

    BASE_CONFIGS.flip = CFG()
    BASE_CONFIGS.flip.axes = [-1, 0, 1]
    return BASE_CONFIGS


def get_image_processing_key_args():
    return ['rotate.random', 'rotate.aug_angle_step', 'color.include_org', 'color.brightnesss', 'color.contrasts',
            'color.saturations', 'color.hues', 'flip.axes']


class ImageProcessing():
    def rotate(self, im, angle=None, reshape=True, order=3):
        angle = random.randint(0, 360) if angle is None else angle
        return rotate(im, angle=angle, reshape=reshape, order=order)

    def flip(self, im, axis=0):
        out = im.copy()
        if axis in [0, 1]:
            if len(out.shape) > 2:
                return out[::-1, :, :] if axis == 0 else im[:, ::-1, :]
            else:
                return out[::-1, :] if axis == 0 else out[:, ::-1]
        return out

    def change_color(self, im, brightness=0, contrast=0, saturation=0, hue=0):
        from torchvision import transforms
        transformer = transforms.ColorJitter(brightness=brightness, contrast=contrast, saturation=saturation, hue=hue)
        return np.asarray(transformer(Image.fromarray(im)))

    def replace_background(self, im, fg_mask, bg):
        out = im.copy()
        h, w = im.shape[:2]
        bh, bw = bg.shape[:2]
        if (bh, bw) != (h, w):
            bg = cv2.resize(bg, dsize=(w, h), interpolation=cv2.INTER_CUBIC)
        locs = fg_mask == 0
        out[locs] = bg[locs]
        return out

    def augmentation(self, im, color_params=None, rotate_params=None, flip_params=None):
        out = im.copy()
        if color_params is not None:
            out = self.change_color(out, brightness=color_params['brightness'],
                                    contrast=color_params['contrast'],
                                    saturation=color_params['saturation'],
                                    hue=color_params['hue'])
        if rotate_params is not None:
            out = self.rotate(out, angle=rotate_params['angle'],
                              reshape=rotate_params['reshape'],
                              order=rotate_params['order'])
        if flip_params is not None:
            out = self.flip(out, flip_params['axis'])

        return out

    def make_augmentation_params(self, brightnesss=[0, 2], contrasts=[0, 2], saturations=[0, 2], hues=[0, 0.2],
                                 angles=range(0, 360, 4), flip_axis=[-1, 0, 1]):

        Brightness, Contrast, Saturation, Hue, Angle, Flip = np.meshgrid(
            brightnesss, contrasts, saturations, hues, angles, flip_axis
        )
        print(f'1-->{len(Brightness)} augmentation params initialized ...')
        return Brightness.flatten().tolist(), Contrast.flatten().tolist(), Saturation.flatten().tolist(), \
            Hue.flatten().tolist(), Angle.flatten().tolist(), Flip.flatten().tolist()


def get_imageprocessing_gui_obj():
    from kpick.base.base import DetGuiObj

    class ImageProcessingGui(ImageProcessing, DetGuiObj):
        def __init__(self, args=None, cfg_path=None):
            super().__init__(args=args, cfg_path=cfg_path, default_args=get_default_image_processing_config())

        def rotate_and_save(self, rgbd, filename='unnamed.png', angles=[45, ], disp_mode='rgb'):
            for angle in angles:
                rgbd_rot = rgbd.rotate(angle=angle) if angle != 0 else rgbd
                name, ext = os.path.splitext(filename)
                filename_ = f'{name}_rot{angle}{ext}'
                self.save(rgbd_rot, filename=filename_)
            return {'im': rgbd_rot.disp(mode=disp_mode)}

        def init_acc(self):
            angles = range(0, 360, self.args.rotate.aug_angle_step) if not self.args.rotate.random else [None, ]
            self.aug_params = self.make_augmentation_params(self.args.color.brightnesss,
                                                            self.args.color.contrasts,
                                                            self.args.color.saturations,
                                                            self.args.color.hues,
                                                            angles,
                                                            self.args.flip.axes
                                                            )
            print(f'1-->{len(self.aug_params[0])} augmentation params initialized ...')

            self.save_dirs = [os.path.join(aug_dir, ProcUtils().get_current_time_str(format='%m%d'))
                              for aug_dir in self.args.save.aug_dirs]
            for save_dir in self.save_dirs:
                os.makedirs(save_dir, exist_ok=True)
            self.num_aug_dir = len(self.save_dirs)

        def finalize_acc(self):
            self.__delattr__('aug_params')
            print('Augmentation params removed ...')

        def augmentaion_and_save(self, rgbd, filename='unnamed.png', sub_thread=None):
            count = 0
            num_aug = len(self.aug_params[0])
            for brightness, contrast, saturation, hue, angle, ax in zip(*self.aug_params):
                color_params_ = {'brightness': brightness,
                                 'contrast': contrast,
                                 'saturation': saturation,
                                 'hue': hue}
                rotate_params = {'angle': angle, 'reshape': self.args.rotate.reshape, 'order': self.args.rotate.order}
                flip_params = {'axis': ax}

                color_param_list = [None, color_params_] if self.args.color.include_org else [color_params_, ]

        def save(self, rgbd, filename='unnamened.png', dir_name='save'):

            save_dir = os.path.join('data', 'gui_process', dir_name)
            save_rgb_dir = os.path.join(save_dir, 'rgb')
            save_depth_dir = os.path.join(save_dir, 'depth')
            save_depth_u8_dir = os.path.join(save_dir, 'depth_u8')
            save_depth_norm_dir = os.path.join(save_dir, 'depth_norm')
            save_depth_jet_dir = os.path.join(save_dir, 'depth_jet')

            os.makedirs(save_rgb_dir, exist_ok=True)
            os.makedirs(save_depth_dir, exist_ok=True)
            os.makedirs(save_depth_u8_dir, exist_ok=True)
            os.makedirs(save_depth_norm_dir, exist_ok=True)
            os.makedirs(save_depth_jet_dir, exist_ok=True)

            if rgbd.hasRgb:
                cv2.imwrite(os.path.join(save_dir, 'rgb', filename), rgbd.bgr())
            if rgbd.hasDepth:
                cv2.imwrite(os.path.join(save_dir, 'depth', filename), rgbd.depth)
                cv2.imwrite(os.path.join(save_dir, 'depth_u8', filename), rgbd.disp(mode='depth'))
                cv2.imwrite(os.path.join(save_dir, 'depth_norm', filename), rgbd.disp(mode='depth_norm'))
                cv2.imwrite(os.path.join(save_dir, 'depth_jet', filename), rgbd.disp(mode='depth_jet'))

        def gui_process_single(self, rgbd, method_ind=0, filename='unnamed', disp_mode='rgb', **kwargs):
            if method_ind == 0:
                ret = self.augmentaion_and_save(rgbd, filename=filename, sub_thread=kwargs['sub_thread'])
            return ret

    return ImageProcessingGui


def demo_image_processing_gui(cfg_path='image_processing.cfg'):
    from ketisdk.gui.gui import GUI, GuiModule
    # phoxi_modules = [get_phoxi_module()] if run_phoxi else []
    detect_module = GuiModule(
        ImageProcessingGui, num_method=1, key_args=get_image_processing_key_args(),
        cfg_path=cfg_path, default_cfg_path='configs/default.cfg', )
    modules = [detect_module, ]

    GUI('Image Processing Demo', modules=modules)


def estimate_params_mix_two_gaussians(gray_im):
    # Define a function for a Gaussian distribution
    def gaussian(x, mean, std):
        return 1 / (std * np.sqrt(2 * np.pi)) * np.exp(-(x - mean) ** 2 / (2 * std ** 2))

    # Define a function for the combined Gaussian distribution
    def combined_gaussian(x, *params):
        mean1, std1, weight1, mean2, std2, weight2 = params
        return weight1 * gaussian(x, mean1, std1) + weight2 * gaussian(x, mean2, std2)

    # Fit the combined Gaussian distribution to the data
    gray_values = gray_im.flatten()
    hist = np.histogram(gray_values)
    popt, _ = curve_fit(combined_gaussian, hist[1], hist[0])

    # Extract the means and variances
    mean1, std1, weight1, mean2, std2, weight2 = popt
    # variance1 = std1 ** 2
    # variance2 = std2 ** 2
    return mean1, std1, mean2, std2


def fit_surface_params(points):
    """
    Fits a plane to a list of 3D points and returns the plane parameters.

    Args:
      points: A numpy array of shape (n, 3), where each row represents a 3D point (x, y, z).

    Returns:
      A numpy array of shape (4,) containing the plane parameters (A, B, C, D) in the equation Ax + By + Cz + D = 0.
    """

    # Check if there are at least 3 points
    if len(points) < 3:
        raise ValueError("At least 3 points are required to fit a plane.")

    # Mean center the points
    mean_center = np.mean(points, axis=0)
    points_centered = points - mean_center

    # Compute covariance matrix
    cov_mat = np.cov(points_centered.T)

    # Compute the eigenvector corresponding to the smallest eigenvalue
    eigenvalues, eigenvectors = np.linalg.eig(cov_mat)
    normal = eigenvectors[:, np.argmin(eigenvalues)]

    # Compute the plane equation parameters
    A, B, C = normal
    D = -np.dot(normal, mean_center)

    return A, B, C, D


def calc_depth_from_surface_params(x, y, params):
    A, B, C, D = params
    C += 1e-10
    return int(-A / C * x - B / C * y - D / C)


def estimate_depth_surface_from_points(points, im_shape):
    A, B, C, D = fit_surface_params(points)


def concat_images_grid(rgbs, ncols=4, dsize=(60, 40), border=5):
    rgbs = [cv2.resize(el, dsize=dsize, interpolation=cv2.INTER_CUBIC) for el in rgbs]
    n = len(rgbs)
    count = 0
    rgbs_grid = []
    while count<n:
        row = []
        for i in range(ncols):
            row.append(rgbs[count])
            count += 1
            if count>=n:
                break
        rgbs_grid.append(row)
    nrows = len(rgbs_grid)
    
    w,h = dsize
    wmax = w*ncols + border*(ncols-1)
    hmax = h*nrows +border*(nrows-1)
    
    out = np.zeros((hmax, wmax, 3), 'uint8')
    for r in range(nrows):
        for c in range(len(rgbs_grid[r])):
            i,j = r*(h+border), c*(w+border)
            out[i:i+h, j:j+w, :] = rgbs_grid[r][c]
    return out

    
def show_mask_on_rgb(rgb, mask):
    '''
    :param rgb: mxnx3 uint8
    :param mask: mxn float32 range(0,1)
    :return: mxnx3 uint8
    '''
    mask = mask if isinstance(mask, np.ndarray) else mask.detach().cpu().numpy()
    rgb = rgb if isinstance(rgb, np.ndarray) else rgb.detach().cpu().numpy()
    mask = np.repeat(mask[..., np.newaxis], 3, axis=-1)
    heatmap = 255-cv2.applyColorMap((255*mask).astype('uint8'), cv2.COLORMAP_JET)
    return (np.multiply(rgb, 1-mask) + np.multiply(heatmap, mask)).astype('uint8')

def show_masks_on_rgb(rgb, masks, show_instance=True):
    if masks is None:
        return rgb
    if len(masks)==0:
        return rgb
    rgb = rgb if isinstance(rgb, np.ndarray) else rgb.detach().cpu().numpy()
    masks = [m if isinstance(m, np.ndarray) else m.detach().cpu().numpy() for m in masks]
    n = len(masks)
    colors = cv2.applyColorMap((np.arange(len(masks)) / n * 255).astype('uint8'), cv2.COLORMAP_JET)
    out = rgb.copy()
    for m, color in zip(masks, colors):
        loc = np.where(m>0)
        out[loc] = 0.6*out[loc] + tuple((0.4*color).tolist())
    return out


def loc2Dto3D(Px, Py, Z, fx, fy, cx, cy):
    return np.multiply(Px - cx, Z) / fx, np.multiply(Py - cy, Z) / fy

def loc3Dto2D(X,Y,Z, fx, fy, cx, cy):
    Z += 1e-5
    return (np.divide(X, Z) * fx + cx).astype('int'), (np.divide(X, Z) * fy + cy).astype('int')

def loc3DtoNormal(X,Y,Z, mask=None):
    # get neighbor point locations
    h,w = X.shape[:2]
    mask_ = np.ones((h-1, w-1), 'uint8')
    mask_ = np.pad(mask_, pad_width=1, constant_values=0)
    mask = mask_ if mask is None else np.logical_and(mask, mask_)
    Iy,  Ix = np.where(mask)
    I = Ix + Iy * w
    Im1m1, I1m1, I01 = I - w -1, I + w - 1, I + 1

    # get neighbor vectors
    V = np.concatenate((X.reshape((-1,1)), Y.reshape((-1,1)), Z.reshape((-1,1))), axis=1)
    Vm1m1, V1m1, V01 = V[Im1m1.tolist(), :],V[I1m1.tolist(), :], V[I01.tolist(), :]

    # calculate normal vector
    V0 = Vm1m1 - V01
    V1 = V1m1 - V01
    V2 = np.cross(V0, V1, axis=1)
    V2_norm = np.linalg.norm(V2, axis=1, keepdims=True) + 1e-4
    V2 = np.divide(V2, V2_norm)

    return V2

def visualize_depth(depth, depth_min=0, depth_max=1, show_gray=True):
    out = np.clip(depth.astype('float32'), depth_min, depth_max)
    out = (out-depth_min)/(depth_max-depth_min)
    if show_gray:
        return (255*out).astype('uint8')
    return cv2.applyColorMap((255*out).astype('uint8'), cv2.COLORMAP_JET)

def get_normal_vector_map(xyz):
    h, w = xyz.shape[:2]

    # xyz_00 = xyz.copy()[1:-1, 1:-1,:]
    xyz_m1m1 = xyz[:-2, :-2, :]
    xyz_1m1 = xyz[2:, :-2, :]
    xyz_01 = xyz[1:-1, 2:, :]

    V0, V1 = xyz_m1m1 - xyz_01, xyz_1m1 - xyz_01
    V2 = np.cross(V0, V1, axis=-1)
    V2 = np.divide(V2, np.linalg.norm(V2, axis=-1, keepdims=True) + 1e-10)
    V2 = np.pad(V2, pad_width=((1, 1), (1, 1), (0, 0)), mode='edge')
    return V2

def get_lineset_cloud(points0, points1, color=[1,0,0]):
    '''
    Parameters
    ----------
    points0: Nx3
    points1 Nx3

    Returns
    -------
    '''
    import open3d as o3d

    n = len(points0)
    points = np.concatenate((points0, points1), axis=0)
    lines = np.arange(n).reshape(-1,1)
    lines = np.concatenate((lines, lines+n), axis=-1)

    colors = [color]*len(lines)
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def get_clouds(xyz, rgb=None, mask=None, scale=1000., show_normal_map=False):
    import open3d as o3d
    '''
    Parameters
    ----------
    xyz: HxWx3 (int16) 
    rgb: HxWx3 (uint8)
    mask HxW
    Returns
    -------
    '''
    xyz = xyz.astype('float32') / scale
    mask = (0 < xyz[..., 2]) & (xyz[..., 2] < 2) & (mask > 0 if mask is not None else True)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz[mask])
    if rgb is not None:
        cloud.colors = o3d.utility.Vector3dVector(rgb[mask].astype('float32') / 255.)
        
    clouds = [cloud,]

    # normal_map
    if show_normal_map:
        h, w = xyz.shape[:2]
        Px, Py = np.meshgrid(range(0,w, 20), range(0,h,20))
        Py, Px = Py.flatten(), Px.flatten()
        m = np.zeros_like(mask)
        m[(Py, Px)] = mask[(Py, Px)]
        loc = np.where(m>0)
        normal_map = get_normal_vector_map(xyz)
        normals = normal_map[loc].reshape(-1,3)

        
        P0 = xyz[loc].reshape(-1,3)
        P1 = P0 + 0.01*normals
        clouds.append(get_lineset_cloud(P0, P1, [0,0,1]))

    return clouds

def show_3d(xyz, rgb=None, mask=None):
    import open3d as o3d
    o3d.visualization.draw_geometries([get_cloud(xyz, rgb, mask),])

def show_cloud(clouds):
    import open3d as o3d

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # for cl in clouds:
    #     vis.add_geometry(cl)
    # render_option = vis.get_render_option()
    # render_option.line_width = 500.0 # Set line thickness
    # vis.run()
    # vis.destroy_window()
    o3d.visualization.draw_geometries(clouds)


if __name__ == '__main__':
    import open3d as o3d

    points0 = np.array([[0,0,0], [0,0,0], [0,0,0]])
    points1 = np.array([[1,0,0], [0,1,0], [0,0,1]])
    cloud = get_lineset_cloud(points0=points0, points1=points1)

    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "unlitLine"
    mat.line_width = 10  # note that this is scaled with respect to pixels,
    # so will give different results depending on the
    # scaling values of your system
    o3d.visualization.draw({
        "name": "lines",
        "geometry": cloud,
        "material": mat
    })


    # show_cloud([cloud,])

