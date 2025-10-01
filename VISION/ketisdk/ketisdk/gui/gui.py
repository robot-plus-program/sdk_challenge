import datetime
import multiprocessing
import os, sys
import shutil

sys.path.append(os.getcwd())

import tkinter as tk
from tkinter import filedialog as fd
from tkinter import ttk
from tkinter import messagebox as msgbox
from PIL import Image, ImageTk

import cv2
# from ..import_basic_utils import *
from ketisdk.utils.proc_utils import ProcUtils, WorkSpace, Timer, CFG, ArrayUtils, PolyUtils
from ketisdk.utils.gui_utils import GuiUtils
from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
import numpy as np
import time

from threading import Thread
from glob import glob
from functools import partial
from ketisdk.utils.proc_utils import CFG
from ketisdk.gui.default_config import default_args
import json
from ketisdk.sensor.sensor import CameraInfo

commands = ['switch_imshow', 'show_roi', 'crop_roi', 'crop_all', 'crop_all_rename', 'measure', 'show_3D']


class GUI():

    def __init__(self, title='unnamed', gui_size=(1200, 1200), im_disp_scale=(1000, 600),
                 default_cfg_path=None, modules=[], test_dirs=[], run_defaults=None, processes=None,
                 data_root=None, rgb_formats=None, depth_formats=None, add_tcp_client=False, sensor_nbuf=1):
        self.input_entries = []
        self.test_dirs = test_dirs
        self.gui_size = gui_size
        self.modules = modules
        self.processes = processes
        self.im_disp_scale = im_disp_scale
        self.default_cfg_path = default_cfg_path
        if self.default_cfg_path is None: self.default_cfg_path = 'configs/default.cfg'
        if not ProcUtils().is_exist(self.default_cfg_path):
            self.args = default_args()
        else:
            self.args = CFG(cfg_path=self.default_cfg_path, separate=True)
            self.args.merge_with(default_args())
        if rgb_formats is not None:
            self.args.path.im_suffixes = rgb_formats
        if depth_formats is not None:
            self.args.path.depth_suffixes = depth_formats
        if data_root is not None: self.args.path.root_dir = data_root
        self.add_tcp_client = add_tcp_client
        self.sensor_nbuf = sensor_nbuf

        # self.args = self.args_.flatten()
        self.key_commands = ProcUtils().get_command_keys(commands)
        self.init_params()

        self.window = tk.Tk()
        self.window.geometry('{}x{}'.format(gui_size[0], gui_size[1]))
        self.window.title(title)

        self.source_selected = tk.StringVar()
        self.show_steps_var = tk.BooleanVar()
        self.flag_run_all_var = tk.BooleanVar()
        self.flag_run_acc_var = tk.BooleanVar()
        self.flag_hold_var = tk.BooleanVar()
        self.flag_loop_var = tk.BooleanVar()
        self.flag_muti_proc = tk.BooleanVar()
        # self.str_num_run = tk.StringVar()
        self.str_range = tk.StringVar()
        # self.str_range_max = tk.StringVar()
        self.str_goto = tk.StringVar()
        self.str_input_texts = tk.StringVar()

        self.ann_format_var = tk.StringVar()
        self.ann_format_var.set('CT')
        self.flag_show_mask_var = tk.BooleanVar()
        self.flag_show_mask_var.set(True)
        self.flag_show_box_var = tk.BooleanVar()
        self.flag_show_add_im_var = tk.BooleanVar()
        self.flag_show_add_im_var.set(True)
        self.flag_show_add_sym_var = tk.BooleanVar()
        self.flag_show_add_key_var = tk.BooleanVar()
        self.flag_show_add_graph_var = tk.BooleanVar()
        self.flag_lock_key_press_var = tk.BooleanVar()

        #
        self.fr_task_cmd = tk.Frame(master=self.window)
        # self.fr_task_cmd.pack(fill=tk.BOTH, expand=False, side=tk.LEFT)
        self.fr_config = tk.Frame(master=self.window, borderwidth=5, relief=tk.RAISED)
        # self.fr_config.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        self.viewer_parent = ttk.Notebook(master=self.fr_config)

        self.layout_module()

        if run_defaults is not None:
            dir_ind = run_defaults['dir_ind'] if 'dir_ind' in run_defaults else None
            if dir_ind is not None:
                root_dir, im_dir = os.path.split(self.test_dirs[dir_ind])
                self.open_dir_(root_dir, os.path.join(im_dir, '*'))
            run_behaves = run_defaults['run_behaves'] if 'run_behaves' in run_defaults else []
            if 'all' in run_behaves: self.flag_run_all_var.set(True)
            if 'acc' in run_behaves: self.flag_run_acc_var.set(True)
            if 'show_steps' in run_behaves: self.show_steps_var.set(True)
            if 'loop' in run_behaves: self.flag_loop_var.set(True)

            im_range = run_defaults['im_range'] if 'im_range' in run_defaults else ''
            self.str_range.set(im_range)
            # if len(im_range) == 1: range_min, range_max = im_range[0], ''
            # if len(im_range) == 2: range_min, range_max = im_range
            # self.str_range_min.set(str(range_min))
            # self.str_range_max.set(str(range_max))

        self.viewer_parent.pack(fill=tk.BOTH)
        self.fr_task_cmd.pack(fill=tk.BOTH, expand=False, side=tk.LEFT)
        self.fr_config.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)

        self.window.bind("<Key>", self.do_key_press)
        self.window.bind('<Up>', self.do_upKey)
        self.window.bind('<Down>', self.do_downKey)
        self.window.bind("<Button-4>", self.do_mouseRollUp)
        self.window.bind("<Button-5>", self.do_mouseRollDown)
        # self.window.bind("<space>", self.do_spacePressed)

        self.window.mainloop()
        # control_thread.join()

    def set_vision_viewer(self, tab_caption, viewer_parent=None):
        if viewer_parent is None: viewer_parent = self.viewer_parent
        imshow_tab = ttk.Frame(master=viewer_parent)  # add viewer to sensor
        fr_imshow = tk.Frame(master=imshow_tab)
        viewer_parent.add(imshow_tab, text=tab_caption)
        fr_imshow.pack(fill=tk.BOTH, expand=True, side=tk.LEFT, anchor=tk.NW)
        viewer = tk.Label(fr_imshow)
        # viewer.place(x=0, y=0)
        viewer.bind("<Button 1>", self.do_mouse_click)
        viewer.bind("<Motion>", self.do_mouse_move)
        viewer.pack(fill=tk.BOTH, expand=True, side=tk.LEFT, anchor=tk.NW)
        return viewer

    def set_textbox_viewer(self, name='unnamed_configs', viewer_parent=None):
        if viewer_parent is None: viewer_parent = self.viewer_parent
        conf_tab = ttk.Frame(master=viewer_parent)
        viewer_parent.add(conf_tab, text=name)

        conf_viewer = tk.Text(master=conf_tab, width=self.tab_width,
                              height=self.tab_height)  # , font=("Courier", 14))  # , height=10)
        # conf_viewer.grid(row=0, column=0)
        conf_viewer.pack(fill=tk.BOTH)

        return conf_viewer

    def layout_module(self):

        self.btn_w1, self.btn_w2, self.btn_w2_lb, self.btn_w3, self.btn_w4, self.btn_w5 = 17, 7, 5, 3, 2, 1

        hgl_frame = tk.Frame(master=self.fr_task_cmd)
        self.highlight_label = tk.Label(master=hgl_frame, text='   ', height=2)
        self.highlight_label.grid(row=0, column=0)
        hgl_frame.pack()

        btn_frame = tk.Frame(master=self.fr_task_cmd)
        tk.Button(master=btn_frame, text='img', command=self.open_image, width=self.btn_w5).grid(row=0, column=0)
        tk.Button(master=btn_frame, text='dir', command=self.open_dir, width=self.btn_w5).grid(row=0, column=1)
        tk.Button(master=btn_frame, text='root>', command=self.open_rgbd_dir, width=self.btn_w5).grid(row=0, column=2)
        tk.Button(master=btn_frame, text='cfg>', command=self.open_dir_from_args, width=self.btn_w5).grid(row=0,
                                                                                                          column=3)
        tk.Button(master=btn_frame, text='pdf>', command=self.open_image_from_pdf, width=self.btn_w5).grid(row=0,
                                                                                                           column=4)
        tk.Label(master=btn_frame, text='Goto', width=self.btn_w2_lb).grid(row=1, column=0)
        entr = tk.ttk.Entry(master=btn_frame, textvariable=self.str_goto, width=self.btn_w2_lb, state='disabled')
        entr.grid(row=1, column=1)
        tk.Button(master=btn_frame, text='>>>', command=self.goto_im, width=self.btn_w5).grid(row=1, column=2)
        self.input_entries.append(entr)

        for j, test_dir in enumerate(self.test_dirs):
            root_dir, im_dir = os.path.split(test_dir)
            tk.Button(master=btn_frame, text='dir%d' % j,
                      command=partial(self.open_dir_, root_dir, os.path.join(im_dir, '*')), width=self.btn_w5). \
                grid(row=1 + (j // 5), column=j - (j // 5) * 5)
        btn_frame.pack()

        txt_frame = tk.Frame(master=self.fr_task_cmd)
        tk.Label(master=txt_frame, text='Text', width=self.btn_w2_lb).grid(row=0, column=0)
        entr = tk.ttk.Entry(master=txt_frame, textvariable=self.str_input_texts, width=self.btn_w1, state='disabled')
        entr.grid(row=0, column=1)
        self.input_entries.append(entr)
        txt_frame.pack()

        self.viewers, viewer_ind, self.sensor_module_inds = [], 0, []
        for j, module in enumerate(self.modules):
            if 'sensor' not in module.category: continue
            # tk.Label(master=self.fr_task_cmd, text='').pack(fill=tk.X)
            self.add_sensor(sensor=module)
            self.sensor_module_inds.append(j)
            module.viewer_ind = viewer_ind
            viewer = self.set_vision_viewer(tab_caption=module.name)
            viewer_ind += 1
            self.viewers.append(viewer)

        self.viewers.append(self.set_vision_viewer(tab_caption='viewer'))

        # tk.Label(master=self.fr_task_cmd, text='').pack(fill=tk.X)
        fr_chk = tk.Frame(master=self.fr_task_cmd, highlightbackground='green', highlightcolor='green',
                          highlightthickness=2)
        tk.Checkbutton(fr_chk, variable=self.flag_hold_var, text='Hold', width=self.btn_w3).grid(row=0, column=0)
        tk.Checkbutton(fr_chk, variable=self.flag_run_all_var, text='All', width=self.btn_w3).grid(row=0, column=1)
        tk.Checkbutton(fr_chk, variable=self.flag_run_acc_var, text='Acc', width=self.btn_w3).grid(row=0, column=2)
        tk.Checkbutton(fr_chk, variable=self.show_steps_var, text='Step', width=self.btn_w3).grid(row=1, column=0)
        tk.Checkbutton(fr_chk, variable=self.flag_muti_proc, text='Mult', width=self.btn_w3).grid(row=1, column=1)
        tk.Checkbutton(fr_chk, variable=self.flag_loop_var, text='Loop', width=self.btn_w3).grid(row=1, column=2)
        # tk.ttk.Entry(fr_chk, textvariable=self.str_num_run, width=self.btn_w3).grid(row=0, column=3)
        # tk.Label(fr_chk, text='Range', width=self.btn_w2_lb).grid(row=2, column=0)
        tk.Button(fr_chk, text='Sample', command=self.sampling, width=self.btn_w2_lb).grid(row=2, column=0)
        entr = tk.ttk.Entry(fr_chk, textvariable=self.str_range, width=2 * self.btn_w2_lb, state='disabled')
        entr.grid(row=2, column=1, columnspan=2)
        if hasattr(self, 'input_entries'):
            self.input_entries.append(entr)
        else:
            self.input_entries = [entr, ]
        fr_chk.pack()

        if self.add_tcp_client:
            client_module = GuiModule(None, type='tcp_client', name='tcp_client', category='tcp_client', num_method=1)
            self.modules = [client_module, ] + self.modules

        for j, module in enumerate(self.modules):
            if 'sensor' in module.category: continue
            self.add_module(module_ind=j)

        self.add_process_layout()

        btn_frame = tk.Frame(master=self.fr_task_cmd)
        j = 0
        tk.Button(master=btn_frame, text='Load cfg', width=self.btn_w2, command=self.reload_configs). \
            grid(row=j // 2, column=j - 2 * (j // 2))
        j += 1
        self.btn_select_workspace = tk.Button(btn_frame, text='Draw ws', width=self.btn_w2,
                                              command=self.mouse_select_workspace)
        self.btn_select_workspace.grid(row=j // 2, column=j - 2 * (j // 2))
        # j += 1
        # self.btn_select_workspace = tk.Button(btn_frame, text='Draw box3D', width=self.btn_w2,
        #                                       command=self.mouse_select_box3d)
        # self.btn_select_workspace.grid(row=j // 2, column=j - 2 * (j // 2))
        j += 1
        tk.Button(btn_frame, text='Unset ws', width=self.btn_w2, command=self.unset_workspace). \
            grid(row=j // 2, column=j - 2 * (j // 2))
        j += 1
        tk.Button(master=btn_frame, text='Save im', width=self.btn_w2, command=self.save_image). \
            grid(row=j // 2, column=j - 2 * (j // 2))
        j += 1
        tk.Button(master=btn_frame, text='Save seq', width=self.btn_w2, command=self.save_sequence). \
            grid(row=j // 2, column=j - 2 * (j // 2))
        j += 1
        self.btn_save_video = tk.Button(master=btn_frame, text='Save vid', width=self.btn_w2, command=self.save_video)
        self.btn_save_video.grid(row=j // 2, column=j - 2 * (j // 2))
        j += 1
        tk.Button(master=btn_frame, text='Make Vid', width=self.btn_w2, command=self.create_video). \
            grid(row=j // 2, column=j - 2 * (j // 2))
        j += 1
        self.btn_est_depth_map = tk.Button(master=btn_frame, text='Est Depth', width=self.btn_w2,
                                           command=self.mouse_estimate_depth_map)
        self.btn_est_depth_map.grid(row=j // 2, column=j - 2 * (j // 2))
        btn_frame.pack()

        # tk.Label(master=self.fr_task_cmd).pack()
        key_frame = tk.Frame(self.fr_task_cmd)
        self.key_command_viewer = tk.Label(master=key_frame, highlightbackground='green', highlightcolor='green',
                                           highlightthickness=2)
        tk.Checkbutton(key_frame, variable=self.flag_lock_key_press_var, text='Lock Key Press', width=self.btn_w1,
                       command=self.on_off_input_entries).pack(fill=tk.X)
        self.key_command_viewer.pack(fill=tk.X)
        self.update_key_command_viewer()
        key_frame.pack()

        ################################################## EROOOOOOOOOOOOOOOOOOOORRRR
        self.tab_height, self.tab_width = 80, 200
        self.default_conf_viewer = self.set_textbox_viewer('default_configs', viewer_parent=self.viewer_parent)

        # conf_tab = ttk.Frame(master=self.viewer_parent)
        # self.default_conf_viewer = tk.Text(master=conf_tab, width=self.tab_width,
        #                                    height=self.tab_height)  # , font=("Courier", 14))  # , height=10)
        # self.default_conf_viewer.grid(row=0, column=0)
        # self.viewer_parent.add(conf_tab, text='default_configs')

        GuiUtils().update_textbox(self.default_conf_viewer, self.args.to_string())

    def add_process_layout(self):
        modules = [mod for mod in self.modules if "sensor" not in mod.category]
        num_mod = len(modules)
        # if num_mod==0: return
        tk.Label(master=self.fr_task_cmd, text=f'=====PROCESS=====').pack()
        prc_frame = tk.Frame(master=self.fr_task_cmd)
        self.str_processes = tk.StringVar()
        if self.processes is None:
            self.processes = [j + self.num_sensor + mod.default_method_ind / 10 for j, mod in enumerate(modules)]
        # txt = f'{list(range(num_mod))}'[1:-1]
        txt = f'{self.processes}'[1:-1]
        self.str_processes.set(f'{txt}')
        entr = tk.ttk.Entry(master=prc_frame, textvariable=self.str_processes, width=self.btn_w2, state='disabled')
        entr.grid(row=0, column=0)
        self.process_ask_next_var = tk.BooleanVar()
        tk.Checkbutton(prc_frame, variable=self.process_ask_next_var, text='Ask_next', width=self.btn_w2).grid(row=0,
                                                                                                               column=1)
        tk.Button(master=prc_frame, text='Run', command=self.run_process, width=self.btn_w3).grid(row=0, column=2)

        self.str_server_address = tk.StringVar()
        self.str_server_address.set(self.args.conn.address)
        entr1 = tk.ttk.Entry(master=prc_frame, textvariable=self.str_server_address, width=self.btn_w2,
                             state='disabled')
        entr1.grid(row=1, column=0)
        self.str_server_port = tk.StringVar()
        self.str_server_port.set(self.args.conn.port)
        entr2 = tk.ttk.Entry(master=prc_frame, textvariable=self.str_server_port, width=self.btn_w2, state='disabled')
        entr2.grid(row=1, column=1)
        self.btn_server = tk.Button(master=prc_frame, text='Run Server', command=self.run_process_server,
                                    width=self.btn_w2)
        self.btn_server.grid(row=1, column=2)

        self.input_entries = self.input_entries + [entr, entr1, entr2] \
            if hasattr(self, 'input_entries') else [entr, entr1, entr2]

        prc_frame.pack()
        tk.Label(master=self.fr_task_cmd, text=f'{"=" * 16}').pack()

    def on_off_input_entries(self):
        turn_on = self.flag_lock_key_press_var.get()
        if not hasattr(self, 'input_entries'): return
        for entr in self.input_entries:
            if turn_on:
                entr.config(state='enabled')
            else:
                entr.config(state='disabled')

    def get_module_args(self, module):
        # if module.args_ is not None:
        #     if module.args is None: module.args =  module.args_.flatten()   # args not None means there are pre-defined configs
        # if module.args is not None: args = module.args.copy()
        # else: args = CFG()
        # for section_name in self.args.keys():
        #     default_section = getattr(self.args, section_name)
        #     if hasattr(args, section_name):
        #         section = getattr(args, section_name)
        #         for option_name in default_section.keys():
        #             setattr(section, option_name, getattr(default_section, option_name))
        #     else:
        #         setattr(args, section_name, default_section)
        # # [setattr(args_, key, getattr(self.args_, key)) for key in self.args_.keys()]
        # # args = args_.flatten()
        # if module.args is None:  module.args = args
        # else: module.args.merge_with(args)
        if module.args is None:
            module.args = self.args.copy()
        else:
            module.args.merge_with(self.args, force=True)

    def update_all_modules_args(self):
        if self.modules is None: return
        if len(self.modules) == 0: return
        for module in self.modules:
            if not hasattr(module, 'args'): module.args = CFG(cfg_path=module.cfg_path, separate=True)
            self.get_module_args(module)
            if hasattr(module, 'worker'):
                module.worker.args = module.args
                if hasattr(module.worker, 'reload_params'): module.worker.reload_params()

    def update_after_change_args_(self):
        # self.args = self.args_.flatten()
        for module in self.modules:
            if module.cfg_path is not None:
                module.args = CFG(cfg_path=module.cfg_path, separate=True)
                self.get_module_args(module)
                module.args.write(module.cfg_path)
        self.update_all_modules_args()
        self.set_workspace()
        GuiUtils().update_textbox(self.default_conf_viewer, self.args.to_string())

    def init_params(self):
        modules = [module for module in self.modules if 'sensor' in module.category]  # arrange sensor first
        self.num_sensor = len(modules)

        self.update_all_modules_args()
        self.set_workspace()

        self.mouse_loc = (0, 0)
        self.click_locs = []

        self.detected = None
        self.detected_disp = None
        self.tcp_returned = {}
        self.rgbds = [None] * (self.num_sensor + 1)
        self.num_viewer = self.num_sensor + 1
        self.view_scales = [(1, 1)] * self.num_viewer

        # flags
        self.flag_select_workspace = False
        self.flag_show_click_locs = False
        self.flag_show_roi = [None, 'X', 'Y']
        self.flag_imshow_modes = ['rgb', 'depth', 'depth_norm', 'depth_jet']
        # self.flag_show_roi = False
        self.flag_show_detected = False
        self.flag_90_rot = False
        self.flag_show_annotation = False
        self.flag_measure = False

    def open_image(self):
        control_thread = Thread(target=self.open_image_, daemon=True)
        control_thread.start()

    def open_image_(self):
        try:
            filepath = fd.askopenfilename(initialdir='data')
            if ProcUtils().isimpath(filepath):
                self.rgbd_path = [filepath, ]
                # self.rgb = cv2.imread(filepath)[:,:,::-1]
                self.rgbds[-1] = self.get_rgbd_from_path(use_rgb=True, use_depth=False)
                self.update_im_viewer()
                self.viewer_parent.select(self.num_viewer - 1)

            else:
                print('%s is not an image ...' % filepath)
        except:
            pass

    def open_image_from_pdf(self):
        from pdf2image import convert_from_path as read_pdf
        try:
            filepath = fd.askopenfilename(initialdir='data', title='Select a pdf file...',
                                          filetypes=(("pdf files", "*.pdf"), ("all files", "*.*")))
            self.data_root, filename = os.path.split(filepath)
            filename = filename.replace('.pdf', '')
            im_dir = os.path.join(self.data_root, 'image')
            ProcUtils().rmdir(im_dir)
            images = read_pdf(filepath, dpi=300)

            for j, im in enumerate(images):
                im_ind = str(j).rjust(3, '0')
                ArrayUtils().save_array_v3(np.array(im), os.path.join(im_dir, '%s_%s.png' % (filename, im_ind)))

            self.args.path.im_suffixes = ['image/*']
            if hasattr(self.args.path, 'depth_suffixes'): delattr(self.args.path, 'depth_suffixes')
            # self.update_after_change_args_()
            self.open_dir_(data_root=self.data_root)
        except:
            pass

    def open_dir_from_args(self, module=None):
        # if module is not None:
        # self.copy_args_sections_from_module(module_args_=module.args_, section_names=['path'])
        # self.update_after_change_args_()
        if module is not None:
            if hasattr(module.args_, 'path'):
                self.copy_args_sections_from_module(module_args_=module.args_, section_names=['path'])
        # self.open_dir_(data_root=self.args_.path.root_dir)
        control_thread = Thread(target=self.open_dir_, args=(self.args.path.root_dir,), daemon=True)
        control_thread.start()

    def open_dir(self):
        self.args.path.im_suffixes = '*'
        if hasattr(self.args.path, 'depth_suffixes'): delattr(self.args.path, 'depth_suffixes')
        # self.update_after_change_args_()
        control_thread = Thread(target=self.open_dir_, daemon=True)
        control_thread.start()

    def open_rgbd_dir(self):
        self.args.path.im_suffixes = 'image/*'
        self.args.path.depth_suffixes = 'depth/*'
        # self.update_after_change_args_()
        control_thread = Thread(target=self.open_dir_, daemon=True)
        control_thread.start()

    def open_dir_(self, data_root=None, im_suffixes=None, depth_suffixes=None):
        if hasattr(self, 'viewer_parent'): self.viewer_parent.select(self.num_viewer - 1)
        try:
            if data_root is not None:
                self.data_root = data_root
            else:
                self.data_root = fd.askdirectory(initialdir='data', title='Select root directory')
            if im_suffixes is not None: self.args.path.im_suffixes = im_suffixes
            if depth_suffixes is not None: self.args.path.depth_suffixes = depth_suffixes

            self.args.path.root_dir = self.data_root
            self.update_after_change_args_()

            print('Root %s selected ...' % self.data_root)
            self.get_im_list()
            self.num_im = len(self.paths)
            if self.num_im == 0:
                print('No image in %s ...' % self.data_root)
                return

            print('%d images ...' % self.num_im)

            if not hasattr(self, 'im_ind'): self.im_ind = 0
            self.rgbd_path = self.paths[self.im_ind]
            self.rgbds[-1] = self.get_rgbd_from_path()
            self.update_im_viewer()

        except:
            print('Failed to open image dir')

    def next_im(self, forward=True):
        try:
            self.flag_show_detected = False
            self.highlight_label.configure(text='   ')
            tab_ind = self.get_current_tab_ind()
            if tab_ind >= self.num_viewer: return
            if tab_ind < self.num_viewer - 1:
                self.tab2sensor(tab_ind).flag_get_data = True
                return

            if not hasattr(self, 'paths'): return
            # if self.im_ind==(self.num_im-1): return
            try:
                im_range = np.arange(*eval(self.str_range.get()))
            except:
                im_range = np.arange(self.num_im)
            ind = int(np.argmin(np.abs(im_range - self.im_ind)))

            if forward:
                self.im_ind = im_range[ind + 1] if ind < len(im_range) - 1 else im_range[ind]
            else:
                self.im_ind = im_range[ind - 1] if ind > 0 else im_range[ind]
            self.im_ind = min(max(0, self.im_ind), self.num_im - 1)

            self.rgbd_path = self.paths[self.im_ind]
            print('[%d/%d] %s' % (self.im_ind, self.num_im, self.rgbd_path['base']))

            self.rgbds[-1] = self.get_rgbd_from_path()

            if self.hold_detect():
                # self.run_detect_dir_(self.just_executed_module, self.just_executed_method_ind)
                if 'module' in self.just_executed:
                    self.run_detect_(module=self.just_executed['module'],
                                     method_ind=self.just_executed['method_ind'])
                if 'process' in self.just_executed: self.run_process_()
                self.flag_show_detected = True
            else:
                self.flag_show_detected = False

            self.update_im_viewer()


        except:
            pass

    def goto_im(self):
        try:
            self.flag_show_detected = False
            self.highlight_label.configure(text='   ')
            tab_ind = self.get_current_tab_ind()
            if tab_ind >= self.num_viewer: return
            if tab_ind < self.num_viewer - 1:
                self.tab2sensor(tab_ind).flag_get_data = True
                return

            if not hasattr(self, 'paths'): return
            # if self.im_ind==(self.num_im-1): return

            try:
                goto_ind = int(self.str_goto.get())
                if 0 <= goto_ind and goto_ind < self.num_im - 1:
                    self.im_ind = goto_ind
            except:
                pass

            self.rgbd_path = self.paths[self.im_ind]
            print('[%d/%d] %s' % (self.im_ind, self.num_im, self.rgbd_path['base']))

            self.rgbds[-1] = self.get_rgbd_from_path()

            if self.hold_detect():
                # self.run_detect_dir_(self.just_executed_module, self.just_executed_method_ind)
                if 'module' in self.just_executed:
                    self.run_detect_(module=self.just_executed['module'],
                                     method_ind=self.just_executed['method_ind'])
                if 'process' in self.just_executed: self.run_process_()
                self.flag_show_detected = True
            else:
                self.flag_show_detected = False

            self.update_im_viewer()


        except:
            pass

    def hold_detect(self):
        if not self.flag_hold_var.get(): return False
        if not hasattr(self, 'just_executed'): return False
        to_hold = False
        if 'module' in self.just_executed:
            to_hold = self.just_executed['module'].category == 'detector'
        if 'process' in self.just_executed:
            to_hold = self.modules[self.just_executed['process'][-1]].category == 'detector'
        return to_hold

    def mouse_estimate_depth_map(self):
        if self.hold_detect(): return
        self.click_locs = []
        self.flag_show_click_locs = True
        self.flag_measure = True
        self.btn_est_depth_map.configure(text='Calc Depth', command=self.mouse_calc_depth_map)

    def mouse_calc_depth_map(self):
        self.flag_show_click_locs = False
        self.flag_measure = False
        self.btn_est_depth_map.configure(text='Est Depth', command=self.mouse_estimate_depth_map)
        if len(self.click_locs) < 3: return

        # calc depth
        from ketisdk.vision.utils.image_processing import fit_surface_params
        rgbd = self.rgbds[self.get_current_tab_ind()]
        points = np.array(self.click_locs)
        left, top, right, bottom = np.amin(points[0, :]), np.amin(points[1, :]), np.amax(points[0, :]), np.amax(
            points[1, :])
        depth_crop = rgbd.depth[top:bottom, left:right]
        Y, X = np.where(depth_crop > 10)
        Z = depth_crop[(Y, X)]
        X, Y = X + left, Y + top
        self.depth_surface_params = fit_surface_params(
            np.concatenate((X.reshape(-1, 1), Y.reshape(-1, 1), Z.reshape(-1, 1)), axis=1))
        with open('data/depth_surface_params', 'w') as f:
            f.write('\n'.join(str(el) for el in self.depth_surface_params))
        print(f'Depth surface params (Ax+ By +Cz+D=0): {self.depth_surface_params} saved at data/depth_surface_params')

    def mouse_select_workspace(self):
        if self.hold_detect(): return
        self.click_locs = []
        self.flag_show_click_locs = True
        self.flag_measure = True
        self.btn_select_workspace.configure(text='Set ws', command=self.set_mouse_workspace)

    def set_mouse_workspace(self):
        self.flag_show_click_locs = False
        self.flag_measure = False
        self.btn_select_workspace.configure(text='Draw ws', command=self.mouse_select_workspace)
        if len(self.click_locs) < 3: return
        self.args.sensor.crop_poly = [el[:2] for el in self.click_locs]
        self.update_after_change_args_()

        if self.rgbds[-1] is not None:
            self.rgbds[-1].workspace = self.workspace
            self.update_im_viewer()

    def set_mouse_box3d(self):
        self.flag_show_click_locs = False
        self.flag_measure = False
        self.btn_select_workspace.configure(text='Draw box3D', command=self.mouse_select_workspace)

    def unset_workspace(self):
        self.args.sensor.crop_poly, self.args.sensor.crop_rect = None, None
        self.update_after_change_args_()
        if self.rgbds[-1] is not None:
            self.rgbds[-1].workspace = self.workspace
            self.update_im_viewer()

    def update_im_viewer(self, im=None, viewer_ind=-1):
        viewer = self.viewers[viewer_ind]
        # im_on_image = self.im_on_viewers[viewer_ind]
        if im is None:
            im = self.rgbd_disp(rgbd=self.rgbds[viewer_ind])
        h, w = im.shape[:2]
        wo, ho = self.im_disp_scale
        wo, ho = ArrayUtils().fix_resize_scale(in_size=(w, h), out_size=(wo, ho))
        # self.im_disp_scale = (wo, ho)
        self.view_scales[viewer_ind] = (wo / w, ho / h)

        img = ImageTk.PhotoImage(Image.fromarray(cv2.resize(im, (wo, ho))))
        viewer.configure(image=img, text=self.viewer_info, compound=tk.TOP, font=("Courier", 18), anchor=tk.NW)
        viewer.image = img
        # viewer.itemconfigure(im_on_image, image=img)

    def reload_configs(self):
        self.args = CFG().from_string(GuiUtils().textbox2text(self.default_conf_viewer), separate=True)
        self.args.write(path=self.default_cfg_path)
        self.update_after_change_args_()

    def draw_roi(self, im, pt=None):
        if pt is None: pt = self.mouse_loc
        height, width = im.shape[:2]
        out = np.copy(im)
        is_gray = len(out.shape) == 2
        if ArrayUtils().pt_in_im_range(pt, height=height, width=width):
            roi_org = ArrayUtils().crop_array_patch(out, center=pt, pad_size=self.args.disp.pad_size)

            # Display zoomed RoI
            roi = cv2.resize(roi_org, self.args.disp.roi_disp_size, fx=0, fy=0, interpolation=cv2.INTER_CUBIC)
            h, w = roi.shape[:2]

            if self.flag_imshow_modes[0] in ['rgb', 'depth']:
                if self.flag_show_roi[0] == 'X':
                    x = np.arange(w)
                    B = roi.mean(axis=0) if is_gray else roi[:, :, 0].mean(axis=0)
                    B = h - h * B / 255
                    pts = np.vstack((x, B)).astype(np.int32).T
                    cv2.polylines(roi, [pts], isClosed=False, color=(255, 0, 0))

                    if not is_gray:
                        G = roi[:, :, 1].mean(axis=0)
                        G = h - h * G / 255
                        R = roi[:, :, 2].mean(axis=0)
                        R = h - h * R / 255
                        pts = np.vstack((x, G)).astype(np.int32).T
                        cv2.polylines(roi, [pts], isClosed=False, color=(0, 255, 0))
                        pts = np.vstack((x, R)).astype(np.int32).T
                        cv2.polylines(roi, [pts], isClosed=False, color=(0, 0, 255))

                if self.flag_show_roi[0] == 'Y':
                    y = np.arange(h)
                    B = roi.mean(axis=1) if is_gray else roi[:, :, 0].mean(axis=1)
                    B = w * B / 255
                    pts = np.vstack((B, y)).astype(np.int32).T
                    cv2.polylines(roi, [pts], isClosed=False, color=(255, 0, 0))

                    if not is_gray:
                        G = roi[:, :, 1].mean(axis=1)
                        G = w * G / 255
                        R = roi[:, :, 2].mean(axis=1)
                        R = w * R / 255
                        pts = np.vstack((G, y)).astype(np.int32).T
                        cv2.polylines(roi, [pts], isClosed=False, color=(0, 255, 0))
                        pts = np.vstack((R, y)).astype(np.int32).T
                        cv2.polylines(roi, [pts], isClosed=False, color=(0, 0, 255))

            cv2.drawMarker(roi, (w // 2, h // 2), self.args.disp.marker_color, cv2.MARKER_CROSS,
                           self.args.disp.marker_size, self.args.disp.marker_thick)

            xm, ym = pt
            ss = 50

            y_end = ym + ss + h
            if y_end >= height: y_end = ym - ss
            x_end = xm + ss + w
            if x_end >= width: x_end = xm - ss
            if is_gray:
                out[y_end - h:y_end, x_end - w:x_end] = roi
            else:
                out[y_end - h:y_end, x_end - w:x_end, :] = roi

        return out

    def do_show_detect(self):
        return self.flag_show_detected and self.detected is not None

    def show_rgbd_3D(self, ):
        tab_ind = self.get_current_tab_ind()
        if tab_ind >= self.num_viewer: return
        rgbd = self.rgbds[tab_ind]

        bg_depth_params = None
        try:
            with open('data/depth_surface_params', 'r') as f:
                bg_depth_params = [float(el) for el in f.readlines()]
        except:
            print('Failed to read bg depth params')

        cam_info = self.args.sensor.rgb_size[ ::-1] + self.args.sensor.intrinsic + (self.args.sensor.depth_scale,)
        cam_info = CameraInfo(*cam_info)
        if tab_ind < self.num_sensor:
            cam_info = self.modules[self.sensor_module_inds[tab_ind]].worker.get_info()
        cloud = rgbd.show_3D(camera_info=cam_info, bg_depth_params=bg_depth_params)

    def rgbd_disp(self, rgbd=None, intr_params=None):
        # timer = Timer()

        show_detect = self.do_show_detect()
        if show_detect:
            rgbd_disp = self.detected_disp
        else:
            if rgbd is None:
                return np.zeros((720, 1080), 'uint8')
            rgbd_disp = rgbd.disp(mode=self.flag_imshow_modes[0])

        if rgbd_disp is None: return np.zeros((720, 1080), 'uint8')
        if self.flag_show_roi[0] is not None: rgbd_disp = self.draw_roi(rgbd_disp)

        disp_height, disp_width = rgbd_disp.shape[:2]
        self.viewer_info = f'Size: {disp_width}x{disp_height}, Mode: {self.flag_imshow_modes[0]}\n'

        if rgbd is not None:
            if rgbd.pt_in_im_range(self.mouse_loc):
                realLoc = rgbd.get_locVsCamCoor(self.mouse_loc, intr_params=intr_params)
                realLoc = f'{realLoc.astype("int").flatten().tolist()[:2]}' if realLoc is not None else ''
                self.viewer_info += f'{self.mouse_loc}{realLoc}{rgbd.val_str(self.mouse_loc)}\n'

        if ArrayUtils().pt_in_im_range(self.mouse_loc, height=disp_height, width=disp_width):
            do_show_real_value = False if self.detected is None else 'array_real' in self.detected
            if do_show_real_value:
                self.viewer_info += f'{self.detected["array_real"][self.mouse_loc[::-1]]}'
            else:
                self.viewer_info += f'{rgbd_disp[self.mouse_loc[::-1]]}'

        if self.flag_show_click_locs: rgbd_disp = PolyUtils().draw_poly(rgbd_disp, [el[:2] for el in self.click_locs])
        # if self.flag_show_click_locs:
        #     for pt in self.click_locs:
        #         rgbd_disp = cv2.drawMarker(rgbd_disp, pt, self.args.disp.marker_color, cv2.MARKER_TILTED_CROSS,
        #                                    self.args.disp.marker_size, self.args.disp.marker_thick)

        if self.flag_measure:
            xm, ym = self.mouse_loc
            ss = self.args.disp.line_thick
            x0, y0 = ArrayUtils().truncate(xm - ss // 2, 0, rgbd.width), ArrayUtils().truncate(ym - ss // 2, 0,
                                                                                               rgbd.height)
            x1, y1 = ArrayUtils().truncate(x0 + ss, 0, rgbd.width), ArrayUtils().truncate(y0 + ss, 0, rgbd.height)
            rgbd_disp[:, x0:x1, :] = self.args.disp.line_color
            rgbd_disp[y0:y1, :, :] = self.args.disp.line_color

            if len(self.click_locs) > 0:
                x0, y0 = self.click_locs[-1][:2]
                x1, y1 = self.mouse_loc
                xc, yc = (x0 + x1) // 2, (y0 + y1) // 2
                dx, dy = abs(x1 - x0), abs(y1 - y0)
                dd = np.linalg.norm((dx, dy))

                cv2.line(rgbd_disp, (x0, y0), (x1, y1), self.args.disp.line_color, self.args.disp.line_thick)
                cv2.putText(rgbd_disp, 'dx:%d-dy:%d-l:%.1f' % (dx, dy, dd), (xc, yc), cv2.FONT_HERSHEY_COMPLEX,
                            self.args.disp.text_scale, self.args.disp.text_color, self.args.disp.text_thick)

        # if self.flag_90_rot: rgbd_disp = np.rot90(rgbd_disp, -1)
        # timer.pin_time('option_disp')
        # print(timer.pin_times_str())
        return rgbd_disp

    def do_mouse_move(self, event):
        try:
            tab_ind = self.get_current_tab_ind()
            if tab_ind >= self.num_viewer: return
            x0, y0 = event.x, event.y

            rgbd = self.rgbds[tab_ind]
            if rgbd is None: return

            fx, fy = self.view_scales[tab_ind]
            x0, y0 = int(x0 / fx), int(y0 / fy)
            self.mouse_loc = (x0, y0)

            if tab_ind == (self.num_viewer - 1):
                self.update_im_viewer()
        except:
            pass

    def do_mouse_click(self, event):
        # mouse_loc = self.correct_mouse_loc((event.x, event.y))
        try:
            rgbd = self.rgbds[self.get_current_tab_ind()]
            x, y = self.mouse_loc
            loc = (x, y, rgbd.depth[(y, x)]) if rgbd.hasDepth else (x, y)
            self.click_locs.append(loc)
        except:
            pass

    def do_mouseRollUp(self, event):
        if hasattr(self, 'flag_rollUp'):
            self.flag_rollUp = True
        else:
            self.__setattr__('flag_rollUp', True)
        self.do_mouseRoll_()

    def do_mouseRollDown(self, event):
        if hasattr(self, 'flag_rollUp'):
            self.flag_rollUp = False
        else:
            self.__setattr__('flag_rollUp', False)
        self.do_mouseRoll_()

    # def do_mouseRollDown(self, event):
    #     if hasattr(self, 'flag_rollUp'):
    #         self.flag_rollUp = False
    #     else:
    #         self.__setattr__('flag_rollUp', False)
    #     self.do_mouseRoll_()

    def do_mouseRoll_(self):
        if self.flag_rollUp:
            rat = 1.05
        else:
            rat = 1 / 1.05

        self.im_disp_scale = (int(self.im_disp_scale[0] * rat), int(self.im_disp_scale[1] * rat))
        if self.get_current_tab_ind() != self.num_viewer - 1: return
        if self.rgbds[-1] is None: return
        self.update_im_viewer()

    def do_spacePressed(self, event):
        self.save_image()

    def do_key_press(self, event):
        if self.flag_lock_key_press_var.get(): return
        tab_ind = self.get_current_tab_ind()
        if tab_ind >= self.num_viewer: return
        self.pressed_key = event.char
        if self.pressed_key not in self.key_commands: return 1

        key_command = self.key_commands[self.pressed_key]

        if key_command == 'show_annotation':
            self.flag_show_annotation = True
            ProcUtils().change_cmd_dict(self.key_commands, prev='show_annotation', lat='stop_show_annotation')
            self.update_key_command_viewer()
            rgbd = self.rgbds[-1]
            if rgbd is None: return
            self.update_im_viewer(im=self.rgbd_disp(rgbd)[:, :, ::-1])

        if key_command == 'stop_show_annotation':
            self.flag_show_annotation = False
            ProcUtils().change_cmd_dict(self.key_commands, prev='stop_show_annotation', lat='show_annotation')
            self.update_key_command_viewer()
            rgbd = self.rgbds[-1]
            if rgbd is None: return
            self.update_im_viewer(im=self.rgbd_disp(rgbd))

        if key_command == 'switch_imshow':
            self.flag_imshow_modes = self.flag_imshow_modes[1:] + [self.flag_imshow_modes[0]]

            rgbd = self.rgbds[-1]
            if rgbd is None: return
            self.update_im_viewer(im=self.rgbd_disp(rgbd))

        if key_command == 'show_roi':
            self.flag_show_roi = self.flag_show_roi[1:] + [self.flag_show_roi[0], ]

        if key_command == '90_rot':
            self.flag_90_rot = True
            ProcUtils().change_cmd_dict(self.key_commands, prev='90_rot', lat='stop_90_rot')
            self.update_key_command_viewer()

        if key_command == 'stop_90_rot':
            self.flag_90_rot = False
            ProcUtils().change_cmd_dict(self.key_commands, prev='stop_90_rot', lat='90_rot')
            self.update_key_command_viewer()

        if key_command == 'crop_roi':
            self.mouse_crop_roi()
            ProcUtils().change_cmd_dict(self.key_commands, prev='crop_roi', lat='save_roi')

            self.update_key_command_viewer()

        def crop_all_images(keep_name=True):
            if self.get_current_tab_ind() != self.num_viewer - 1: return
            if not hasattr(self, 'paths'):
                print('Load image before cropping ...')
                return
            crop_dir = 'data/gui_crop'
            crop_rgb_dir = os.path.join(crop_dir, 'rgb')
            crop_depth_dir = os.path.join(crop_dir, 'depth')
            os.makedirs(crop_rgb_dir, exist_ok=True)
            os.makedirs(crop_depth_dir, exist_ok=True)
            for path in self.paths:
                rgbd = self.get_rgbd_from_path(path)
                if rgbd is None: continue
                rgbd_crop = rgbd.crop()
                hasRgb = 'im' in path
                hasDepth = 'depth' in path
                if hasRgb:
                    name = os.path.split(path['im'])[-1].replace('_rgb', '') if keep_name \
                        else datetime.datetime.now().strftime('%y%m%d%H%M%S%f') + '.png'
                if hasDepth:
                    name = os.path.split(path['depth'])[-1].replace('_depth', '') if keep_name \
                        else datetime.datetime.now().strftime('%y%m%d%H%M%S%f') + '.png'
                if hasRgb: ArrayUtils().save_array_v3(rgbd_crop.bgr(), os.path.join(crop_rgb_dir, name))
                if hasDepth: ArrayUtils().save_array_v3(rgbd_crop.depth, os.path.join(crop_depth_dir, name))
            print('All images cropped ....')

        if key_command == 'crop_all':
            crop_all_images(keep_name=True)

        if key_command == 'crop_all_rename':
            crop_all_images(keep_name=False)

        if key_command == 'save_roi':
            self.mouse_save_roi(tab_ind)
            ProcUtils().change_cmd_dict(self.key_commands, prev='save_roi', lat='crop_roi')
            self.update_key_command_viewer()

        if key_command == 'measure':
            self.click_locs = []
            self.flag_measure = True
            ProcUtils().change_cmd_dict(self.key_commands, prev='measure', lat='stop_measure')
            self.update_key_command_viewer()

        if key_command == 'stop_measure':
            self.flag_measure = False
            ProcUtils().change_cmd_dict(self.key_commands, prev='stop_measure', lat='measure')
            self.update_key_command_viewer()

        if key_command == 'show_3D':
            Thread(target=self.show_rgbd_3D, daemon=True).start()

    def mouse_crop_roi(self):
        self.click_locs = []
        self.flag_show_click_locs = True
        self.flag_measure = True

    def mouse_save_roi(self, tab_ind):
        if len(self.click_locs) > 2:
            ws = WorkSpace(pts=[(x, y) for x, y, _ in self.click_locs])
            left, top, right, bottom = ws.bbox
            rgbd_crop = self.rgbds[tab_ind].crop(left=left, right=right, top=top, bottom=bottom)
            mask = ws.get_mask_crop()
            unmasked = np.where(mask == 0)

            filename = ProcUtils().get_current_time_str()
            fold = 'data/gui_crop'
            ArrayUtils().save_array_v3((255 * mask).astype('uint8'), os.path.join(fold, filename + '_mask.png'))
            if rgbd_crop.hasRgb:
                ArrayUtils().save_array_v3(rgbd_crop.bgr(), os.path.join(fold, filename + '_rgb.png'))
                bgr_mask = np.copy(rgbd_crop.bgr())
                bgr_mask[unmasked] = 0
                ArrayUtils().save_array_v3(bgr_mask, os.path.join(fold, filename + '_rgb_mask.png'))
            if rgbd_crop.hasDepth:
                ArrayUtils().save_array_v3(rgbd_crop.depth, os.path.join(fold, filename + '_depth.png'))
                depth_mask = np.copy(rgbd_crop.depth)
                depth_mask[unmasked] = 0
                ArrayUtils().save_array_v3(depth_mask, os.path.join(fold, filename + '_depth_mask.png'))
            ProcUtils().open_dir(fold)

        self.click_locs = []
        self.flag_show_click_locs = False
        self.flag_measure = False

    def do_upKey(self, event):
        self.next_im(forward=False)

    def do_downKey(self, event):
        self.next_im(forward=True)

    def update_key_command_viewer(self):
        cmd_text = ''
        for key_str in self.key_commands:
            cmd_text += '%s: %s\n' % (key_str, self.key_commands[key_str])
        self.key_command_viewer.configure(text=cmd_text)

    def get_current_tab_ind(self):
        return self.viewer_parent.index(self.viewer_parent.select())

    def get_im_list(self):
        """ get image path lists """
        self.paths = []

        # if not self.use_rgb and not self.use_depth:
        #     print(f'{"+" * 10} Please check im_suffixes and depth_suffixes')
        #     return

        data_types = [k.replace('_suffixes', '') for k in self.args.path.keys() if k.endswith('_suffixes')]
        data_suffixes = [self.args.path.__getattribute__(f'{p}_suffixes') for p in data_types]
        data_dict = dict()
        for t, s in zip(data_types, data_suffixes):
            data_dict.update({t: [s, ] if isinstance(s, str) else s})

        if 'im' in data_dict:
            base_suffixes = data_dict['im']
        else:
            base_suffixes = data_dict['depth']

        for j, b_suf in enumerate(base_suffixes):
            b_paths = sorted(glob(os.path.join(self.data_root, b_suf)))
            for b_path in b_paths:
                apath = dict()
                b_path = b_path.replace('\\', '/')
                if not ProcUtils().isimpath(b_path) and not b_path.endswith('.npy'): continue

                for d_type in data_dict:
                    d_sufs = data_dict[d_type]
                    if d_sufs is None: continue
                    d_suf = d_sufs[j]
                    if d_suf is None: continue
                    d_path = b_path
                    for bb, dd in zip(b_suf.split('*'), d_suf.split('*')):
                        d_path = d_path.replace(bb, dd)
                    if not os.path.exists(d_path) and not d_path.endswith('.npy'): continue
                    apath.update({d_type: d_path})
                apath.update({'base': b_path})
                self.paths.append(apath)

    def get_rgbd_from_path(self, rgbd_path=None, use_rgb=None, use_depth=None):
        """
        :param paths: list of rgbd path. paths[0]: image path, paths[-1]: depth path
        """
        try:
            if rgbd_path is None: rgbd_path = self.rgbd_path
            if use_rgb is None: use_rgb = 'im' in rgbd_path
            if use_depth is None: use_depth = 'depth' in rgbd_path
            rgb, depth = None, None
            if use_rgb:
                if ProcUtils().isimpath(rgbd_path['im']): rgb = cv2.imread(rgbd_path['im'])[:, :, ::-1]
                if rgbd_path['im'].endswith('.npy'): rgb = cv2.load(rgbd_path['im'])
                if self.flag_90_rot: rgb = np.rot90(rgb)
            if use_depth:
                if ProcUtils().isimpath(rgbd_path['depth']): depth = cv2.imread(rgbd_path['depth'], -1)
                if rgbd_path['depth'].endswith('.npy'):
                    depth = np.load(rgbd_path['depth']).squeeze()
                # if rgbd_path['depth'].endswith('.ply'):
                if np.amax(depth) < 10: depth = (1000 * depth).astype('uint16')
                if self.flag_90_rot: depth = np.rot90(depth)

            extra = dict()
            for k in rgbd_path:
                if k in ['im', 'depth']: continue
                extra.update({f'{k}_path': rgbd_path[k]})

            rgbd = RGBD(rgb=rgb, depth=depth, workspace=self.workspace, denoise_ksize=self.args.sensor.denoise_ksize,
                        depth_min=self.args.sensor.depth_min, depth_max=self.args.sensor.depth_max, extra=extra)
        except:
            rgbd = None
            print('Failed to get rgbd path')

        return rgbd

    def set_workspace(self):
        pts, crop_rect, bound_margin = None, None, None
        if hasattr(self.args, 'sensor'):
            if hasattr(self.args.sensor, 'crop_poly'): pts = self.args.sensor.crop_poly
            if hasattr(self.args.sensor, 'crop_rect'): crop_rect = self.args.sensor.crop_rect
            if hasattr(self.args.sensor, 'bound_margin'): bound_margin = self.args.sensor.bound_margin

        if pts is None and crop_rect is None:
            self.workspace = None
        else:
            self.workspace = WorkSpace(pts=pts, bbox=crop_rect, bound_margin=bound_margin)

    def set_workspace_from_cfg(self):
        if self.cfg_crop_poly is not None:
            ProcUtils().replace_confile_value(self.guiConfile, 'sensor', 'crop_poly', self.cfg_crop_poly)

        if self.cfg_crop_rect is not None:
            ProcUtils().replace_confile_value(self.guiConfile, 'sensor', 'crop_rect', self.cfg_crop_rect)

        self.guiConfile2textview()
        self.update_conf_viewer()
        self.reload_configs()

        if self.rgbds[-1] is not None:
            self.rgbds[-1].workspace = self.workspace
            self.update_im_viewer()

    def run_sensor(self, sensor):
        self.viewer_parent.select(sensor.viewer_ind)
        sensor.worker = sensor.module(nbuf=self.sensor_nbuf)
        sensor.worker.start(device_serial=sensor.serial)
        sensor.on = True

        sensor.btn_run.configure(text='Stop', command=partial(self.stop_sensor, sensor))
        sensor.flag_get_data = True
        control_thread = Thread(target=self.run_sensor_, args=(sensor,), daemon=True)
        control_thread.start()
        # self.run_sensor_(sensor_ind=sensor_ind)

    def run_sensor_(self, sensor):

        while sensor.on:
            timer = Timer()
            if sensor.flag_get_data:
                self.rgbds[sensor.viewer_ind] = sensor.worker.get_rgbd(workspace=self.workspace,
                                                                       rot_90=self.flag_90_rot,
                                                                       depth_min=self.args.sensor.depth_min,
                                                                       depth_max=self.args.sensor.depth_max,
                                                                       denoise_ksize=self.args.sensor.denoise_ksize)

            if self.hold_detect():
                if 'module' in self.just_executed:
                    self.run_detect_(module=self.just_executed['module'],
                                     method_ind=self.just_executed['method_ind'], show_fps=True)
                if 'process' in self.just_executed:
                    self.run_process_(show_fps=True)
                sensor.flag_get_data = True
                continue
            intr_params = sensor.worker.intr_params if hasattr(sensor.worker, 'intr_params') else None
            rgbd_disp = self.rgbd_disp(self.rgbds[sensor.viewer_ind], intr_params=intr_params)

            if sensor.flag_get_data:
                cv2.putText(rgbd_disp, '%.2f FPS' % timer.fps(), (50, 50),
                            cv2.FONT_HERSHEY_COMPLEX, self.args.disp.text_scale, self.args.disp.text_color,
                            self.args.disp.text_thick)
            # self.rgbd_disp()
            self.update_im_viewer(rgbd_disp, viewer_ind=sensor.viewer_ind)

    def pause_sensor(self, sensor):
        if not hasattr(sensor, 'worker'): return
        self.viewer_parent.select(sensor.viewer_ind)
        sensor.on = False
        sensor.btn_pause.configure(text='Resume', command=partial(self.resume_sensor, sensor))

    def resume_sensor(self, sensor):
        if not hasattr(sensor, 'worker'): return
        self.viewer_parent.select(sensor.viewer_ind)

        sensor.on = True
        sensor.btn_pause.configure(text='Pause', command=partial(self.pause_sensor, sensor))
        control_thread = Thread(target=self.run_sensor_, args=(sensor,), daemon=True)
        control_thread.start()

    def stop_sensor(self, sensor):
        self.viewer_parent.select(sensor.viewer_ind)
        sensor.on = False
        time.sleep(1)
        sensor.worker.stop()
        sensor.btn_run.configure(text='Run', command=partial(self.run_sensor, sensor))

    def add_sensor(self, sensor):

        btn_frame = tk.Frame(master=self.fr_task_cmd)
        sensor_type = sensor.type

        lb = tk.Label(master=btn_frame, text=sensor.short_name)
        lb.grid(row=0, column=0)

        btn_run = tk.Button(master=btn_frame, text='Run', width=self.btn_w2_lb,
                            command=partial(self.run_sensor, sensor))
        btn_run.grid(row=0, column=1)

        btn_pause = tk.Button(master=btn_frame, text='Pause', width=self.btn_w2_lb,
                              command=partial(self.pause_sensor, sensor))
        btn_pause.grid(row=0, column=2)
        btn_frame.pack()

        sensor.btn_run = btn_run
        sensor.btn_pause = btn_pause

    def init_module(self, module, change_cfg_path=False, use_thread=True):
        if use_thread:
            control_thread = Thread(target=self.init_module_, args=(module, change_cfg_path), daemon=True)
            control_thread.start()
        else:
            self.init_module_(module=module, change_cfg_path=change_cfg_path)

    def init_module_(self, module, change_cfg_path=False):
        try:
            # if module.cfg_path is None: change_cfg_path = True
            if change_cfg_path:
                module.cfg_path = fd.askopenfilename(
                    initialdir='configs', title='Select a {} config file'.format(module.type))
                if len(module.cfg_path) == 0: module.cfg_path = None

            if module.cfg_path is not None:
                if os.path.exists(module.cfg_path):
                    module.args = CFG(cfg_path=module.cfg_path, separate=True)
                else:
                    print(f'{module.cfg_path} does not exist ...')
                #     return
            if module.default_cfg_path is not None: module.args.merge_with(
                CFG(cfg_path=module.default_cfg_path, separate=True))
            self.get_module_args(module)
            worker = module.module(args=module.args, cfg_path=module.cfg_path)

            setattr(module, 'worker', worker)
            module.args = module.worker.args
            print('{} {} initialized ...'.format('+' * 10, module.name))

            if module.run_after_init_method_ind is not None:
                self.run_module(module, method_ind=module.run_after_init_method_ind)
        except:
            pass

    # def run_module(self, module, method_ind=0, run_thread=None, record_module=True, **kwargs):
    def run_module(self, module, method_ind, run_thread=True, record_module=True, **kwargs):
        print('{} {} run{} running ...'.format('+' * 10, module.name, method_ind))
        if record_module: self.just_executed = {'module': module, 'method_ind': method_ind}
        mod_category = module.category
        if mod_category == 'detector':
            run_module_ = self.run_detect_
        elif mod_category == 'dataset':
            run_module_ = self.run_dataset_
        elif mod_category == 'robot':
            run_module_ = self.run_robot
        elif mod_category == 'acquisition':
            run_module_ = self.run_acquisition
        elif mod_category == 'tcp_client':
            run_module_ = self.run_tcp_client
        else:
            run_module_ = self.run_general_module_

        if run_thread is None: run_thread = module.run_thread
        if run_thread:
            self.sub_threads = []
            # control_thread = Thread(target=run_module_, args=(kwargs), daemon=True)
            control_thread = Thread(target=run_module_, args=(module, method_ind), kwargs=kwargs, daemon=True)
            control_thread.start()
            # control_thread.join()
        else:
            run_module_(module, method_ind)

    def run_general_module_(self, module, method_ind=0):
        if not hasattr(module, 'worker'): return
        module.worker.gui_run_module(method_ind=method_ind)

    def run_dataset_(self, module, method_ind=0):
        timer = Timer()
        tab_ind = self.get_current_tab_ind()
        if tab_ind >= self.num_viewer: return
        # self.viewer_parent.select(self.num_viewer-1)
        # module.worker.args.show_steps = self.show_steps_var.get()

        rgbd = self.rgbds[-1]
        _, filename = os.path.split(self.rgbd_path[0])
        self.detected = module.worker.gui_show_single(rgbd=rgbd, method_ind=method_ind, filename=filename)
        self.show_detected(rgbd, mod_type=module.type)

        print('{} {} executed in {}'.format('+' * 10, module.name, timer.pin_times_str()))
        self.flag_show_detected = True

    def tab2sensor(self, tab_ind):
        for mod in self.modules:
            if 'sensor' not in mod.category: continue
            if tab_ind == mod.viewer_ind:
                sensor = mod
                break
        return sensor

    def run_robot(self, module, method_ind=0):
        tab_ind = self.get_current_tab_ind()
        rgbd = self.rgbds[tab_ind]
        module.worker.gui_run(method_ind=method_ind, detected=self.detected, click_locs=self.click_locs)

    def run_acquisition(self, module, method_ind=0):
        input_shortname = module.input_var.get()
        sensor = None
        for mod in self.modules:
            if input_shortname == mod.short_name:
                sensor = mod.worker
        module.worker.gui_run(sensor=sensor, method_ind=method_ind, detected=self.detected)

    def run_detect_(self, module, method_ind=0, show_fps=False, show_detected=True, **kwargs):
        tab_ind = self.get_current_tab_ind()
        if tab_ind >= self.num_viewer: return
        if not hasattr(module, 'worker'): return
        # self.just_executed_type = 'detector'
        flag_select_sensor = tab_ind < (self.num_viewer - 1)
        module.worker.args.flag.show_steps = self.show_steps_var.get()

        if show_fps: timer = Timer()
        # input_name = module.input_var.get()
        # if input_name =='Img':
        timer = Timer()
        if flag_select_sensor:
            self.run_detect_sensor_(module, method_ind, self.tab2sensor(tab_ind))
        else:
            self.run_detect_dir_(module, method_ind)
        self.flag_show_detected = show_detected

        # else:
        #     if self.detected is None: return
        #     if input_name != self.detected['module_name']: return
        #     self.execute_detect_and_show(module,method_ind,self.detected['rgbd'], self.detected['filename'], detected=self.detected)

        if show_fps: cv2.putText(self.detected_disp, '%.2f FPS' % timer.fps(), (50, 50),
                                 cv2.FONT_HERSHEY_COMPLEX, self.args.disp.text_scale, self.args.disp.text_color,
                                 self.args.disp.text_thick)
        # else: print(f'Runtime: {timer.run_time_str()}')
        if self.detected_disp is not None and self.flag_show_detected:
            self.update_im_viewer(im=self.rgbd_disp(self.rgbds[tab_ind]), viewer_ind=tab_ind)

    # def run_secondary_detect(self, module, method_ind, clean_detect=False):
    #     if self.detected is None: return
    #     if module.input_var.get()!=self.detected['module_name']: return
    #     self.execute_detect_and_show(module,method_ind,self.detected['rgbd'], self.detected['filename'])
    #     # self.update_im_viewer()
    #     if clean_detect: self.detected=None

    def show_detected(self, viewer_ind=0):
        # self.detected_disp = None
        if self.detected is None: return
        # if hasattr(self.detected, 'im'):
        if 'im' in self.detected:
            self.detected_disp = np.copy(self.detected['im'])
        # self.update_im_viewer(im=self.detected_disp, viewer_ind=viewer_ind)

    def execute_detect_and_show(self, module, method_ind, rgbd, filename='unnamed.png', viewer_ind=0, detected=None,
                                sub_thread=None, im_dir=None, sensor=None, workspace=None):
        self.detected = module.worker.gui_process_single(
            rgbd=rgbd, method_ind=method_ind, filename=filename, disp_mode=self.flag_imshow_modes[0], detected=detected,
            sub_thread=sub_thread, im_dir=im_dir, sensor=sensor, workspace=workspace,
            input_texts=self.str_input_texts.get(),
            rgb_buf=sensor.rgb_buf if sensor else None, depth_buf=sensor.depth_buf if sensor else None)
        if isinstance(self.detected, dict):
            self.detected.update({'module_name': module.short_name, 'filename': filename})
            if 'rgbd' not in self.detected: self.detected.update({'rgbd': RGBD(rgb=self.detected['im'],
                                                                               workspace=rgbd.workspace,
                                                                               depth_min=rgbd.depth_min,
                                                                               depth_max=rgbd.depth_max,
                                                                               denoise_ksize=self.args.sensor.denoise_ksize), })

            announce = self.detected[
                'announce'] if 'announce' in self.detected else f'{module.short_name} run{method_ind} finished'
            self.highlight_label.configure(text=announce)
        # self.flag_show_detected = True
        self.show_detected(viewer_ind=viewer_ind)

    def run_detect_tcp_(self, module, method_ind=0):
        db_result_dir = os.path.join('data/tcp_data', f'{module.short_name}_run{method_ind}')
        file_name = f'data/tcp_data/{ProcUtils().get_current_time_str()}.png'
        rgbd = self.rgbds[-1]
        if rgbd.hasRgb:
            ArrayUtils().save_array_v3(rgbd.bgr(), os.path.join('data/tcp_data/rgb', file_name))
        if rgbd.hasDepth:
            ArrayUtils().save_array_v3(rgbd.depth, os.path.join('data/tcp_data/depth', file_name))
        self.run_detect_dir_single(module, method_ind, file_name, self.rgbds[-1], db_result_dir)

    def run_detect_sensor_(self, module, method_ind, sensor):
        sensor.flag_get_data = False
        sensor_root_dir = os.path.join('data', sensor.type, ProcUtils().get_current_time_str('%m%d'))
        # sensor_result_dir = os.path.join(sensor_root_dir, '{}_run{}'.format(module.short_name, method_ind + 1))

        rgbd = self.rgbds[sensor.viewer_ind]
        input_name = module.input_var.get()
        detected = None if input_name == 'src' else self.detected
        filename = ProcUtils().get_current_time_str() + '.png'
        self.execute_detect_and_show(module, method_ind, rgbd, viewer_ind=sensor.viewer_ind,
                                     detected=detected, filename=filename, sensor=sensor.worker,
                                     workspace=self.workspace)

        if not self.hold_detect() and self.detected_disp is not None:
            # save
            filename = ProcUtils().get_current_time_str()
            ArrayUtils().save_array_v3(rgbd.bgr(), os.path.join(sensor_root_dir, f'{filename}_rgb.png'))
            ArrayUtils().save_array_v3(rgbd.depth, os.path.join(sensor_root_dir, f'{filename}_depth.png'))
            if self.args.sensor.save_disp:
                ArrayUtils().save_array_v3(rgbd.disp(mode=self.flag_imshow_modes[0])[:, :, ::-1],
                                           os.path.join(sensor_root_dir, f'{filename}_disp.png'))
            if len(self.detected_disp.shape) < 3:
                out = self.detected_disp
            else:
                out = self.detected_disp[:, :, ::-1]
            ArrayUtils().save_array_v3(out, os.path.join(sensor_root_dir,
                                                         f'{filename}_{module.short_name}_{method_ind}.png'))

    def run_detect_dir_single(self, module, method_ind, path, rgbd, db_result_dir, sub_thread=None):
        im_dir, filename = os.path.split(path['base']) if 'base' in path else os.path.split(path)
        db_result_dir = os.path.join(os.path.split(im_dir)[0], 'detected')
        # timer = Timer()
        input_name = module.input_var.get()
        detected = None if input_name == 'src' else self.detected
        self.execute_detect_and_show(module, method_ind, rgbd, filename, detected=detected,
                                     sub_thread=sub_thread, im_dir=im_dir)

        # timer.pin_time('predict_time')
        # print(timer.pin_times_str())

        # save
        if self.detected_disp is not None:
            # filename = os.path.split(path['base'])[-1] if 'base' in path else path
            if len(self.detected_disp.shape) < 3:
                out = self.detected_disp
            else:
                out = self.detected_disp[:, :, ::-1]
            ArrayUtils().save_array_v3(out, os.path.join(db_result_dir, filename))
            # if module.run_thread:  self.update_im_viewer(im=self.detected_disp)

    def run_detect_dir_(self, module, method_ind=0):
        db_result_dir = os.path.join(self.args.path.root_dir, f'{module.short_name}_run{method_ind}')
        flag_run_all = self.flag_run_all_var.get() or self.flag_run_acc_var.get()
        flag_run_acc = self.flag_run_acc_var.get()
        flag_multi_proc = self.flag_muti_proc.get() and flag_run_all

        if flag_run_all:
            paths = self.paths
        else:
            if hasattr(self, 'rgbd_path'):
                paths = [self.rgbd_path]
            else:
                print('{} No image loaded...'.format('+' * 10))
                if self.rgbds[-1] is not None:
                    self.run_detect_tcp_(module, method_ind)
                return

        num_im = len(paths)
        # try:
        #     range_min = min(max(0, int(self.str_range_min.get())), num_im - 1)
        # except:
        #     range_min = 0
        # try:
        #     range_max = min(max(0, int(self.str_range_max.get())), num_im - 1)
        # except:
        #     range_max = num_im
        # range_max = max(range_min, range_max)
        try:
            im_range = range(*eval(self.str_range.get()))
        except:
            im_range = range(num_im)
        num_cpu_use = multiprocessing.cpu_count() - 2

        if flag_run_acc: module.worker.init_acc()
        # j = -1
        # for j, path in enumerate(paths):
        do_loop = True
        while do_loop:
            do_loop = self.flag_loop_var.get()
            # if hasattr(module.worker, 'reload_params'): module.worker.reload_params()
            for i, j in enumerate(im_range):
                # while True:
                #     j += 1
                #     if self.flag_loop_var.get():
                # j %= num_im
                # if hasattr(module.worker, 'reload_params') and j == 0: module.worker.reload_params()
                # if j >= num_im: break
                path = paths[j]
                print('[%d/%d] %s' % (j, num_im, path['base']))
                rgbd = self.get_rgbd_from_path(path)

                if flag_multi_proc:
                    sub_thread = i % num_cpu_use

                    if sub_thread == 0:
                        num_sub_thread = len(self.sub_threads)
                        for t in self.sub_threads:
                            t.join()
                        print(f'{"+" * 30}{num_sub_thread} threads joined ...')
                        self.sub_threads = []

                    print(f'{"+" * 30}Threading on  CPU_{sub_thread} ...')
                    t = Thread(target=self.run_detect_dir_single,
                               args=(module, method_ind, path, rgbd, db_result_dir, sub_thread), daemon=True)
                    self.sub_threads.append(t)
                    t.start()
                else:
                    self.run_detect_dir_single(module, method_ind, path, rgbd, db_result_dir)

            if flag_multi_proc:
                num_sub_thread = len(self.sub_threads)
                for t in self.sub_threads:
                    t.join()
                print(f'{"+" * 30}{num_sub_thread} threads joined ...')
                self.sub_threads = []

            if flag_run_acc:
                module.worker.finalize_acc()
                if hasattr(module.worker, 'print_acc'):
                    self.highlight_label.configure(text=module.worker.print_acc())

    def popup_args(self, module, name='unnamed'):

        popup_root = tk.Toplevel(self.window)
        popup_root.geometry('{}x{}'.format(self.gui_size[0] // 2, self.gui_size[1]))
        popup_root.title(name)

        fr_button = tk.Frame(master=popup_root)
        fr_viewer = tk.Frame(master=popup_root)
        viewer_parent = ttk.Notebook(master=fr_viewer)

        def reload_args(module, tbx_args):
            module.args = CFG().from_string(GuiUtils().textbox2text(tbx_args), separate=True)
            self.get_module_args(module)
            if hasattr(module, 'worker'):
                module.worker.args = module.args
                if hasattr(module.worker, 'reload_params'): module.worker.reload_params()
            GuiUtils().update_textbox(tbx_args, module.args.to_string())

        def save_args(module, tbx_args):
            reload_args(module, tbx_args)
            args = CFG().from_string(GuiUtils().textbox2text(tbx_args), separate=True)
            args_ = args.logical_and(CFG(cfg_path=module.cfg_path))
            args_.write(module.cfg_path)

            if module.key_args is None: return
            for key_name in module.key_args:
                arg_val = eval(f'module.args.{key_name}')
                module.key_args_strings_dict[key_name].set(f'{arg_val}')

        def reset_args(module, tbx_args):
            module.args = CFG(cfg_path=module.cfg_path, separate=True)
            self.get_module_args(module)
            if hasattr(module, 'worker'): module.worker.args = module.args
            GuiUtils().update_textbox(tbx_args, module.args.to_string())

        # tbx_args_ = self.set_textbox_viewer(name='args_', viewer_parent=viewer_parent)
        # GuiUtils().update_textbox(tbx_args_, module.args.to_string())
        tbx_args = self.set_textbox_viewer(name='args', viewer_parent=viewer_parent)
        GuiUtils().update_textbox(tbx_args, module.args.to_string())

        tk.Button(master=fr_button, text='Reload configs', width=10,
                  command=partial(save_args, module, tbx_args)).grid(row=0, column=0)
        # tk.Button(master=fr_button, text='Save configs', width=10,
        #           command=partial(save_args, module, tbx_args, tbx_args)).grid(row=0, column=1)
        tk.Button(master=fr_button, text='Reset configs', width=10,
                  command=partial(reset_args, module, tbx_args)).grid(row=0, column=1)

        viewer_parent.pack(fill=tk.BOTH)
        fr_button.pack(fill=tk.BOTH, expand=False)
        fr_viewer.pack(fill=tk.BOTH, expand=False)

    def update_key_args(self, module):
        if module.key_args is None: return
        for key_name in module.key_args:
            section_name, option_name = key_name.split('.')
            try:
                section = module.args.__getattribute__(section_name)
            except:
                aa = 1
            val = module.key_args_strings_dict[key_name].get()
            try:
                val = eval(val)
            except:
                pass
            section.__setattr__(option_name, val)
            module.args.__setattr__(section_name, section)

        self.get_module_args(module)
        if hasattr(module, 'worker'):
            module.worker.args = module.args
            if hasattr(module.worker, 'reload_params'): module.worker.reload_params()

        module.args.write(module.cfg_path)

    def copy_args_sections_from_module(self, module_args_, section_names=None):
        if module_args_ is None:
            print('{} config is None o...'.format('+' * 10))
            return
        if section_names is None: section_names = module_args_.keys()
        for section_name in section_names:
            if not hasattr(module_args_, section_name):
                print('{} config is None or has no {} section ...'.format('+' * 10, section_name))
                continue
            default_section = self.args_.__getattribute__(section_name)
            module_section = module_args_.__getattribute__(section_name)
            default_options = default_section.keys()
            for option_name in module_section.keys():
                default_section.__setattr__(option_name, module_section.__getattribute__(option_name))

    def set_preprocess_params_from_module(self, module):
        self.copy_args_sections_from_module(module.args_, section_names=['sensor'])
        self.update_after_change_args_()

    def run_tcp_client(self, module, method_ind):
        tab_ind = self.get_current_tab_ind()
        if tab_ind >= self.num_viewer:
            tab_ind = 0
            self.viewer_parent.select(tab_ind)
        sensor = self.viewer_ind_to_sensor(tab_ind)

        rgbd = self.rgbds[tab_ind]
        # if rgbd is None:
        #     print('Empty input')
        #     return

        from ketisdk.base.base_tcp import ClientThread
        # address, port = self.str_server_address.get(), int(self.str_server_port.get())
        # self.args.conn.address, self.args.conn.port = address, port
        self.update_after_change_args_()
        self.reload_configs()
        # self.client = ClientThread(host=address, port=port)
        self.client = ClientThread(host=module.host, port=module.port)

        data_to_send = self.tcp_returned
        if module.input_var.get() == 'src':
            data_to_send = {
                'rgb': rgbd.rgb if rgbd is not None else None,
                'depth': rgbd.depth if rgbd is not None else None,
                'text': self.str_input_texts.get(),
                'workspace': self.args.sensor.crop_poly}
        self.tcp_returned = self.client.send_and_get_return(data_to_send)
        if self.tcp_returned is None:
            print('Return is Null')
            return
        if 'msg' in self.tcp_returned:
            print(self.tcp_returned['msg'])
        self.detected = {'rgbd': rgbd}
        try:
            self.detected['sensor_info'] = sensor.worker.info
        except:
            print('Cannot get sensor info')
        self.detected.update(self.tcp_returned)
        self.flag_show_detected = True
        self.show_detected(viewer_ind=tab_ind)

    def run_process_server(self):
        from ketisdk.base.base_tcp import ServerThread
        def run_prc(data):
            rgb = data['rgb'] if 'rgb' in data else None
            depth = data['depth'] if 'depth' in data else None

            rgbd = RGBD(rgb=rgb, depth=depth, depth_min=self.args.sensor.depth_min,
                        depth_max=self.args.sensor.depth_max)
            rgbd.set_workspace(self.args.sensor.crop_poly)
            self.rgbds[-1] = rgbd
            self.viewer_parent.select(self.num_viewer - 1)
            self.update_im_viewer()
            self.run_process_()
            if 'rgbd' in self.detected: del self.detected['rgbd']
            return self.detected

        class ProcessServer(ServerThread):
            def process_received_data(self):
                return run_prc(self.data)

        self.btn_server.config(text='Stop Server', command=self.stop_process_server)
        address, port = self.str_server_address.get(), int(self.str_server_port.get())
        self.args.conn.address, self.args.conn.port = address, port
        self.update_after_change_args_()
        self.reload_configs()
        self.server = ProcessServer(host=address, port=port)
        self.server_thread = Thread(target=self.server.listen, daemon=True)
        self.server_thread.start()

    def stop_process_server(self):
        self.btn_server.config(text='Run Server', command=self.run_process_server)
        # self.server.listen_thread.join()
        self.server.terminate()
        self.server_thread.join()

    def run_process(self):
        control_thread = Thread(target=self.run_process_, daemon=True)
        control_thread.start()

    def run_process_(self, show_fps=False):
        first_run = True
        while first_run or self.flag_loop_var.get():
            first_run = False
            timer = Timer()
            processes_inds = eval(self.str_processes.get())
            if not hasattr(processes_inds, '__iter__'): processes_inds = [processes_inds, ]
            self.just_executed = {'process': processes_inds}
            # self.just_executed_process_ind = process_ind
            modules = [self.modules[int(j)] for j in processes_inds]
            methods = [int(10 * j) - 10 * int(j) for j in processes_inds]
            num_module = len(modules)
            if num_module == 0:
                print('Process has no module')
                return
            for j, module in enumerate(modules):
                # if j == 0:
                #     module.input_var.set('Img')
                # else:
                #     module.input_var.set(f'{modules[j - 1].short_name}')
                show_detected = self.process_ask_next_var.get() or j == num_module - 1
                self.run_module(module=module, method_ind=methods[j], run_thread=False,
                                record_module=False,
                                show_detected=show_detected)
                timer.pin_time(module.name)
                if self.process_ask_next_var.get() and j < num_module - 1:
                    rep = msgbox.askquestion(title='Process', message='want to execute next?')
                    if rep == 'no': break
            print(timer.pin_times_str())

    def add_module(self, module_ind):
        module = self.modules[module_ind]
        # names = [mod.short_name for j, mod in enumerate(self.modules) if j != module_ind and mod.category == 'sensor']
        # names = ['Img', ] + names
        # names += [mod.short_name for j, mod in enumerate(self.modules) if j != module_ind and mod.category != 'sensor']
        # input_sources = names[module_ind:] + names[:module_ind]
        input_sources = ['src', 'ret'] if module_ind - self.num_sensor == 0 else ['ret', 'src']

        tk.Label(master=self.fr_task_cmd, text=f'=====MODULE {module_ind}=====').pack()
        module_frame = tk.Frame(master=self.fr_task_cmd,
                                highlightbackground='green', highlightcolor='green', highlightthickness=2)
        lb_frame = tk.Frame(master=module_frame)
        col = 0
        # if module.input_sources is not None:
        module.input_var = tk.StringVar()
        module.input_var.set(input_sources[0])
        tk.OptionMenu(lb_frame, module.input_var, *input_sources).grid(row=0, column=col)
        col += 1

        tk.Label(master=lb_frame, text=f'>>{module.short_name}').grid(row=0, column=col)
        lb_frame.pack()
        col += 1

        if module.category == 'tcp_client':
            tk.Button(master=lb_frame, text='Send', width=self.btn_w3,
                      command=partial(self.run_module, module, 0)).grid(row=0, column=col)
            module_frame.pack()
            print(f'Client {module.host}:{module.port} initialized .....')
            return

        module.method_ind_var = tk.StringVar()
        module.method_ind_var.set(module.default_method_ind)
        tk.OptionMenu(lb_frame, module.method_ind_var, *list(range(module.num_method))).grid(row=0, column=col)

        btn_width = self.btn_w5 if module.category == 'detector' else self.btn_w3

        btn_frame = tk.Frame(master=module_frame)
        btn_init = tk.Button(master=btn_frame, text='Int', width=btn_width,
                             command=partial(self.init_module, module, True))
        btn_init.grid(row=0, column=0)
        btn_init_ = tk.Button(master=btn_frame, text='Int_', width=btn_width,
                              command=partial(self.init_module, module, False))
        btn_init_.grid(row=0, column=1)
        btn_args_ = tk.Button(master=btn_frame, text='args_', width=btn_width,
                              command=partial(self.popup_args, module, '{} args_'.format(module.name)))
        btn_args_.grid(row=0, column=2)
        if module.category == 'detector':
            btn_prep = tk.Button(master=btn_frame, text='prep', width=btn_width,
                                 command=partial(self.set_preprocess_params_from_module, module))
            btn_prep.grid(row=0, column=3)
            btn_dir = tk.Button(master=btn_frame, text='dir', width=btn_width,
                                command=partial(self.open_dir_from_args, module))
            btn_dir.grid(row=0, column=4)

        num_method = module.num_method

        btn_runs = []
        # if num_method == 1:
        #     btn_run = tk.Button(master=btn_frame, text='Run', width=self.btn_w3, command=partial(self.run_module, module_ind, 0))
        #     btn_run.grid(row=0, column=2)
        #     btn_runs.append(btn_run)
        #     btn_frame.pack()
        # else:
        btn_frame.pack()
        btn_frame = tk.Frame(master=module_frame)
        for i in range(num_method):
            btn_text = 'Run'
            if num_method > 1: btn_text += str(i)
            row = i // 3
            col = i - 3 * row
            btn_run = tk.Button(master=btn_frame, text=btn_text, width=self.btn_w3,
                                command=partial(self.run_module, module, i))
            btn_run.grid(row=row + 1, column=col)
            btn_runs.append(btn_run)
        btn_frame.pack()

        if module.key_args is not None:
            key_frame = tk.Frame(master=module_frame)
            module.key_args_strings_dict = dict()
            if not hasattr(self, 'input_entries'): self.input_entries = []
            for j, key_name in enumerate(module.key_args):
                tk.Label(key_frame, text=f'{key_name}', width=self.btn_w1).grid(row=j, column=0)
                str_var = tk.StringVar()
                entr = tk.ttk.Entry(key_frame, textvariable=str_var, width=self.btn_w2,
                                    state='enabled' if self.flag_lock_key_press_var.get() else 'disabled')
                entr.grid(row=j, column=1)
                self.input_entries.append(entr)
                module.key_args_strings_dict[key_name] = str_var
            tk.Button(master=key_frame, text='Reload_args', width=self.btn_w1,
                      command=partial(self.update_key_args, module)).grid(row=j + 1, column=0)
            key_frame.pack()

        module_frame.pack()

        self.init_module(module, change_cfg_path=False, use_thread=module.key_args is None)
        if module.key_args is not None:
            for key_name in module.key_args:
                try:
                    arg_val = eval(f'module.args.{key_name}')
                    module.key_args_strings_dict[key_name].set(f'{arg_val}')
                except:
                    print(f'{key_name} not in args')

    def save_image(self):
        tab_ind = self.get_current_tab_ind()
        if tab_ind == self.num_viewer - 1:
            if self.args.sensor.save_disp:
                rgbd = self.rgbds[tab_ind]
                ArrayUtils().save_array_v3(rgbd.disp(mode=self.flag_imshow_modes[0])[:, :, ::-1],
                                           filepath=os.path.join('data/tmp', f'full_disp.png'))
                ArrayUtils().save_array_v3(rgbd.crop().disp(mode=self.flag_imshow_modes[0], show_ws=False)[..., ::-1],
                                           filepath=os.path.join('data/tmp', f'crop_disp.png'))
            return

        if tab_ind > self.num_viewer - 1: return
        rgbd = self.rgbds[tab_ind]
        sensor = self.viewer_ind_to_sensor(tab_ind)
        sensor_name = sensor.name
        subdir = ProcUtils().get_current_time_str('%m%d')
        subdir = os.path.join('data', sensor_name, subdir)
        filename = ProcUtils().get_current_time_str()
        if rgbd.hasRgb:
            ArrayUtils().save_array_v3(rgbd.bgr(), filepath=os.path.join(subdir, f'full_{filename}_rgb.png'))
            ArrayUtils().save_array_v3(rgbd.crop_rgb()[:, :, ::-1],
                                       filepath=os.path.join(subdir, f'crop_{filename}_rgb.png'))
        if rgbd.hasDepth:
            ArrayUtils().save_array_v3(rgbd.depth,
                                       filepath=os.path.join(subdir, f'full_{filename}_depth.png'))
            ArrayUtils().save_array_v3(rgbd.crop_depth(),
                                       filepath=os.path.join(subdir, f'crop_{filename}_depth.png'))
        if self.args.sensor.save_disp:
            ArrayUtils().save_array_v3(rgbd.disp(mode=self.flag_imshow_modes[0])[:, :, ::-1],
                                       filepath=os.path.join(subdir, f'full_{filename}_disp.png'))
            ArrayUtils().save_array_v3(rgbd.crop().disp(mode=self.flag_imshow_modes[0], show_ws=False)[..., ::-1],
                                       filepath=os.path.join(subdir, f'crop_{filename}_disp.png'))
        if rgbd.workspace is not None:
            ArrayUtils().save_array_v3(255 * rgbd.workspace.get_mask(max_shape=(rgbd.width, rgbd.height)),
                                       filepath=os.path.join(subdir, f'full_{filename}_mask.png'))

        # ProcUtils().open_dir(subdir)

    def sampling(self):
        sampling_dir = 'data/sampling'
        os.makedirs(sampling_dir, exist_ok=True)
        try:
            im_range = range(*eval(self.str_range.get()))
            print(f'Copying sampled images')
            for i in im_range:
                print(f'Image [{i}/{self.num_im}]')
                for name in self.paths[i]:
                    if name == 'base':
                        continue
                    p = self.paths[i][name]
                    print(f'\t{p}')
                    pdir, pname = os.path.split(p)
                    shutil.copy(p, os.path.join(sampling_dir, pname))
            print(f'Copying completed ....')

        except:
            print(f'Failed to sampling image from input range {self.str_range.get()}')

    def save_sequence(self):
        control_thread = Thread(target=self.save_sequence_, daemon=True)
        control_thread.start()

    def save_sequence_(self):
        num_im = int(input('Number of Images to be saved?\n'))
        waitTime = float(input('Wait time (Recommend: 5)s?\n'))
        print(f'Capture and save {num_im} to disk ...')
        if num_im <= 0: return
        for i in range(num_im):
            self.save_image()
            print(f'Image [{i + 1}/{num_im}] captured. Please suffer ...')
            time.sleep(waitTime)
        print('Sequence capturing finished')

    def viewer_ind_to_sensor(self, viewer_ind=0):
        for mod in self.modules:
            if not hasattr(mod, 'viewer_ind'): continue
            if mod.viewer_ind == viewer_ind: return mod

    def save_video(self):
        tab_ind = self.get_current_tab_ind()
        if tab_ind >= self.num_viewer - 1: return
        if self.rgbds[tab_ind] is None: return

        self.btn_save_video.configure(text='Stop save', command=partial(self.stop_save_video, tab_ind))
        self.acc_rgbd = []

        self.flag_save_video = True
        thread_control = Thread(target=self.acc_frame_, args=(tab_ind,), daemon=True)
        thread_control.start()

    def acc_frame_(self, sensor_ind):
        while self.flag_save_video:
            print('accumulating frame ...')
            self.acc_rgbd.append(self.rgbds[sensor_ind])
            time.sleep(1 / self.args.sensor.vid_fps)
            ProcUtils().clscr()

    def stop_save_video(self, tab_ind):
        if not hasattr(self, 'acc_rgbd'): return
        self.flag_save_video = False
        self.btn_save_video.configure(text='Save video', command=self.save_video)
        thread_control = Thread(target=self.stop_save_video_, args=(tab_ind,), daemon=True)
        thread_control.start()

    def stop_save_video_(self, viewer_ind):
        subdir = ProcUtils().get_current_time_str('%m%d')
        height, width = self.acc_rgbd[0].height, self.acc_rgbd[0].width
        writer = ProcUtils().init_vid_writer(size=(width, height),
                                             fold=os.path.join('data', self.viewer_ind_to_sensor(viewer_ind).name,
                                                               subdir),
                                             fps=self.args.sensor.vid_fps, isColor=True)
        for rgbd in self.acc_rgbd:
            writer.write(rgbd.bgr())

        self.acc_rgbd = []
        writer.release()
        print('video saved ...')

    def create_video(self):
        thread_control = Thread(target=self.create_video_, daemon=True)
        thread_control.start()

    def save_map_value(self):
        self.flag_stop_map_value = True
        self.flag_show_click_locs = False
        self.flag_measure = False
        save_dir = 'data/json'
        if not os.path.exists(save_dir): os.makedirs(save_dir)
        with open(os.path.join(save_dir, 'map_values.json'), 'w') as outfile:
            json.dump(self.valuesToMap, outfile)
        self.click_locs = []
        self.btn_map_value.configure(text='Map value', command=self.map_value)

    def map_value(self):
        Thread(target=self.map_value_, daemon=True).start()

    def map_value_(self):
        self.click_locs = []
        self.flag_show_click_locs = True
        self.flag_measure = True
        self.btn_map_value.configure(text='Save map', command=self.save_map_value)
        # self.highlight_label.configure(text='Click to select\nESC to exit')
        last_num_click = 0
        self.valuesToMap = {'im_vals': [], 'input_vals': []}
        self.flag_stop_map_value = False
        while not self.flag_stop_map_value:
            time.sleep(1)
            num_click = len(self.click_locs)
            if num_click > last_num_click:
                rgbd = self.rgbds[self.get_current_tab_ind()]
                im_val = self.click_locs[-1]
                if rgbd.hasDepth: im_val += (int(rgbd.depth[self.click_locs[-1][::-1]]),)
                self.valuesToMap['im_vals'].append(im_val)
                self.valuesToMap['input_vals'].append(eval(input('Enter value to be mapped: ')))
                last_num_click = num_click

    def create_video_(self):
        if not hasattr(self, 'paths'): return
        num_im = len(self.paths)
        if num_im <= 1: return

        rgbs = [cv2.imread(path['im']) for path in self.paths]
        h, w = rgbs[0].shape[:2]
        writer = ProcUtils().init_vid_writer(size=(w, h), fold=self.data_root,
                                             fps=self.args.sensor.vid_fps, isColor=True)
        for rgb in rgbs:
            writer.write(rgb)

        rgbs = []
        writer.release()
        print('video saved ...')


class GuiModule():
    def __init__(self, module, type='detector', name='unnamed', category='detector', args=None,
                 cfg_path=None, default_cfg_path=None, num_method=1, default_method_ind=0, short_name=None,
                 run_thread=True,
                 run_after_init_method_ind=None, key_args=None,
                 input_sources=None, **kwargs):
        self.module = module
        self.type = type
        self.name = name
        self.category = category
        self.args = args
        self.cfg_path = cfg_path
        self.default_cfg_path = default_cfg_path
        self.num_method = num_method
        self.default_method_ind = default_method_ind
        self.short_name = short_name
        self.run_thread = run_thread
        self.run_after_init_method_ind = run_after_init_method_ind
        self.input_sources = input_sources
        self.key_args = key_args
        if self.short_name is None: self.short_name = self.name[:min(len(self.name), 10)]
        for key in kwargs:
            if not hasattr(self, key):
                setattr(self, key, kwargs[key])
            else:
                self.__setattr__(key, kwargs[key])


class GuiProcess():
    def __init__(self, gui_modules, module_inds=None, method_inds=None, input_sources=None, name='unnamed_process',
                 im_in_ret=True, ask_next=True):
        self.gui_modules = gui_modules
        self.method_inds = method_inds if method_inds is not None else [0, ] * len(gui_modules)
        self.module_inds = module_inds if module_inds is not None else list(range(len(gui_modules)))
        self.input_sources = input_sources
        self.name = name
        self.ask_next = ask_next
        self.im_in_ret = im_in_ret


if __name__ == '__main__':
    GUI()
