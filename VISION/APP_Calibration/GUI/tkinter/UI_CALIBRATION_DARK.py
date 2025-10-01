import tkinter as tk
from tkinter import ttk

def gui_start():
    global val, w, root
    root = tk.Tk()
    uiApp = UIApp(root, "Calibration")
    root.mainloop()

'''
class UIButtons(tk.Frame):
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)

        button_frame = ttk.Frame(self)
        button_frame.grid(row=1, column=1, padx=10, sticky="nsew", columnspan=3)
        button_frame.columnconfigure(index=0, weight=1)

        self.button_start = ttk.Button(button_frame, text="Handeye Calibration")
        self.button_start.grid(row=0, column=0, padx=5, pady=10, sticky="ew")
        self.button_detection = ttk.Button(button_frame, text="Load Calibration Data")
        self.button_detection.grid(row=2, column=0, padx=5, pady=30, sticky="ew")

        # label_roi = ttk.Label(button_frame)
        # label_roi.grid(row=2, column=0, padx=5, pady=10, sticky="ew")
        # label_roi.configure(text="ROI Setting")
        #
        # self.button_setroi = ttk.Button(button_frame, text="Set ROI")
        # self.button_setroi.grid(row=3, column=0, padx=5, pady=10, sticky="ew")
        #
        # self.button_saveroi = ttk.Button(button_frame, text="Save ROI")
        # self.button_saveroi.grid(row=4, column=0, padx=5, pady=10, sticky="ew")
        #
        # label_roi = ttk.Label(button_frame)
        # label_roi.grid(row=5, column=0, padx=5, pady=10, sticky="ew")
        # label_roi.configure(text="Background Setting")
        #
        # self.button_savebackground = ttk.Button(button_frame, text="Save Background")
        # self.button_savebackground.grid(row=6, column=0, padx=5, pady=10, sticky="ew")

'''

class UIApp:
    def __init__(self, root, app_name="app_name"):
        self._root = root

        # Import the tcl file
        theme_path = "../../../GUI/tkinter/Forest-ttk-theme-master/forest-dark.tcl"
        self._root.tk.call('source', theme_path)

        # Set the theme with the theme_use method
        ttk.Style().theme_use('forest-dark')

        # Set title
        self._root.title(app_name)

        # Set Geometry and Grid
        self._root.geometry("1800x1000")
        self._root.rowconfigure(index=0, weight=1)
        self._root.rowconfigure(index=1, weight=1)
        self._root.rowconfigure(index=2, weight=5)
        self._root.rowconfigure(index=3, weight=1)

        self._root.columnconfigure(index=0, weight=9)
        self._root.columnconfigure(index=1, weight=2)

        ## layout
        ## buttons

        self.set_button_layout()
        #self.buttonUI = UIButtons(self._root)
        #self.buttonUI.pack(side="top", fill="both", expand=True)

        ## images
        self.set_image_layout()

        ## setting param
        self.set_setting_param_layout()

        ## log view
        self.set_log_layout()




    def set_button_layout(self):
        ## set button frame
        button_frame = ttk.Frame(self._root)
        button_frame.grid(row=1, column=1, padx=10, sticky="nsew", columnspan=3)
        button_frame.columnconfigure(index=0, weight=1)

        ### ip label
        self.label_ip = ttk.Label(button_frame)
        self.label_ip.grid(row=0, column=0, padx=0, pady=0, sticky="ew")
        self.label_ip.configure(text="Handeye Calibration")

        self.button_calibration_depth = ttk.Button(button_frame, text="Calibration_depth")
        self.button_calibration_depth.grid(row=1, column=0, pady=10, sticky="nsew")#, padx=0, pady=0, sticky="ew")

        self.button_calibration_pnp = ttk.Button(button_frame, text="Calibration_pnp")
        self.button_calibration_pnp.grid(row=2, column=0, pady=10, sticky="nsew")#sticky="ew")

        self.button_Load_Calibration = ttk.Button(button_frame, text="Load Calibration Data")
        self.button_Load_Calibration.grid(row=3, column=0, pady=10, sticky="nsew")#sticky="ew")

        self.button_Save_image = ttk.Button(button_frame, text="Save image")
        self.button_Save_image.grid(row=4, column=0, pady=10, sticky="nsew")#sticky="ew")

        self.button_Test_image = ttk.Button(button_frame, text="Test_image")
        self.button_Test_image.grid(row=5, column=0, pady=10, sticky="nsew")#sticky="ew")

        '''
        label_roi = ttk.Label(button_frame)
        label_roi.grid(row=2, column=0, padx=5, pady=10, sticky="ew")
        label_roi.configure(text="ROI Setting")

        self.button_setroi = ttk.Button(button_frame, text="Set ROI")
        self.button_setroi.grid(row=3, column=0, padx=5, pady=5, sticky="ew")

        self.button_saveroi = ttk.Button(button_frame, text="Save ROI")
        self.button_saveroi.grid(row=4, column=0, padx=5, pady=5, sticky="ew")

        label_roi = ttk.Label(button_frame)
        label_roi.grid(row=5, column=0, padx=5, pady=10, sticky="ew")
        label_roi.configure(text="Background Setting")

        self.button_savebackground = ttk.Button(button_frame, text="Save Background")
        self.button_savebackground.grid(row=6, column=0, padx=5, pady=5, sticky="ew")

        label_roi = ttk.Label(button_frame)
        label_roi.grid(row=7, column=0, padx=5, pady=10, sticky="ew")
        label_roi.configure(text="Config Setting")
        self.button_reloadconfig = ttk.Button(button_frame, text="Reload Configs")
        self.button_reloadconfig.grid(row=8, column=0, padx=5, pady=5, sticky="ew")
        '''
    def set_image_layout(self):
        ### set image layout
        img_paned = ttk.PanedWindow(self._root)
        img_paned.grid(row=1, column=0, pady=(10, 5), sticky="nsew", rowspan=2)
        img_Frame = ttk.Frame(img_paned)
        img_paned.add(img_Frame, weight=1)
        img_notebook = ttk.Notebook(img_Frame)

        ### tab 1
        self.img_tab1 = ttk.Frame(img_notebook)
        self.img_tab1.columnconfigure(index=0, weight=1)
        img_notebook.add(self.img_tab1, text="color")

        self.img_tab1_label = ttk.Label(self.img_tab1, text="Empty Image 1", justify="center")
        self.img_tab1_label.grid(row=0, column=0, pady=10)

        ## tab 2
        self.img_tab2 = ttk.Frame(img_notebook)
        self.img_tab2.columnconfigure(index=0, weight=1)
        img_notebook.add(self.img_tab2, text="depth")

        self.img_tab2_label = ttk.Label(self.img_tab2, text="Empty Image 2", justify="center")
        self.img_tab2_label.grid(row=0, column=0, pady=10)

        img_notebook.pack(expand=True, fill="both", padx=10, pady=0)

        ## tab 3
        self.img_tab3 = ttk.Frame(img_notebook)
        self.img_tab3.columnconfigure(index=0, weight=1)
        img_notebook.add(self.img_tab3, text="Calibration")

        self.img_tab3_label = ttk.Label(self.img_tab3, text="Empty Image 3", justify="center")
        self.img_tab3_label.grid(row=0, column=0, pady=10)



    def set_setting_param_layout(self):
        # Setting Notebook
        setting_paned = ttk.PanedWindow(self._root)
        setting_paned.grid(row=2, column=1, pady=(10, 5), sticky="nsew", rowspan=2)
        setting_Frame = ttk.Frame(setting_paned)
        setting_paned.add(setting_Frame, weight=1)
        setting_notebook = ttk.Notebook(setting_Frame)

        # tab 1
        self.setting_tab1 = ttk.Frame(setting_notebook)
        self.setting_tab1.columnconfigure(index=0, weight=1)
        self.setting_tab1.columnconfigure(index=1, weight=1)
        self.setting_tab1.rowconfigure(index=0, weight=1)

        self.setting_tab2 = ttk.Frame(setting_notebook)
        self.setting_tab2.columnconfigure(index=0, weight=1)
        self.setting_tab2.columnconfigure(index=1, weight=1)

        setting_notebook.add(self.setting_tab1, text="setting1")
        setting_notebook.add(self.setting_tab2, text="setting2")
        setting_notebook.pack(expand=True, fill="both", padx=10, pady=0)

        ## normal
        label_normal = ttk.Label(self.setting_tab1, text="normal depth threshold")
        label_normal.grid(row=0, column=0, padx=5, pady=10, sticky="new")
        #label_normal.configure(text="normal depth threshold", anchor = 'center')

        self.normal_depth_threshold = ttk.Entry(self.setting_tab1)
        self.normal_depth_threshold.insert(0, "780")
        self.normal_depth_threshold.grid(row=0, column=1, padx=5, pady=(0, 10), sticky="new")



        #self.normal_depth_threshold.grid(row=0, column=1, padx=5, pady=(0, 10), sticky="ew")
    def set_log_layout(self):
        log_Frame = ttk.Frame(self._root)
        log_Frame.grid(row=3, column=0, padx=10, sticky="nsew")
        log_Scroll = ttk.Scrollbar(log_Frame, orient="vertical")
        log_Scroll.pack(side="right", fill="y")

        self.listbox = tk.Listbox(log_Frame, selectmode="extended")
        self.listbox.pack(expand=True, fill="both")
        log_Scroll.config(command=self.listbox.yview)


if __name__ == '__main__':
    gui_start()