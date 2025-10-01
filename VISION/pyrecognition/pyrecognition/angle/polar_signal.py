import cv2
import numpy as np
import copy
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt

def update_threshold_h_max(val):
    threshold = cv2.getTrackbarPos('Threshold_h_max', 'Threshold Adjustment')
    return threshold

def update_threshold_h_min(val):
    threshold = cv2.getTrackbarPos('Threshold_h_min', 'Threshold Adjustment')
    return threshold


def update_threshold_s_max(val):
    threshold = cv2.getTrackbarPos('Threshold_s_max', 'Threshold Adjustment')
    return threshold


def update_threshold_s_min(val):
    threshold = cv2.getTrackbarPos('Threshold_s_min', 'Threshold Adjustment')
    return threshold


def update_threshold_v_max(val):
    threshold = cv2.getTrackbarPos('Threshold_v_max', 'Threshold Adjustment')
    return threshold

def update_threshold_v_min(val):
    threshold = cv2.getTrackbarPos('Threshold_v_min', 'Threshold Adjustment')
    return threshold
def update_kernelsize_1(val):
    threshold = cv2.getTrackbarPos('Threshold_kernelsize_1', 'Threshold Adjustment')
    return threshold


def median_filter(signal, window_size):
    """
    Apply a 1D median filter to a signal.

    Parameters:
        signal (array-like): The input 1D signal.
        window_size (int): The size of the sliding window (must be odd).

    Returns:
        np.ndarray: The filtered signal.
    """
    if window_size % 2 == 0:
        raise ValueError("Window size must be odd.")

    half_window = window_size // 2
    padded_signal = np.pad(signal, (half_window, half_window), mode='edge')
    filtered_signal = np.zeros_like(signal)

    for i in range(len(signal)):
        window = padded_signal[i:i + window_size]
        filtered_signal[i] = np.median(window)

    return filtered_signal

def update_kernelsize_2(val):
    threshold = cv2.getTrackbarPos('Threshold_kernelsize_2', 'Threshold Adjustment')
    return threshold

def mouse_event(event, x, y, flags, param):
    global gx, gy
    global display_img
    global roi
    if event == cv2.EVENT_FLAG_LBUTTON:
        Xc = x
        Yc = y
        print(Xc, Yc)
        roi.append([Xc,Yc])
    if roi.__len__()==1:
        display_img = copy.deepcopy(origin_rgb)
        cv2.rectangle(display_img,roi[0],(x,y),(0,255,0),2)

def detect_circle(orignal,param1=20, param2=10, minRadius=95, maxRadius=120):
    print(minRadius, maxRadius)
    gray = cv2.cvtColor(orignal, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius
                               )
    if type(circles)==type(None):
        return False,None
    return True,circles[0][0]
def median_filter_1d(signal, kernel_size=3):
    assert kernel_size % 2 == 1, "kernel_size must be odd"
    pad_size = kernel_size // 2
    padded = np.pad(signal, (pad_size, pad_size), mode='edge')
    filtered = np.array([
        np.median(padded[i:i+kernel_size])
        for i in range(len(signal))
    ])
    return filtered
def calculate_dif_angle(rgb,
                        ref_angle=-86.94130242590418,
                        # roi=[417, 5, 1492, 1074],
                        roi=[330, 12, 1393, 1098],
                        **kwargs):
    print("kwargs:  ",kwargs)
    print(roi)
    rgb=copy.deepcopy(rgb[roi[1]:roi[3],roi[0]:roi[2],:])

    ret, reference_r, reference_angle, center_x, center_y, cur_contour, ret_img = detect_polar(rgb,**kwargs)
    
    # "
    if ret:
        ref_angle_line = [-86.94130242590418, -55.086531684653785, -22.272727273]
        for angle in ref_angle_line:
            X = reference_r * np.cos(np.deg2rad(angle))
            Y = reference_r * np.sin(np.deg2rad(angle))

            X = X + center_x
            Y = Y + center_y
            cv2.line(ret_img, (int(center_x), int(center_y)), (int(X), int(Y)), (255, 255, 0), 1)
        print(f"ref:{ref_angle} {reference_angle}")
        print("differ:", reference_angle - (ref_angle))
        X = reference_r * np.cos(np.deg2rad(reference_angle))
        Y = reference_r * np.sin(np.deg2rad(reference_angle))

        X = X + center_x
        Y = Y + center_y
        differ_angle=reference_angle - (ref_angle)
        return True,X,Y,differ_angle,reference_angle,ret_img
    else:
        return False,None,None,None,None,None,None
def cal_threshold(origin_rgb):
    hsv = cv2.cvtColor(origin_rgb, cv2.COLOR_BGR2HSV)
    v_image = cv2.medianBlur(hsv[:, :, -1], 11)
    # 1. 히스토그램 계산
    counts, bin_edges = np.histogram(v_image.flatten(), bins=255)

    counts = median_filter(counts, 5)
    # 2. 봉우리 찾기
    peaks, _ = find_peaks(counts)

    # 3. 봉우리 위치(bin 중심 계산)
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
    peak_positions = bin_centers[peaks]
    peak_heights = counts[peaks]
    thershold = peak_positions[np.argsort(peak_heights)[-1]] + 20
    return thershold
def validate_angle(sort_angle, pulse_index):
    nCr = [[i,i+1 ] for i in list(range(pulse_index.shape[0]-1))]
    reduce_ret = []
    correct_index=[]
    for index in nCr:
        diff = abs(sort_angle[index[0]] - sort_angle[index[1]])
        mult = np.round(diff / (360 / 11))
        reduce = diff - (360 / 11) * mult
        if abs(diff) < 25:
            reduce_ret.append([reduce] + list(index)+[diff])
        else:
            correct_index.append(index[1])
    if reduce_ret.__len__() > 0:
        # print(reduce_ret)

        thre = 1#reduce_ret.__len__()
        poplist = np.zeros(pulse_index.shape[0])
        for pairindex in reduce_ret:
            poplist[pairindex[1]] += 1
            # poplist[pairindex[2]] += 1
        if (poplist == thre).any():
            pop_index = np.where(poplist == thre)[0]
            new_list_index = np.array(list(set(list(range(pulse_index.shape[0]))) - set(list(pop_index))))
            new_pulse_index=pulse_index[new_list_index]
        else:
            print("error")
            return pulse_index
    else:
        return pulse_index
    return new_pulse_index
def detect_polar(rgb,max_h_threshold=120,max_s_threshold=100,max_v_threshold=105,blur_size=11
                 ,param1=30, param2=40, minRadius=140, maxRadius=165,
                     angle_range=[-120,90],
                     single_med_filtersize=11,**kwargs):
    print("max_s_threshold",max_s_threshold)
    print("param1",param1)
    print("param2",param2)        
    print("blur_size",blur_size,type(blur_size))
    rgb = cv2.medianBlur(rgb, blur_size)
    print("detect_circle")
    ret_circle,circle_point = detect_circle(rgb,param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
    
    if ret_circle==False:
        print("ret_circle==False")
        return False, None, None, None, None, None, None, None
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)


    img_h, img_s, img_v = cv2.split(hsv)
    img_v = cv2.medianBlur(img_v, blur_size)
    img_v = cv2.equalizeHist(img_v)
    min_h_threshold =0
    img_h_threshold = (min_h_threshold < img_h) * (img_h < max_h_threshold)

    min_s_threshold = 0
    img_s_threshold = (min_s_threshold < img_s) * (img_s < max_s_threshold)

    min_v_threshold = 0

    v_threshold=max_v_threshold
    img_v_threshold = (min_v_threshold < img_v) * (img_v < v_threshold)

    data = ((img_h_threshold * img_s_threshold * img_v_threshold) * 255).astype(np.uint8)

    kernel_size = 4
    kernel_size = kernel_size * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    kernel_size = (kernel_size + 1) * 2 + 1
    kernel_post = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    kernel_size = 1
    kernel_size = kernel_size * 2 + 1
    kernel_2 = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    closing = cv2.morphologyEx(data, cv2.MORPH_CLOSE, kernel)
    opening_1 = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel_2)
    opening_2 = cv2.morphologyEx(opening_1, cv2.MORPH_OPEN, kernel_2)
    opening = cv2.morphologyEx(opening_2, cv2.MORPH_OPEN, kernel_2)
    ret = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    for i in range(0, 5):
        ret = cv2.morphologyEx(ret, cv2.MORPH_CLOSE, kernel_post)
    cv2.circle(ret, circle_point[0:2].astype(int), 330, (255, 255, 255), -1)

    contours, hierarchy = cv2.findContours(ret, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours.__len__() == 0:
        ret_img = copy.deepcopy(rgb)
        cv2.putText(
            ret_img,
            text="error contour",
            org=(50, 100),  # 텍스트 시작 위치 (x, y)
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,  # 글꼴 종류
            fontScale=1.5,  # 글자 크기
            color=(0, 0, 255),  # 글자 색상 (B, G, R) → 빨간색
            thickness=2,  # 선 두께
            lineType=cv2.LINE_AA  # 앤티앨리어싱
        )
        # cv2.imshow("contours", ret_img)
        # cv2.waitKey(10)
        print("done")
        return  False, None, None, None, None, None, None, None
    index = np.argsort([con.shape[0] for con in contours])[-1]

    mask = np.zeros_like(rgb)
    cv2.fillPoly(mask, [contours[index]], [255, 255, 255])

    center_x=circle_point[0]
    center_y=circle_point[1]
    select_contour=contours[index]
    orin_x = select_contour[:, 0, 0]
    orin_y = select_contour[:, 0, 1]
    x = orin_x[orin_y < center_y] - center_x
    y = orin_y[orin_y < center_y] - center_y
    r = np.sqrt(x ** 2 + y ** 2)  # Radius
    theta = np.arctan2(y, x)  # Angle in radians
    angle = np.rad2deg(theta)
    window_size = 11
    # filtered_signal = median_filter(r, window_size)
    # plt.close()
    # fig = plt.figure()
    list_angle = copy.deepcopy(angle[np.argsort(angle)])
    list_r = copy.deepcopy(r[np.argsort(angle)])


    min_angle=angle_range[0]
    max_angle=angle_range[1]
    sort_angle=list_angle[(min_angle< list_angle) & (list_angle < max_angle)]
    sort_r=list_r[(min_angle < list_angle) & (list_angle < max_angle)]
    # plt.plot(list_angle, list_r)
    # plt.plot(sort_angle, sort_r)

    med_sort_r=median_filter_1d(sort_r,single_med_filtersize)
    signal = ((med_sort_r > np.mean(med_sort_r)) * 1)

    cand_pulse_index=np.where((signal[1::] - signal[0:-1]) == 1)[0]
    pulse_index=validate_angle(sort_angle[cand_pulse_index], cand_pulse_index)
    cur_contour=[contours[index]]
    ret_img=copy.deepcopy(rgb)


    cv2.drawContours(ret_img, cur_contour, 0, (0, 255, 0), 3)
    for _index in cand_pulse_index:
        X = sort_r[_index] * np.cos(np.deg2rad( sort_angle[_index]))
        Y =  sort_r[_index] * np.sin(np.deg2rad( sort_angle[_index]))

        X = X + center_x
        Y = Y + center_y

        cv2.line(ret_img, (int(center_x), int(center_y)), (int(X), int(Y)), (0, 255, 0), 3)
    for _index in pulse_index:
        X = sort_r[_index] * np.cos(np.deg2rad( sort_angle[_index]))
        Y =  sort_r[_index] * np.sin(np.deg2rad( sort_angle[_index]))

        X = X + center_x
        Y = Y + center_y

        cv2.line(ret_img, (int(center_x), int(center_y)), (int(X), int(Y)), (0, 0, 255), 2)
    start_index=pulse_index[sort_angle[pulse_index] > min_angle][0]
    reference_r = sort_r[start_index]
    reference_angle = sort_angle[start_index]

    for i in range(0, 11):
        angle_data = 360 / 11 * i + sort_angle[start_index]
    print("final")
    return True, reference_r,reference_angle,center_x,center_y,[contours[index]],ret_img

if __name__ == '__main__':
    global roi
    global display_img
    cv2.namedWindow('result')
    cv2.setMouseCallback('result', mouse_event)
    roi=[]
    imgpath="/media/keti/5aa8b858-eaa4-4b47-ade7-e2704a1b97da/DB/Talos/"
    filename="1_Gain_200_exposure_50000_focus_630.png"
    origin_rgb=cv2.imread(imgpath+filename)
    display_img = copy.deepcopy(origin_rgb)
    while roi.__len__()<2:
        cv2.imshow("result", display_img)
        cv2.waitKey(30)

    print(roi)
    rgb=copy.deepcopy(origin_rgb[roi[0][1]:roi[1][1],roi[0][0]:roi[1][0],:])
    
    initial_threshold = 127
    window_name='Threshold Adjustment'
    cv2.namedWindow(window_name)
    cv2.createTrackbar('Threshold_h_min', window_name, 0, 255, update_threshold_h_min)
    cv2.createTrackbar('Threshold_h_max', window_name, 0, 255, update_threshold_h_max)

    cv2.createTrackbar('Threshold_s_min', window_name, 0, 255, update_threshold_s_min)
    cv2.createTrackbar('Threshold_s_max', window_name, 0, 255, update_threshold_s_max)
    
    cv2.createTrackbar('Threshold_v_min', window_name, 0, 255, update_threshold_v_min)
    cv2.createTrackbar('Threshold_v_max', window_name, 0, 255, update_threshold_v_max)

    cv2.createTrackbar('Threshold_kernelsize_1', window_name, 3, 10, update_kernelsize_1)
    cv2.createTrackbar('Threshold_kernelsize_2', window_name, 3, 10, update_kernelsize_2)

    cv2.setTrackbarPos("Threshold_h_min",window_name,0)
    cv2.setTrackbarPos("Threshold_s_min",window_name,0)
    cv2.setTrackbarPos("Threshold_v_min",window_name,0)

    cv2.setTrackbarPos("Threshold_h_max",window_name,120)
    cv2.setTrackbarPos("Threshold_s_max",window_name,100)
    cv2.setTrackbarPos("Threshold_v_max",window_name,110)

    cv2.setTrackbarPos("Threshold_kernelsize_1", window_name, 4)
    cv2.setTrackbarPos("Threshold_kernelsize_2", window_name, 1)
    while 1:
        # 초기 Threshold 값initial_threshold
        # 초기 상태로 표시
        max_h_threshold=update_threshold_h_max(initial_threshold)
        max_s_threshold=update_threshold_s_max(initial_threshold)
        max_v_threshold=update_threshold_v_max(initial_threshold)

        ret,reference_r, reference_angle, center_x, center_y,cur_contour,ret_img=detect_polar(rgb,max_h_threshold,max_s_threshold,max_v_threshold)
        if ret==False:
            continue
        X=reference_r*np.cos(np.deg2rad(reference_angle))
        Y=reference_r*np.sin(np.deg2rad(reference_angle))

        X = X + center_x
        Y = Y + center_y

        # cv2.line(ret_img,(int(center_x),int(center_y)),(int(X),int(Y)),(0,0,255),2)
        cv2.drawContours(ret_img, cur_contour, 0, (0, 255, 0), 3)
        cv2.imshow(window_name,ret_img)
        cv2.imshow("plot_image",plot_image)
        cv2.waitKey(10)
        print("done")
