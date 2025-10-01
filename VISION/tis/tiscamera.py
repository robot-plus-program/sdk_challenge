import copy
import sys
import time
import cv2
import numpy as np
import os
import threading

# TIS SDK 경로 추가
sys.path.append(__file__.split("tiscamera.py")[0]+"python-common")
import TIS  # The Imaging Source 카메라 라이브러리

# TIS 카메라 클래스 (스레드 기반 프레임 획득)
class TISCamera(threading.Thread):
    def run(self):
        # 지속적으로 이미지를 스냅하여 origin_image에 저장
        while True:
            if self.Tis.Snap_image(1 / (self.fps + 10)):
                self.origin_image = self.Tis.Get_image()
                time.sleep(0.01)

    # 포커스와 노출값 수동 설정
    def setting(self, focus, exposure):
        self.Set_Focus(focus)
        self.Set_ExposureAutoReference(exposure)

    # 카메라 장치 설정
    def set_device(self, device_serial, sensor_size=(1920, 1080), fps=30, showvideo=False):
        self.device_serial = device_serial
        self._sensor_width, self._sensor_height = sensor_size
        self.Tis = TIS.TIS()
        self.img_width = 1280
        self.out_size = (self.img_width, int(5360 * self.img_width / 7716))
        self.resize_ratio = 1 / 4
        self.focus = 1023
        self.origin_image = np.array([])
        self.fps = fps

        fps_2 = f"{fps}/1"
        print("Open TIS Cam")
        self.Tis.openDevice(device_serial, self._sensor_width, self._sensor_height, fps_2, TIS.SinkFormats.BGRA, showvideo)
        print("Start TIS Pipeline")
        self.Tis.Start_pipeline()

    # 종료 시 파이프라인 중지
    def __del__(self):
        self.Tis.Stop_pipeline()

    # 단발성 이미지 획득
    def get_image_origin(self, out_size=(1280, 720)):
        if self.Tis.Snap_image(0.33):
            return self.Tis.Get_image()        
    # 지속 프레임 기반 이미지 반환 (origin_image에서 가져옴)
    def get_image(self, resize_ratio=None):

        pre_exp=self.Tis.Get_Property("ExposureTime")
        count=0
        while 1:
            cur_exp = self.Tis.Get_Property("ExposureTime")
            print(cur_exp,end=" ")
            time.sleep(0.1)
            if cur_exp==pre_exp:
                count+=1
            pre_exp=copy.deepcopy(cur_exp)
            if count>20:
                break
        print("done")
        if resize_ratio is None:
            resize_ratio = self.resize_ratio
        image = copy.deepcopy(self.origin_image)
        while image.size == 0:
            image = copy.deepcopy(self.origin_image)
        resize_img = cv2.resize(image, dsize=(0, 0), fx=resize_ratio, fy=resize_ratio)
        out_img = cv2.cvtColor(resize_img, cv2.COLOR_BGRA2BGR)
        origin_image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        return out_img, origin_image

    # 수동 초점 조정
    def increase_focus(self, val=10):
        self.focus = self.Tis.Get_Property("Focus")
        self.focus += int(val)
        self.Tis.Set_Property("Focus", self.focus)
        print("focus: ", self.focus)

    def decrease_focus(self, val=10):
        self.focus = self.Tis.Get_Property("Focus")
        self.focus -= int(val)
        self.Tis.Set_Property("Focus", self.focus)
        print("focus: ", self.focus)

    # 자동 초점 수행
    def auto_focus(self):
        self.focus = self.Tis.Set_Property("FocusAuto", "Once")

    # 자동 노출 기준값 설정
    def Set_ExposureAutoReference(self, val=128):
        self.Tis.Set_Property("ExposureAuto", "Off")
        self.Tis.Set_Property("ExposureAutoReference", val)

    # 수동 포커스 설정
    def Set_Focus(self, val=400):
        self.Tis.Set_Property("FocusAuto", "Off")
        self.Tis.Set_Property("Focus", val)


# ----------- 실행부 -----------
if __name__ == '__main__':
    setting_1 = {
        "device_serial": "20120594",
        "sensor_size": (1920, 1080),
        "auto_focus": "Off",
        "focus": 660,
        "auto_exposure": "Off",
        "exposure": 50000,
        "GainAuto": "Off",
        "Gain": 200,
        "fps": 30
    }

    tis_setting = setting_1
    tisCam = TISCamera()
    tisCam.set_device(tis_setting['device_serial'], tis_setting['sensor_size'], tis_setting['fps'], showvideo=False)

    # 저장 폴더 설정
    save_folder = "/media/keti/5aa8b858-eaa4-4b47-ade7-e2704a1b97da/DB/Talos/"
    tisCam.start()  # 이미지 획득 스레드 시작

    # 초기 카메라 파라미터 수동 설정
    Gain = tis_setting["Gain"]
    exposure = tis_setting["exposure"]
    focus = tis_setting["focus"]

    tisCam.Tis.Set_Property("GainAuto", "Off")
    tisCam.Tis.Set_Property("Gain", Gain)
    tisCam.Tis.Set_Property("ExposureAuto", "Off")
    tisCam.Tis.Set_Property("ExposureTime", exposure)
    tisCam.Tis.Set_Property("FocusAuto", "Off")
    tisCam.Tis.Set_Property("Focus", focus)

    rot_t = 0
    count = 0

    while True:
        tis_rgb, orignal = tisCam.get_image()

        if tis_rgb is not None:
            h, w, c = orignal.shape
            cv2.line(tis_rgb, (0, int(h / 2)), (w, int(h / 2)), (0, 255, 0), 3)
            cv2.line(tis_rgb, (int(w / 2), 0), (int(w / 2), h), (0, 255, 0), 3)

            gray = cv2.cvtColor(orignal, cv2.COLOR_BGR2GRAY)
            draw_rgb = copy.deepcopy(orignal)

            # 허프 변환을 이용한 원 검출
            rows = orignal.shape[0]
            circles = cv2.HoughCircles(
                gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                param1=20, param2=10,
                minRadius=95, maxRadius=120
            )
            if circles is not None:
                circle_point = circles[0][0]
                cv2.circle(draw_rgb, circle_point[0:2].astype(int), int(circle_point[-1]), (0, 255, 0), 3)

            # 이미지 출력
            cv2.imshow('TIS Cam', draw_rgb)
            key = cv2.waitKey(1)

        # 종료 키: ESC
        if key == 27:
            cv2.destroyAllWindows()
            break

        # 이미지 저장: c 키
        if key == ord("c"):
            count += 1
            filename = f'{count}_Gain_{Gain}_exposure_{int(exposure)}_focus_{focus}.png'
            cv2.imwrite(os.path.join(save_folder, filename), orignal)
            rot_t += 1

        # 초점 조절 키: f (증가), r (감소)
        if key == ord("f"):
            tisCam.increase_focus()
        elif key == ord("r"):
            tisCam.decrease_focus()

        # 자동 초점 수행: a 키
        if key == ord("a"):
            tisCam.auto_focus()

        # 체스보드 패턴 검출 시각화: l 키
        if key == ord("l"):
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            w_size, h_size = 6, 9
            objp = np.zeros((w_size * h_size, 3), np.float32)
            objp[:, :2] = np.mgrid[0:w_size, 0:h_size].T.reshape(-1, 2)

            img = cv2.resize(orignal, dsize=(0, 0), fx=1 / 4, fy=1 / 4)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (w_size, h_size), None)
            cv2.drawChessboardCorners(img, (w_size, h_size), corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(1000)



'''
TIS Property List
Exposure Time	ExposureTime
Exposure Auto	ExposureAuto
Exposure Auto Reference	ExposureAutoReference
Exposure Auto Lower Limit	ExposureAutoLowerLimit
Exposure Auto Upper Limit	ExposureAutoUpperLimit
Exposure Auto Upper Limit Auto	ExposureAutoUpperLimitAuto
Exposure Auto Highlight Reduction	ExposureAutoHighlightReduction
Gain	Gain
Gain Auto	GainAuto
Gain Auto Lower Limit	GainAutoLowerLimit
Gain Auto Upper Limit	GainAutoUpperLimit
Focus	Focus
Auto Focus	FocusAuto
Auto Focus ROI Left	AutoFocusROILeft
Auto Focus ROI Top	AutoFocusROITop
Auto Focus ROI Width	AutoFocusROIWidth
Auto Focus ROI Height	AutoFocusROIHeight
Iris	Iris
Auto White Balance	BalanceWhiteAuto
White Balance Red	BalanceWhiteRed
White Balance Green	BalanceWhiteGreen
White Balance Blue	BalanceWhiteBlue
Offset X	OffsetX
Offset Y	OffsetY
Offset Auto Center	OffsetAutoCenter
Sensor Width	SensorWidth
Sensor Height	SensorHeight
Enable Auto Functions ROI	AutoFunctionsROIEnable
Auto Functions ROI Preset	AutoFunctionsROIPreset
Auto Functions ROI Left	AutoFunctionsROILeft
Auto Functions ROI Top	AutoFunctionsROITop
Auto Functions ROI Width	AutoFunctionsROIWidth
Auto Functions ROI Height	AutoFunctionsROIHeight
'''

'''
# Just in case trigger mode is enabled, disable it.
try:
    #Tis.Set_Property("TriggerMode","Off")
    Tis.Set_Property("TriggerMode", "False")

except Exception as error:
    print(error)
'''

