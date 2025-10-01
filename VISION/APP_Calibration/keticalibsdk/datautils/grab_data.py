from ketisdk.sensor.realsense_sensor import RSSensor
import cv2
import datetime
if __name__ == '__main__':

    sensor_enable = True

    # get data from realsense
    if sensor_enable:
        sensor = RSSensor()
        sensor.start()

    while True:
        time_info = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S_')

        if sensor_enable:
            rgb_data, depth_data = sensor.get_data()  # realsense data
            rgb_data = rgb_data[:, :, ::-1]

        cv2.imshow("rgb",rgb_data)
        key=cv2.waitKey(30)
        if key==ord("c"):
            print(f"save data: {time_info}")
            cv2.imwrite(f"./place/{time_info}_rgb.png",rgb_data)
            cv2.imwrite(f"./place/{time_info}_depth.png",depth_data)