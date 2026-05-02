import cv2
import numpy as np

def nothing(x):
    pass

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_SETTINGS, 1)

    cv2.namedWindow("HSV Tuner")
    # 🌟 升级版：加入 H (色相) 滑块，精准定位你的绿光频率！
    cv2.createTrackbar("H_MIN", "HSV Tuner", 35, 179, nothing)
    cv2.createTrackbar("H_MAX", "HSV Tuner", 90, 179, nothing)
    cv2.createTrackbar("S_MIN", "HSV Tuner", 10, 255, nothing)  # 默认降到 10，防止白光过滤
    cv2.createTrackbar("V_MIN", "HSV Tuner", 150, 255, nothing) # 默认拉高到 150，只看亮光

    print("✅ 绿光专属雷达已启动！调整滑块，找出极品参数！按 'q' 退出。")

    while True:
        ret, frame = cap.read()
        if not ret: break

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 实时获取四个滑块的数值
        h_min = cv2.getTrackbarPos("H_MIN", "HSV Tuner")
        h_max = cv2.getTrackbarPos("H_MAX", "HSV Tuner")
        s_min = cv2.getTrackbarPos("S_MIN", "HSV Tuner")
        v_min = cv2.getTrackbarPos("V_MIN", "HSV Tuner")

        # 🟢 实时生成绿光遮罩 (代码极其清爽)
        lower_green = np.array([h_min, s_min, v_min])
        upper_green = np.array([h_max, 255, 255])

        laser_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 加上膨胀效果
        kernel = np.ones((5, 5), np.uint8)
        laser_mask = cv2.dilate(laser_mask, kernel, iterations=1)

        cv2.imshow("Camera (Real)", frame)
        cv2.imshow("Laser Mask (Target)", laser_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"🎯 最终极品参数 ---> H范围:[{h_min}, {h_max}], S下限: {s_min}, V下限: {v_min}")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()