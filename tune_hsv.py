import cv2
import numpy as np

def nothing(x):
    pass

def main():
    # 打开摄像头，并弹出属性面板让你随时调曝光
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_SETTINGS, 1)

    # 创建一个带滑块的控制台窗口
    cv2.namedWindow("HSV Tuner")
    # 我们只调最核心的 S(饱和度下限) 和 V(明度下限)
    cv2.createTrackbar("S_MIN", "HSV Tuner", 30, 255, nothing)
    cv2.createTrackbar("V_MIN", "HSV Tuner", 100, 255, nothing)

    print("✅ 调参雷达已启动！滑动滑块，直到右边的黑窗口只留下唯一的激光点！按 'q' 退出。")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 同样加上高斯模糊
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 实时获取你拖动滑块的数值
        s_min = cv2.getTrackbarPos("S_MIN", "HSV Tuner")
        v_min = cv2.getTrackbarPos("V_MIN", "HSV Tuner")

        # 绿色的大致范围，S下限依然要调低(比如40)防止绿激光中心过曝变白漏抓
        lower_green = np.array([35, s_min, v_min])
        upper_green = np.array([85, 255, 255])
        laser_mask = cv2.inRange(hsv, lower_green, upper_green)
        # 删掉 mask2 和 bitwise_or 的逻辑
        
        # 加上膨胀效果，模拟真实运行情况
        kernel = np.ones((5, 5), np.uint8)
        laser_mask = cv2.dilate(laser_mask, kernel, iterations=1)

        # 显示原画面和处理后的画面
        cv2.imshow("Camera (Real)", frame)
        cv2.imshow("Laser Mask (Target)", laser_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"🎯 最终极品参数 ---> S下限: {s_min}, V下限: {v_min}")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()