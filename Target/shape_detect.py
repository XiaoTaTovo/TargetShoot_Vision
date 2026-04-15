import cv2
import numpy as np
def detect_laser(frame):
    # 将图像转为 HSV 色彩空间，因为红色在 HSV 中最容易提取
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 红色在 HSV 中横跨 0~10 和 160~180 两个区域
    lower_red1 = np.array([0, 100, 200])   # 最后一个 200 是亮度 (V)，激光很亮！
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 200])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 找红点的轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # 找到面积最大的红斑（防止背景有红色小噪点）
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy
    return None, None

def process_shapes(frame, mode):
    """
    处理图像，根据 mode 执行不同任务
    mode == 1: 任务 1 (寻找 A4 黑色矩形靶框)
    mode == 2: 任务 2 (寻找多个几何图形并按边数排序)
    """
    display_frame = frame.copy()
    results = []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY_INV)
    #二值化，低于90的像素变成255（白色），高于90的像素变成0（黑色）
    # 如果现场调参黑色的边框中混入了白色噪点，可以适当调高这个阈值
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#圈出白色区域的轮廓，方便我们后续分析这些轮廓的形状和位置
    # ==========================================
    # 🎯 模式 1：单目标 A4 靶框追踪
    # ==========================================
    if mode == 1:
        max_area = 0
        best_target = None
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 5000: continue # A4框很大，过滤小的
                
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # 找最大的四边形
            if len(approx) == 4 and area > max_area:
                max_area = area
                best_target = cnt
                
        if best_target is not None:
            M = cv2.moments(best_target)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                results.append({'shape': 'A4_Target', 'cx': cx, 'cy': cy, 'vertices': 4})
                
                # 画图反馈
                cv2.drawContours(display_frame, [best_target], -1, (255, 0, 0), 3)
                cv2.circle(display_frame, (cx, cy), 5, (0, 0, 255), -1)

    # ==========================================
    # 🎯 模式 2：上帝透视眼调试模式
    # ==========================================
    elif mode == 2:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1500 : continue 
                
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            vertices = len(approx)
            
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            
            # 2. 计算圆度
            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            
            # 3. 稍微降低圆度的及格线，容忍斜视造成的椭圆
            if circularity > 0.78:
                shape_name, sort_key = "YuanXing", 0
            elif vertices == 3:
                shape_name, sort_key = "SanJiao", 3
            elif vertices == 4:
                shape_name, sort_key = "SiBian", 4
            elif vertices == 10:
                shape_name, sort_key = "WuJiaoXing", 10
            elif vertices == 12:
                shape_name, sort_key = "ShiZiJia", 12
            else:
                shape_name, sort_key = "Unknow", 99 # 暂时不要扔掉未知图形，画出来看看是什么鬼！

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                results.append({'shape': shape_name, 'cx': cx, 'cy': cy, 'vertices': sort_key})
                
                # 👑 上帝视角：把面积(A)和圆度(C)直接打印在图形旁边！
                cv2.drawContours(display_frame, [approx], -1, (0, 255, 0), 2)
                debug_text = f"{shape_name} A:{int(area)} C:{circularity:.2f}"
                cv2.putText(display_frame, debug_text, (cx - 40, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        results = sorted(results, key=lambda x: x['vertices'])
        if len(results) > 0:
            cv2.circle(display_frame, (results[0]['cx'], results[0]['cy']), 8, (0, 0, 255), -1)

    return results, display_frame, thresh
# 这个函数的输入是摄像头捕获的每一帧图像和当前模式，输出是一个包含识别结果的列表、一个带有绘制反馈的显示帧，以及二值化后的调试帧。
# 识别结果列表中的每个元素都是一个字典，包含了目标的形状名称、中心坐标和顶点数量（用于排序）。显示帧上会画出识别到的目标轮廓，并在旁边标注相关信息。调试帧则是二值化后的图像，方便你观察轮廓提取的效果。
