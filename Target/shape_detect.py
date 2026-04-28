import cv2
import numpy as np
import math

def detect_laser(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # ==========================================
    # 🌟 终极夜视仪：黑胶带吸光太严重，疯狂拉低 V (明度) 和 S (饱和度) 的下限！
    # 只要背景够黑，V 降到 50 都不怕误识别！
    # ==========================================
    lower_red1 = np.array([0, 36, 111])   
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 36, 111])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    laser_mask = cv2.bitwise_or(mask1, mask2)

    # ==========================================
    # 🌟 暴力放大：换用 5x5 的大核，膨胀 1 次！把针尖大小的光点强行炸开！
    # ==========================================
    kernel = np.ones((5, 5), np.uint8)
    laser_mask = cv2.dilate(laser_mask, kernel, iterations=1)
    
    cv2.imshow("Laser Magic", laser_mask)

    contours, _ = cv2.findContours(laser_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_cx, best_cy = None, None
    max_area = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 🌟 面积门槛降到 > 0！只要是个红光点存活下来了，哪怕 1 个像素我也认！
        if area > 0:
            rect = cv2.minAreaRect(cnt)
            width, height = rect[1]
            if width == 0 or height == 0: continue
            
            aspect_ratio = max(width, height) / min(width, height)
            
            # 放宽比例到 6.0，极其微弱的光斑容易被拉伸变形
            if aspect_ratio <= 6.0:
                if area > max_area:
                    max_area = area
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        best_cx = int(M["m10"] / M["m00"])
                        best_cy = int(M["m01"] / M["m00"])
                    
    return best_cx, best_cy


def process_shapes(frame, mode):
    display_frame = frame.copy()
    results = []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 🌟 绝杀光线变化：引入 THRESH_OTSU 大津法！
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    
    # 🌟 现场缝合术：把反光断裂的黑胶带重新黏合！
    kernel_close = np.ones((7, 7), np.uint8) 
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_close)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ==========================================
    # 🎯 模式 1(静态靶), 4(动态靶心), 5(动态画圆)：都依赖 A4 黑框！
    # ==========================================
    if mode in [1, 4, 5]:
        max_area = 0
        best_target_rect = None # 🌟 我们存矩形，不再存多边形
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 3000 or area > 150000: continue # 🌟 保护上限和下限，隔绝小图形和外界大阴影
            
            # 🚨 终极防弹衣：不再数 approx 有没有 4 个角！直接算外接矩形！
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if min(w, h) == 0: continue
            
            aspect_ratio = max(w, h) / min(w, h)
            
            # 🌟 只要长宽比像个框（0.8 到 2.5 之间），哪怕胶带贴得像狗啃的也认！
            if 0.8 <= aspect_ratio <= 2.5:
                if area > max_area:
                    max_area = area
                    best_target_rect = rect
                
        if best_target_rect is not None:
            box = cv2.boxPoints(best_target_rect)
            box = np.int32(box) 
            
            cx, cy = int(best_target_rect[0][0]), int(best_target_rect[0][1])
            
            cv2.drawContours(display_frame, [box], 0, (255, 0, 0), 2)
            cv2.circle(display_frame, (cx, cy), 5, (0, 0, 255), -1)

            if mode == 1 or mode == 4:
                results.append({'shape': 'A4_Center', 'cx': cx, 'cy': cy})
                cv2.drawMarker(display_frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            elif mode == 5:
                pt0, pt1, pt2 = box[0], box[1], box[2]
                side1 = math.hypot(pt0[0] - pt1[0], pt0[1] - pt1[1])
                side2 = math.hypot(pt1[0] - pt2[0], pt1[1] - pt2[1])
                
                w_pixels = min(side1, side2)
                pixel_per_mm = w_pixels / 210.0
                r_pixels = 60 * pixel_per_mm
                
                mid_x = (pt0[0] + pt1[0]) / 2.0
                mid_y = (pt0[1] + pt1[1]) / 2.0
                vec_x = mid_x - cx
                vec_y = mid_y - cy
                
                length = math.hypot(vec_x, vec_y)
                if length != 0:
                    u_x, u_y = vec_x / length, vec_y / length
                    target_x = int(cx + u_x * r_pixels)
                    target_y = int(cy + u_y * r_pixels)
                    
                    results.append({'shape': '6cm_Circle', 'cx': target_x, 'cy': target_y})
                    cv2.circle(display_frame, (target_x, target_y), 8, (255, 0, 255), -1)
                    cv2.line(display_frame, (cx, cy), (target_x, target_y), (255, 0, 255), 2)
                    cv2.putText(display_frame, "6cm Sync Point", (target_x+10, target_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    # ==========================================
    # 🎯 模式 2 和 3 代码 (小图形专属)
    # ==========================================
    elif mode == 2 or mode == 3:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 🌟 严格限制面积：下限 800 (防噪点)，上限 10000 (防A4纸和阴影)
            if area < 800 or area > 10000: continue 
                
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            vertices = len(approx)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            
            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            
            # ==========================================
            # 🌟 赛题级排序逻辑：几条边就排第几！彻底解决十字星问题
            # ==========================================
            if circularity > 0.78: 
                shape_name, sort_key = "YuanXing", 0  # 圆形排第一
            else:
                shape_name, sort_key = f"Shape_{vertices}", vertices # 三角(3) -> 正方(4) -> 十字(12)

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                contour_points = approx if mode == 3 else []
                results.append({'shape': shape_name, 'cx': cx, 'cy': cy, 'vertices': sort_key, 'contour': contour_points})
                cv2.drawContours(display_frame, [approx], -1, (0, 255, 0), 2)

        # 按边数升序排列，自动规划打靶路线！
        results = sorted(results, key=lambda x: x['vertices'])

    return results, display_frame, thresh