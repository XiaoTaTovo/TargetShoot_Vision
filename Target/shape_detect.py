import cv2
import numpy as np
import math






import cv2
import numpy as np

def detect_laser(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 🌟 绝杀红墨水，放过红激光！
    # S(饱和度)>80 过滤掉普通的白光干扰
    # V(明度)>200 是核心！红墨水绝对达不到 200，只有激光能！
    lower_red1 = np.array([0, 80, 200])   
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 80, 200])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    laser_mask = cv2.bitwise_or(mask1, mask2)

    # 🚨 拔掉“死胡同”里的钉子：彻底删除 MORPH_OPEN！
    # 直接用膨胀 (Dilate)！哪怕只抓到了激光边缘的 1 颗红色像素，
    # 也能瞬间把它放大成一颗饱满的星星！
    kernel = np.ones((3, 3), np.uint8)
    laser_mask = cv2.dilate(laser_mask, kernel, iterations=1)
    
    cv2.imshow("Laser Magic", laser_mask)

    contours, _ = cv2.findContours(laser_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_cx, best_cy = None, None
    max_area = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 放宽面积，极小的光点也能存活
        if 1 < area < 800:
            rect = cv2.minAreaRect(cnt)
            width, height = rect[1]
            if width == 0 or height == 0: continue
            
            aspect_ratio = max(width, height) / min(width, height)
            
            # 放宽到 4.0，兼容斜着打靶产生的严重椭圆
            if aspect_ratio <= 4.0:
                if area > max_area:
                    max_area = area
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        best_cx = int(M["m10"] / M["m00"])
                        best_cy = int(M["m01"] / M["m00"])
                    
    return best_cx, best_cy


# ... 后面的 process_shapes 不动 ...

def process_shapes(frame, mode):
    display_frame = frame.copy()
    results = []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY_INV)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ==========================================
    # 🎯 模式 1(静态靶), 4(动态靶心), 5(动态画圆)：都依赖 A4 黑框！
    # ==========================================
    if mode in [1, 4, 5]:
        max_area = 0
        best_target = None
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 5000 or area > 100000: continue 
                
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            if len(approx) == 4 and area > max_area:
                max_area = area
                best_target = approx
                
        if best_target is not None:
            rect = cv2.minAreaRect(best_target)
            box = cv2.boxPoints(rect)
            box = np.int32(box) 
            
            cx, cy = int(rect[0][0]), int(rect[0][1])
            
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
    # 🎯 模式 2 和 3 代码
    # ==========================================
    elif mode == 2 or mode == 3:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1500 : continue 
                
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            vertices = len(approx)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            
            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            
            if circularity > 0.78: shape_name, sort_key = "YuanXing", 0
            elif vertices == 3: shape_name, sort_key = "SanJiao", 3
            elif vertices == 4: shape_name, sort_key = "SiBian", 4
            else: shape_name, sort_key = "Unknow", 99 

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                contour_points = approx if mode == 3 else []
                results.append({'shape': shape_name, 'cx': cx, 'cy': cy, 'vertices': sort_key, 'contour': contour_points})
                cv2.drawContours(display_frame, [approx], -1, (0, 255, 0), 2)

        results = sorted(results, key=lambda x: x['vertices'])

    return results, display_frame, thresh