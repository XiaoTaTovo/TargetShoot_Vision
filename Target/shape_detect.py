import cv2
import numpy as np
import math
_last_otsu_thresh = -1

def detect_laser(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    lower_green = np.array([35, 0, 111]) # 这个参数如果之前测了很稳，就保留
    upper_green = np.array([90, 255, 255])
    
    laser_mask = cv2.inRange(hsv, lower_green, upper_green)

    # 膨胀 1 次就够了，千万别写 3！
    kernel = np.ones((5, 5), np.uint8)
    laser_mask = cv2.dilate(laser_mask, kernel, iterations=1)
    
    contours, _ = cv2.findContours(laser_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_cx, best_cy = None, None
    max_area = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 🌟 门槛提高到 10！过滤掉一切微小反光噪点！
        if area > 10:
            if area > max_area:
                max_area = area
                # 🌟 使用外接圆的圆心，比算矩形和重心稳得多，无视光斑变形！
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                best_cx = int(x)
                best_cy = int(y)
                    
    return best_cx, best_cy


def process_shapes(frame, mode):
    display_frame = frame.copy()
    results = []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 🌟 优化1：把模糊核降到 3x3，保护多边形的“尖角”不被盘圆！
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    
    # ==========================================
    # 🌟 绝杀边框晃动：阈值平滑器！
    # ==========================================
    global _last_otsu_thresh
    
    otsu_val, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    
    if _last_otsu_thresh == -1:
        _last_otsu_thresh = otsu_val
    else:
        _last_otsu_thresh = _last_otsu_thresh * 0.9 + otsu_val * 0.1
        
    _, thresh = cv2.threshold(blurred, int(_last_otsu_thresh), 255, cv2.THRESH_BINARY_INV)

    # 现场缝合术
    kernel_close = np.ones((5, 5), np.uint8) 
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_close)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ==========================================
    # 🎯 模式 1(静态靶), 4(动态靶心), 5(动态画圆)：都依赖 A4 黑框！
    # ==========================================
    if mode in [1, 4, 5]:
        max_area = 0
        best_target_rect = None 
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 3000 or area > 150000: continue 
            
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if min(w, h) == 0: continue
            
            aspect_ratio = max(w, h) / min(w, h)
            
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
            if area < 800 or area > 10000: continue 
            
            x, y, w, h = cv2.boundingRect(cnt)
            if y + h > 478 or x <= 2 or x + w > 638 or y <= 2:
                continue
                
            # 🌟 优化2：逼近精度提高到 0.01！让五角星的尖角死死稳住！
            epsilon = 0.01 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            vertices = len(approx)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            
            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            
            if circularity > 0.78: 
                shape_name, sort_key = "YuanXing", 0
            else:
                shape_name, sort_key = f"Shape_{vertices}", vertices

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                
                if shape_name == "YuanXing":
                    (circle_x, circle_y), radius = cv2.minEnclosingCircle(cnt)
                    cv2.circle(display_frame, (int(circle_x), int(circle_y)), int(radius), (0, 255, 0), 2)
                    contour_points = cnt if mode == 3 else []
                else:
                    cv2.drawContours(display_frame, [approx], -1, (0, 255, 0), 2)
                    contour_points = approx if mode == 3 else []
                    
                results.append({'shape': shape_name, 'cx': cx, 'cy': cy, 'vertices': sort_key, 'contour': contour_points})

        results = sorted(results, key=lambda x: x['vertices'])

    return results, display_frame, thresh