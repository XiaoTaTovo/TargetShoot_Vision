import cv2
import numpy as np
import math

def detect_laser(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # ==========================================
    # 🟢 绿光终极过滤
    # ==========================================
    lower_green = np.array([35, 10, 150])  
    upper_green = np.array([90, 255, 255])

    laser_mask = cv2.inRange(hsv, lower_green, upper_green)

    # 暴力放大：换用 5x5 的大核，膨胀 1 次！
    kernel = np.ones((5, 5), np.uint8)
    laser_mask = cv2.dilate(laser_mask, kernel, iterations=1)

    cv2.imshow("Laser Magic", laser_mask)

    contours, _ = cv2.findContours(laser_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_cx, best_cy = None, None
    max_area = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 🌟 面积门槛降到 > 0！哪怕 1 个像素也认！
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
    
    if mode in [1, 4, 5]:
        # 模式 1,4,5 (找大黑框)：胶带反光严重，用 9x9 粗针线强行融合裂缝！
        kernel_close = np.ones((9, 9), np.uint8) 
    else:
        # 模式 2,3 (找小图形)：需要保住尖角和直角，用 3x3 细针线！
        kernel_close = np.ones((3, 3), np.uint8)

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
            if area < 3000 or area > 150000: continue # 保护上限和下限
            
            # 🚨 终极防弹衣：算外接矩形
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if min(w, h) == 0: continue
            
            aspect_ratio = max(w, h) / min(w, h)
            
            # 只要长宽比像个框（0.8 到 2.5 之间）
            if 0.8 <= aspect_ratio <= 2.5:
                extent = area / (w * h)
                if extent > 0.85: # 面积占比至少 85%，极大概率是完整的 A4 黑框
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
                
            # 🌟 1. 粗略相亲：用 0.02 的美颜滤镜，先看清它是几边形！
            epsilon_rough = 0.02 * cv2.arcLength(cnt, True)
            approx_rough = cv2.approxPolyDP(cnt, epsilon_rough, True)
            vertices = len(approx_rough)
            
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            
            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            
            if circularity > 0.78: 
                shape_name, sort_key = "YuanXing", 0  
            else:
                shape_name, sort_key = f"Shape_{vertices}", vertices

            # 使用极其抗畸变的外接矩形中心法 (抛弃受畸变影响的重心法)
            rect = cv2.minAreaRect(cnt)
            cx = int(rect[0][0])
            cy = int(rect[0][1])
            
            # ==========================================
            # 🌟🌟 2. 终极修复：因材施教，动态分配描边画质！🌟🌟
            # ==========================================
            if mode == 3:
                if shape_name == "YuanXing":
                    contour_points = approx_rough
                elif vertices > 6:
                    # 复杂图形（十字12边、星星10边）
                    # 必须用 4K显微镜画质 (0.004)！死死保住每一个内凹角和直角！
                    epsilon_hd = 0.004 * cv2.arcLength(cnt, True)
                    contour_points = cv2.approxPolyDP(cnt, epsilon_hd, True)
                else:
                    # 简单图形（正方4边、三角3边）
                    # 胶带边缘太烂，直接沿用美颜后的 approx_rough (0.02)！
                    # 算法会自动无视掉左下角的刺和右下角的缺角，强制生成极度完美的 4 个角！
                    contour_points = approx_rough
            else:
                contour_points = []

            results.append({'shape': shape_name, 'cx': cx, 'cy': cy, 'vertices': sort_key, 'contour': contour_points})
            
            # 画框显示：用最终决定的那个轮廓画
            draw_target = contour_points if mode == 3 and len(contour_points) > 0 else approx_rough
            cv2.drawContours(display_frame, [draw_target], -1, (0, 255, 0), 2)

        # 按边数升序排列
        results = sorted(results, key=lambda x: x['vertices'])

    return results, display_frame, thresh