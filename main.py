import cv2
import math
import numpy as np
from Target.shape_detect import process_shapes, detect_laser
from Serial.communicate import SerialManager 

def main():
    # 🚨 树莓派专属：删除了 cv2.CAP_DSHOW，使用 Linux 原生驱动
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_SETTINGS, 1)
    if not cap.isOpened(): 
        print("❌ 摄像头打开失败！")
        return
    
    # 🚨 部署到树莓派时，记得改成 '/dev/ttyUSB0'
    serial_manager = SerialManager(port='COM7', baudrate=115200) 
    
    current_mode = 1 
    target_index = 0
    
    smooth_tx, smooth_ty = 0, 0
    trace_points = []      
    current_trace_idx = 0  
    
    smooth_goal_x, smooth_goal_y = 0.0, 0.0 
    
    last_err_x, last_err_y = 0.0, 0.0
    last_laser_cx, last_laser_cy = 0, 0 
    lost_counter = 0  
    
    print("✅ 单曝光脱机实战版 启动！\n👉 请使用 STM32 上的物理按键进行模式切换与启动！")

    while True:
        ret, frame = cap.read()
        if not ret: break
        
        results, display_frame, thresh = process_shapes(frame, current_mode)
        laser_cx, laser_cy = detect_laser(frame)
        
        display_err_x, display_err_y = 0.0, 0.0
        current_target_name = "None"
        
        if laser_cx is not None:
            cv2.drawMarker(display_frame, (laser_cx, laser_cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        if serial_manager.ser is not None:
            while serial_manager.ser.in_waiting > 0:
                try:
                    msg = serial_manager.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "NEXT_TARGET" in msg:
                        if current_mode != 3: 
                            target_index += 1
                    elif msg.startswith("MODE:"):
                        new_mode = int(msg.split(":")[1])
                        if new_mode in [1, 2, 3, 4, 5]:
                            current_mode = new_mode
                            target_index = 0
                            smooth_tx, smooth_ty = 0, 0 
                            trace_points = []
                            smooth_goal_x, smooth_goal_y = 0.0, 0.0 
                            lost_counter = 0 
                            print(f"🔄 收到单片机指令，切换至模式: {current_mode}")
                except: pass

        if current_mode in [1, 4, 5]: 
            target_index = 0 

        # ==========================================
        # 🎯 任务 1, 2, 4, 5 逻辑 (静态+动态目标)
        # ==========================================
        if current_mode in [1, 2, 4, 5]:
            if len(results) > 0 and target_index < len(results):
                target = results[target_index]
                current_target_name = target['shape'] 
                
                if smooth_tx == 0 and smooth_ty == 0:
                    smooth_tx, smooth_ty = target['cx'], target['cy']
                else:
                    smooth_tx = smooth_tx * 0.3 + target['cx'] * 0.7
                    smooth_ty = smooth_ty * 0.3 + target['cy'] * 0.7

                if laser_cx is not None and laser_cy is not None:
                    err_x = smooth_tx - laser_cx
                    err_y = smooth_ty - laser_cy
                    last_err_x, last_err_y = err_x, err_y
                    lost_counter = 0 
                    display_err_x, display_err_y = err_x, err_y
                    serial_manager.send_gimbal_data(err_x, err_y, state=1)
                else:
                    if lost_counter < 30: 
                        lost_counter += 1
                        display_err_x, display_err_y = last_err_x, last_err_y
                        cv2.putText(display_frame, "BLIND COASTING!", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
                        serial_manager.send_gimbal_data(last_err_x, last_err_y, state=1)
                    else:
                        serial_manager.send_gimbal_data(0, 0, state=0)
            else:
                smooth_tx, smooth_ty = 0, 0 
                serial_manager.send_gimbal_data(0, 0, state=0)
                if current_mode == 2 and target_index > 0:
                    cv2.putText(display_frame, "MISSION 2 COMPLETED!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        # ==========================================
        # 🎯 任务 3 逻辑 (高阶轨迹追踪)
        # ==========================================
        elif current_mode == 3: 
            current_target_name = "Tracing Mode"
            if len(results) > 0 and target_index < len(results):
                target = results[target_index]
                
                # 1. 生成轨迹
                if len(trace_points) == 0 and 'contour' in target and len(target['contour']) > 0:
                    trace_points = []
                    cx, cy = target['cx'], target['cy']
                    
                    # 🌟 优化1：全场轮廓强制内缩 15%，完美走在黑带中间！
                    shrink_ratio = 0.90
                    
                    if target['shape'] == 'YuanXing':
                        area = cv2.contourArea(target['contour'])
                        # 🌟 半径缩减！
                        radius = int(((area / 3.14159) ** 0.5) * shrink_ratio)
                        
                        for i in range(72):
                            angle = i * (2 * 3.14159 / 72)
                            x = int(cx + radius * math.cos(angle))
                            y = int(cy + radius * math.sin(angle))
                            trace_points.append((x, y))
                    
                    else:
                        vertices = target['contour']
                        
                        # 🌟 优化2：顶点坐标重排，永远强制从左上角起步！
                        pts = [p[0] for p in vertices]
                        start_idx = np.argmin([p[0] + p[1] for p in pts])
                        vertices = np.roll(vertices, -start_idx, axis=0)

                        # 🌟 把顶点按 shrink_ratio 向中心收缩！
                        shrunk_vertices = []
                        for pt in vertices:
                            vx, vy = pt[0]
                            new_x = int(cx + (vx - cx) * shrink_ratio)
                            new_y = int(cy + (vy - cy) * shrink_ratio)
                            shrunk_vertices.append([[new_x, new_y]])
                            
                        # 画线
                        for i in range(len(shrunk_vertices)):
                            pt1 = shrunk_vertices[i][0]
                            pt2 = shrunk_vertices[(i+1) % len(shrunk_vertices)][0] 
                            dist = math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])
                            
                            steps = max(int(dist / 1.5), 5) 
                            for j in range(steps):
                                x = int(pt1[0] + (pt2[0] - pt1[0]) * (j / float(steps)))
                                y = int(pt1[1] + (pt2[1] - pt1[1]) * (j / float(steps)))
                                trace_points.append((x, y))
                            
                            # 🌟 优化3：删掉了此处人工停车的 20 个废点，过弯将一气呵成！
                                
                    current_trace_idx = 0
                    
                # 2. 追踪逻辑
                if laser_cx is not None and laser_cy is not None and current_trace_idx < len(trace_points):
                    
                    lookahead_steps = 18  # 适当的前瞻
                    target_idx = min(current_trace_idx + lookahead_steps, len(trace_points) - 1)
                    goal_x, goal_y = trace_points[target_idx]
                    
                    err_x = goal_x - laser_cx
                    err_y = goal_y - laser_cy
                    
                    if abs(err_x) < 1.0: err_x = 0.0 
                    if abs(err_y) < 1.0: err_y = 0.0

                    err_x = max(-45.0, min(45.0, err_x))
                    err_y = max(-45.0, min(45.0, err_y))
                    
                    last_laser_cx, last_laser_cy = laser_cx, laser_cy 
                    last_err_x, last_err_y = err_x, err_y
                    lost_counter = 0
                    display_err_x, display_err_y = err_x, err_y
                    
                    serial_manager.send_gimbal_data(err_x * 0.9, err_y * 0.9, state=1)
                    
                    # 画图
                    for i, p in enumerate(trace_points):
                        if i < current_trace_idx: cv2.circle(display_frame, p, 1, (100, 100, 100), -1)
                        elif i == current_trace_idx:
                            cv2.circle(display_frame, p, 2, (0, 0, 255), -1)
                    
                    cv2.circle(display_frame, (int(goal_x), int(goal_y)), 6, (0, 255, 255), -1)
                    cv2.line(display_frame, (laser_cx, laser_cy), (int(goal_x), int(goal_y)), (0, 255, 255), 1)
                    
                    # 🌟 优化4：放宽推进阈值，彻底解决圆画到一半卡死的问题！
                    current_dist = math.hypot(goal_x - laser_cx, goal_y - laser_cy)
                    if current_dist < 15.0:
                        current_trace_idx += 1 
                        
                    if current_trace_idx >= len(trace_points):
                        trace_points = [] 
                        target_index += 1
                        
                else: 
                    # 丢失追踪时的应急盲走
                    if lost_counter < 30 and current_trace_idx < len(trace_points):
                        lost_counter += 1
                        if lost_counter % 3 == 0:
                            current_trace_idx += 1 
                            if current_trace_idx >= len(trace_points):
                                trace_points = []
                                target_index += 1
                        if current_trace_idx < len(trace_points):
                            goal_x, goal_y = trace_points[current_trace_idx]
                            fake_err_x = max(-60, min(60, goal_x - last_laser_cx))
                            fake_err_y = max(-60, min(60, goal_y - last_laser_cy))
                            display_err_x, display_err_y = fake_err_x, fake_err_y
                            serial_manager.send_gimbal_data(fake_err_x * 0.45, fake_err_y * 0.45, state=1)
                        else:
                            serial_manager.send_gimbal_data(0, 0, state=0)
                    else:
                        serial_manager.send_gimbal_data(0, 0, state=0)
            else:
                serial_manager.send_gimbal_data(0, 0, state=0)

        cv2.putText(display_frame, f"MODE: {current_mode} | Target: {current_target_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow("Camera View (AI)", display_frame)
        if thresh is not None: cv2.imshow("Binary View (Debug)", thresh)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break

    serial_manager.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()