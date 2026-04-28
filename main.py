import cv2
from Target.shape_detect import process_shapes, detect_laser
from Serial.communicate import SerialManager 

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened(): return
    
    cap.set(cv2.CAP_PROP_SETTINGS, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 

    serial_manager = SerialManager(port='COM6', baudrate=115200)
    
    current_mode = 1 
    target_index = 0
    
    smooth_tx, smooth_ty = 0, 0
    trace_points = []      
    current_trace_idx = 0  
    
    last_err_x, last_err_y = 0.0, 0.0
    last_laser_cx, last_laser_cy = 0, 0 
    lost_counter = 0  
    
    print("✅ 系统启动! 按 1~5 切换任务模式，按 'q' 退出。")

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
                        # 🚨 绝杀乱跳 Bug：在模式 3 吃豆人时，绝对禁止单片机插嘴切目标！
                        if current_mode != 3: 
                            target_index += 1
                    elif msg.startswith("MODE:"):
                        new_mode = int(msg.split(":")[1])
                        if new_mode in [1, 2, 3, 4, 5]:
                            current_mode = new_mode
                            target_index = 0
                            smooth_tx, smooth_ty = 0, 0 
                            trace_points = []
                            lost_counter = 0 
                except: pass

        if current_mode in [1, 4, 5]: 
            target_index = 0 

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
                    # 模式 1/2/4 的盲开，放宽到 30 帧 (1秒)，给它足够时间滑入靶心
                    if lost_counter < 30: 
                        lost_counter += 1
                        display_err_x, display_err_y = last_err_x, last_err_y
                        cv2.putText(display_frame, "BLIND COASTING!", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
                        serial_manager.send_gimbal_data(last_err_x, last_err_y, state=1)
                    else:
                        serial_manager.send_gimbal_data(0, 0, state=0)
            else:
                smooth_tx, smooth_ty = 0, 0 
                serial_manager.send_gimbal_data(0, 0, state=0)

                if current_mode == 2 and target_index > 0:
                    cv2.putText(display_frame, "MISSION 2 COMPLETED!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        elif current_mode == 3: # 模式3：吃豆人描边
            current_target_name = "Tracing Mode"
            if len(results) > 0 and target_index < len(results):
                target = results[target_index]
                
                if len(trace_points) == 0 and len(target['contour']) > 0:
                    vertices = target['contour']
                    trace_points = []
                    for i in range(len(vertices)):
                        pt1 = vertices[i][0]
                        pt2 = vertices[(i+1) % len(vertices)][0] 
                        for j in range(30):
                            x = int(pt1[0] + (pt2[0] - pt1[0]) * (j / 30.0))
                            y = int(pt1[1] + (pt2[1] - pt1[1]) * (j / 30.0))
                            trace_points.append((x, y))
                    current_trace_idx = 0

                if laser_cx is not None and laser_cy is not None and current_trace_idx < len(trace_points):
                    goal_x, goal_y = trace_points[current_trace_idx]
                    
                    err_x = goal_x - laser_cx
                    err_y = goal_y - laser_cy
                    
                    last_laser_cx, last_laser_cy = laser_cx, laser_cy 
                    last_err_x, last_err_y = err_x, err_y
                    lost_counter = 0
                    
                    display_err_x, display_err_y = err_x, err_y
                    serial_manager.send_gimbal_data(err_x, err_y, state=1)
                    
                    for i, p in enumerate(trace_points):
                        if i < current_trace_idx:
                            cv2.circle(display_frame, p, 2, (100, 100, 100), -1)
                        elif i == current_trace_idx:
                            cv2.circle(display_frame, p, 6, (0, 255, 255), -1)
                            cv2.circle(display_frame, p, 2, (0, 0, 255), -1)
                            cv2.line(display_frame, (laser_cx, laser_cy), p, (0, 255, 255), 1)
                        else:
                            cv2.circle(display_frame, p, 2, (255, 255, 255), -1)
                    
                    if (err_x**2 + err_y**2) ** 0.5 < 15.0:
                        current_trace_idx += 1 
                        if current_trace_idx >= len(trace_points):
                            trace_points = [] 
                            target_index += 1
                else:
                    # ==========================================
                    # 🌟🌟 暴力重构：推土机级幽灵牵引！
                    # ==========================================
                    # 容忍瞎眼的时间放宽到 60 帧（整整 2 秒），绝不轻言放弃！
                    if lost_counter < 60 and current_trace_idx < len(trace_points):
                        lost_counter += 1
                        
                        # 让豆子跑快一点 (每瞎 2 帧跑一步，快速建立拉力)
                        if lost_counter % 2 == 0:
                            current_trace_idx += 1 
                            if current_trace_idx >= len(trace_points):
                                trace_points = []
                                target_index += 1
                                
                        if current_trace_idx < len(trace_points):
                            goal_x, goal_y = trace_points[current_trace_idx]
                            fake_err_x = goal_x - last_laser_cx
                            fake_err_y = goal_y - last_laser_cy
                            
                            # 🚨 解除封印！最大拉力放宽到 80！给单片机足够的油门越野！
                            fake_err_x = max(-80, min(80, fake_err_x))
                            fake_err_y = max(-80, min(80, fake_err_y))
                            
                            display_err_x, display_err_y = fake_err_x, fake_err_y
                            cv2.putText(display_frame, "GHOST PULLING POWER MAX!", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                            
                            serial_manager.send_gimbal_data(fake_err_x, fake_err_y, state=1)
                        else:
                            serial_manager.send_gimbal_data(0, 0, state=0)
                    else:
                        serial_manager.send_gimbal_data(0, 0, state=0)
            else:
                serial_manager.send_gimbal_data(0, 0, state=0)

        cv2.putText(display_frame, f"MODE: {current_mode} | Target: {current_target_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        if (laser_cx is not None or lost_counter < 60) and current_target_name != "None":
            color = (0, 255, 0) if (abs(display_err_x) < 5 and abs(display_err_y) < 5) else (0, 0, 255)
            cv2.putText(display_frame, f"Err X: {display_err_x:.1f} | Err Y: {display_err_y:.1f}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        else:
            cv2.putText(display_frame, "LASER LOST or NO TARGET!", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Camera View (AI)", display_frame)
        cv2.imshow("Binary View (Debug)", thresh)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('1'): current_mode, target_index = 1, 0
        elif key == ord('2'): current_mode, target_index = 2, 0
        elif key == ord('3'): current_mode, target_index, trace_points = 3, 0, []
        elif key == ord('4'): current_mode, target_index = 4, 0
        elif key == ord('5'): current_mode, target_index = 5, 0

    serial_manager.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()