import cv2
from Target.shape_detect import process_shapes, detect_laser
from Serial.communicate import SerialManager 

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened(): return
    
    # 🌟 修正 1：摄像头设置挪到循环外面，只设置一次，提高运行效率
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
    # cap.set(cv2.CAP_PROP_EXPOSURE, -6)

    serial_manager = SerialManager(port='COM10', baudrate=115200)
    
    current_mode = 1 
    target_index = 0
    
    # 🌟 修正 2：平滑变量挪到循环外面！
    smooth_tx, smooth_ty = 0, 0
    trace_points = []      
    current_trace_idx = 0  
    
    print("✅ 系统启动! 按 1~5 切换任务模式，按 'q' 退出。")

    while True:
        ret, frame = cap.read()
        if not ret: break
            
        results, display_frame, thresh = process_shapes(frame, current_mode)
        laser_cx, laser_cy = detect_laser(frame)
        
        if laser_cx is not None:
            cv2.drawMarker(display_frame, (laser_cx, laser_cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        # (串口接收逻辑保持不变...)
        if serial_manager.ser is not None:
            while serial_manager.ser.in_waiting > 0:
                try:
                    msg = serial_manager.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "NEXT_TARGET" in msg:
                        target_index += 1
                    elif msg.startswith("MODE:"):
                        new_mode = int(msg.split(":")[1])
                        if new_mode in [1, 2, 3, 4, 5]:
                            current_mode = new_mode
                            target_index = 0
                            smooth_tx, smooth_ty = 0, 0 # 切换模式时重置平滑值
                            trace_points = []
                except: pass

        # ==========================================
        # 🚀 核心误差发送逻辑 (已修正)
        # ==========================================
        if current_mode in [1, 4, 5]: 
            target_index = 0 

        if current_mode in [1, 2, 4, 5]:
            if len(results) > 0 and target_index < len(results):
                target = results[target_index]
                
                # 🌟 修正 3：平滑计算。
                # 第一次运行或丢失目标太久时，直接赋予初始值，防止从 (0,0) 开始拉
                if smooth_tx == 0 and smooth_ty == 0:
                    smooth_tx, smooth_ty = target['cx'], target['cy']
                else:
                    smooth_tx = smooth_tx * 0.3 + target['cx'] * 0.7
                    smooth_ty = smooth_ty * 0.3 + target['cy'] * 0.7

                if laser_cx is not None and laser_cy is not None:
                    err_x = smooth_tx - laser_cx
                    err_y = smooth_ty - laser_cy
                    serial_manager.send_gimbal_data(err_x, err_y, state=1)
                else:
                    serial_manager.send_gimbal_data(0, 0, state=0)
            else:
                smooth_tx, smooth_ty = 0, 0 # 丢失目标时清零
                serial_manager.send_gimbal_data(0, 0, state=0)

                if current_mode == 2 and target_index > 0:
                    cv2.putText(display_frame, "MISSION 2 COMPLETED!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        elif current_mode == 3: # 模式3：吃豆人描边
            if len(results) > 0 and target_index < len(results):
                target = results[target_index]
                
                # 如果还没生成吃豆人路径
                if len(trace_points) == 0 and len(target['contour']) > 0:
                    vertices = target['contour']
                    trace_points = []
                    # 在顶点之间插值，每条边分 10 份
                    for i in range(len(vertices)):
                        pt1 = vertices[i][0]
                        pt2 = vertices[(i+1) % len(vertices)][0] 
                        for j in range(10):
                            x = int(pt1[0] + (pt2[0] - pt1[0]) * (j / 10.0))
                            y = int(pt1[1] + (pt2[1] - pt1[1]) * (j / 10.0))
                            trace_points.append((x, y))
                    current_trace_idx = 0

                # 开始吃豆人
                if laser_cx is not None and laser_cy is not None and current_trace_idx < len(trace_points):
                    goal_x, goal_y = trace_points[current_trace_idx]
                    
                    err_x = goal_x - laser_cx
                    err_y = goal_y - laser_cy
                    serial_manager.send_gimbal_data(err_x, err_y, state=1)
                    
                    for p in trace_points: cv2.circle(display_frame, p, 2, (255, 255, 255), -1)
                    
                    # 距离小于 15 像素就算吃到
                    if (err_x**2 + err_y**2) ** 0.5 < 15.0:
                        current_trace_idx += 1 
                        if current_trace_idx >= len(trace_points):
                            trace_points = [] 
                            target_index += 1
                else:
                    serial_manager.send_gimbal_data(0, 0, state=0)
            else:
                serial_manager.send_gimbal_data(0, 0, state=0)

        # 屏幕显示与按键切换
        cv2.putText(display_frame, f"MODE: {current_mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
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