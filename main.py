import cv2
from Target.shape_detect import process_shapes
from Target.shape_detect import detect_laser
from Serial.communicate import SerialManager 

def on_mouse_click(event, x, y, flags, param):
    # 如果检测到鼠标左键按下
    if event == cv2.EVENT_LBUTTONDOWN:
        print("\n" + "="*40)
        print(f"🎯 【激光笔校准】你点击的屏幕坐标是: X={x}, Y={y}")
        print(f"🔧 请修改代码: CENTER_X = {x}, CENTER_Y = {y}")
        print("="*40 + "\n")

def main():
    # 1. 初始化摄像头和串口
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened(): return
    
    # 获取摄像头画面的真实宽高，算出绝对中心点
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    CENTER_X = width // 2
    CENTER_Y = height // 2
    
    # 初始化串口 (注意改成你电脑上的实际 COM 口)
    serial_manager = SerialManager(port='COM5', baudrate=115200)
    
    # 初始模式设为 1 (任务1)
    current_mode = 1 
    # 当前正在打第几个目标索引号
    target_index = 0
    print(f"✅ 系统启动! 当前模式: {current_mode}。按 '1' 或 '2' 切换模式，按 'q' 退出。")
    # ==========================================
    # 🌟 新增：提前创建窗口，并绑定鼠标点击事件！
    # ==========================================
    cv2.namedWindow("Camera View (AI)")
    cv2.setMouseCallback("Camera View (AI)", on_mouse_click)

    while True:
        ret, frame = cap.read()
        if not ret: break
            
        # 把当前的 mode 传给识别库
        results, display_frame, thresh = process_shapes(frame, current_mode)
        
          # 在拿到 results 后，顺便去找激光点在哪！
        laser_cx, laser_cy = detect_laser(frame)
        
        # 在画面上画个绿色的十字，告诉你 AI 看到激光点在哪了
        if laser_cx is not None:
            cv2.drawMarker(display_frame, (laser_cx, laser_cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        
        # 画出画面的绝对中心十字准星 (你的激光笔理想落点)
        cv2.line(display_frame, (CENTER_X - 10, CENTER_Y), (CENTER_X + 10, CENTER_Y), (0, 255, 255), 2)
        cv2.line(display_frame, (CENTER_X, CENTER_Y - 10), (CENTER_X, CENTER_Y + 10), (0, 255, 255), 2)

        if serial_manager.ser is not None and serial_manager.ser.in_waiting > 0:
            try:
                # 读取单片机发来的一行数据
                msg = serial_manager.ser.readline().decode('utf-8').strip()
                print(f"收到单片机指令: {msg}")
                
                # 如果单片机说停留2秒结束了，打下一个！
                if "NEXT_TARGET" in msg:
                    target_index += 1
                    print(f"🎯 切换到下一个目标! 当前进度: {target_index}")
            except:
                pass

       
      

        # ==========================================
        # 🚀 核心逻辑：计算误差并发送串口
        # ==========================================
        if len(results) > 0 and target_index < len(results):
            target = results[target_index]
            
            # 🌟 终极顿悟：不再减去画面中心，而是减去激光点当前的真实位置！
            if laser_cx is not None and laser_cy is not None:
                err_x = target['cx'] - laser_cx
                err_y = target['cy'] - laser_cy
                serial_manager.send_gimbal_data(err_x, err_y, state=1)
                
                cv2.putText(display_frame, f"Target {target_index} | Err X:{err_x} Y:{err_y}", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # 如果没找到激光点（比如被挡住了），先别瞎动
                serial_manager.send_gimbal_data(0, 0, state=0)
            
        else:
            serial_manager.send_gimbal_data(0, 0, state=0)
            # 如果是模式2，且 target_index 已经 >= 图形总数，说明全部打完了！
            if current_mode == 2 and len(results) > 0 and target_index >= len(results):
                 cv2.putText(display_frame, "MISSION COMPLETED!", (CENTER_X-100, CENTER_Y-50), 
                             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        cv2.imshow("Camera View (AI)", display_frame)
        cv2.imshow("Binary View (Debug)", thresh)
        # 键盘监听 (模拟 STM32 下发的切换指令)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('1'): 
            current_mode = 1
            target_index = 0  
            print("🔄 切换到 任务 1: A4靶框跟踪")
        elif key == ord('2'):
            current_mode = 2
            target_index = 0 
            print("🔄 切换到 任务 2: 多目标排序打靶")

    serial_manager.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()