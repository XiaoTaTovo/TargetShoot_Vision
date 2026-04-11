import cv2
from Target.shape_detect import process_shapes
from Serial.communicate import SerialManager 

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
    serial_manager = SerialManager(port='COM3', baudrate=115200)
    
    # 初始模式设为 1 (任务1)
    current_mode = 1 
    # 当前正在打第几个目标索引号
    target_index = 0
    print(f"✅ 系统启动! 当前模式: {current_mode}。按 '1' 或 '2' 切换模式，按 'q' 退出。")

    while True:
        ret, frame = cap.read()
        if not ret: break
            
        # 把当前的 mode 传给识别库
        results, display_frame, thresh = process_shapes(frame, current_mode)
        
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
            
            # 🌟 关键：打的是排序后的第 target_index 个目标！
            target = results[target_index]
            
            err_x = target['cx'] - CENTER_X
            err_y = target['cy'] - CENTER_Y
            
            serial_manager.send_gimbal_data(err_x, err_y, state=1)
            
            cv2.putText(display_frame, f"Target {target_index} | Err X:{err_x} Y:{err_y}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 把当前锁定的目标画一个特殊的红圈，方便观察
            cv2.circle(display_frame, (target['cx'], target['cy']), 15, (0, 0, 255), 3)
            
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