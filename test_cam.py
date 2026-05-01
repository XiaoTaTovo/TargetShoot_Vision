import cv2

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # 树莓派上可能不需要 cv2.CAP_DSHOW

# 关闭自动曝光 (极其重要！)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # 0.25 通常代表手动曝光

print("按 'a' 增加曝光，按 'd' 减小曝光，按 'q' 退出")

# 获取初始曝光值
exp = cap.get(cv2.CAP_PROP_EXPOSURE)
print(f"初始曝光值: {exp}")

while True:
    ret, frame = cap.read()
    if not ret: break
    
    cv2.putText(frame, f"Exposure: {exp:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Camera", frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break
    elif key == ord('a'): 
        exp += 1.0  # 某些摄像头可能需要 +100 或 +0.1，你自己试试
        cap.set(cv2.CAP_PROP_EXPOSURE, exp)
        print(f"当前曝光: {cap.get(cv2.CAP_PROP_EXPOSURE)}")
    elif key == ord('d'): 
        exp -= 1.0
        cap.set(cv2.CAP_PROP_EXPOSURE, exp)
        print(f"当前曝光: {cap.get(cv2.CAP_PROP_EXPOSURE)}")

cap.release()
cv2.destroyAllWindows()