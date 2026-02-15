import cv2
import numpy as np
import time
import serial
import threading
import queue
import sys
from hobot_dnn import pyeasy_dnn

# ================= 🔧 1. 全局配置区 =================

# --- 模型路径 ---
MODEL_BLOCK_PATH = '/home/pi/yolov5_model/yolov5_x55.bin'    # 4分类 (找物块)
MODEL_OBSTACLE_PATH = '/home/pi/yolov5_model/yolov5_x58.bin' # 1分类 (避障)
INPUT_SIZE = (640, 640)

# --- 目标颜色设置 ---
TARGET_CLASS_ID = 1  # 0:blue, 1:green, 2:pink, 3:red

# --- 串口与电机 ---
SERIAL_PORT = '/dev/ttyUSB0'
LEFT_MOTOR_INVERT = 1
RIGHT_MOTOR_INVERT = -1
MIN_MOTOR_LIMIT = 10

# --- 速度参数 (已调优) ---
# 1. 前往物块区
SPEED_TO_BLOCK_BASE = 14
SPEED_TO_BLOCK_MAX = 16

# 2. 找物块 & 对准
SPEED_SEARCH_SPIN = 14      # 丢失目标时的旋转速度
SPEED_ALIGN_SPIN = 10       # 🎯 对准时的旋转速度 (低速防过冲)
SPEED_SEARCH_APPROACH = 10  # 🎯 对准后的直线靠近速度
SPEED_BLIND_APPROACH = 12   # 🎯 最后的盲走冲刺速度

# 3. 回程速度
SPEED_CARRY_FAST_BASE = 18
SPEED_CARRY_FAST_MAX = 20
SPEED_CARRY_SLOW_BASE = 15
SPEED_CARRY_SLOW_MAX = 17.5

# 4. 掉线找回速度 (新功能)
SPEED_BACK_FIND_LINE = 13   # 后退寻线速度
SPEED_SPIN_FIND_LINE = 14   # 旋转寻线速度

# --- 计时器参数 ---
TIME_CARRY_FAST_DURATION = 15.0  # 高速回程持续时间
TIME_BLIND_FORWARD = 0.8         # 🎯 盲走补齐距离的时间 (约7cm)

# --- 机械臂角度 ---
SERVO_GRIPPER = 1
SERVO_ARM = 2
ANGLES = {
    "grip_open": 0, "grip_close": 90, 
    "arm_ground": 135, "arm_hold": 0  
}

# --- 避障动作参数 ---
OBSTACLE_FRAME_THRESHOLD = 1
TRIGGER_Y_THRESHOLD = 270
AVOID_TURN_SPEED = 15   
AVOID_DRIVE_SPEED = 17.5 
AVOID_BACK_SPEED = 13   
T_BACK = 0.4; T_TURN_1 = 0.63; T_OUT = 1.3; T_TURN_2 = 0.578
T_PASS = 1.3; T_TURN_3 = 0.4; T_RETURN = 0.5; T_TURN_4 = 0.5

# --- PID & 视觉阈值 ---
KP = 0.125
KP_SIGN = 1
ROI_CROP_PERCENT = 0.11
ROI_VERTICAL_DIVIDER = 4
CROSSROAD_WIDTH_RATIO = 0.80

# 红线阈值
LOWER_RED1 = np.array([0, 80, 80]); UPPER_RED1 = np.array([4, 255, 255])
LOWER_RED2 = np.array([175, 80, 80]); UPPER_RED2 = np.array([180, 255, 255])
RED_STOP_AREA = 1500

# 抓取逻辑阈值
BLOCK_GRAB_AREA = 80000  # 面积达到此值开始盲走
CENTER_TOLERANCE = 70    # 对准容差 (左右70像素)
KP_ALIGNMENT = 0.035     # 对准时的转向灵敏度

# ================= 🧵 2. 驱动与工具类 =================

class RobotDriver:
    def __init__(self, port):
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(2); print("✅ 串口连接成功")
        except Exception as e: print(f"❌ 串口失败: {e}"); sys.exit(1)
    def send(self, cmd): self.ser.write((cmd + '\r\n').encode())
    def move(self, left, right):
        l = int(np.clip(left, -100, 100)); r = int(np.clip(right, -100, 100))
        self.send(f"M1:{l}"); self.send(f"M2:{r}")
    def servo(self, id, angle):
        target = int(np.clip(angle, 0, 180)); self.send(f"S{id}:{target}")
    def stop(self): self.move(0, 0)
    def close(self): self.stop(); self.ser.close()

class ArmController:
    def __init__(self, driver): self.bot = driver
    def grab_and_hold(self):
        print("🦾 [机械臂] 停车抓取 -> 装车")
        self.bot.servo(SERVO_GRIPPER, ANGLES["grip_open"]); time.sleep(0.5)
        self.bot.servo(SERVO_ARM, ANGLES["arm_ground"]); time.sleep(1.2)
        self.bot.servo(SERVO_GRIPPER, ANGLES["grip_close"]); time.sleep(0.8)
        self.bot.servo(SERVO_ARM, ANGLES["arm_hold"]); time.sleep(1.0)
        print("🦾 抓取完成")
    def finish_drop(self):
        print("🦾 [机械臂] 终点卸货")
        self.bot.servo(SERVO_ARM, ANGLES["arm_ground"]); time.sleep(1.0)
        self.bot.servo(SERVO_GRIPPER, ANGLES["grip_open"]); time.sleep(0.5)
        self.bot.servo(SERVO_ARM, ANGLES["arm_hold"])

class CameraReader:
    def __init__(self, camera_id):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(3, 640); self.cap.set(4, 480)
        self.q = queue.Queue(maxsize=1); self.running = True
        self.thread = threading.Thread(target=self._reader); self.thread.daemon = True; self.thread.start()
    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret: break
            if not self.q.empty(): self.q.get_nowait()
            self.q.put(frame)
    def read(self): return self.q.get()
    def clear_buffer(self):
        while not self.q.empty(): self.q.get_nowait()
    def release(self): self.running = False; self.thread.join(); self.cap.release()

STRIDES = [8, 16, 32]
ANCHORS = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
def sigmoid(x): return 1 / (1 + np.exp(-x))
def preprocess_nv12(img, input_size):
    img_resized = cv2.resize(img, input_size); h, w = img_resized.shape[:2]
    yuv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2YUV_I420); uv_h = h // 2
    y = yuv[:h, :]; u = yuv[h:h + uv_h // 2, :].reshape(uv_h // 2, w // 2 * 2); v = yuv[h + uv_h // 2:, :].reshape(uv_h // 2, w // 2 * 2)
    uv = np.zeros((uv_h, w), dtype=np.uint8); uv[:, 0::2] = u.reshape(uv_h, w // 2); uv[:, 1::2] = v.reshape(uv_h, w // 2)
    return np.concatenate((y, uv), axis=0)
def nms(boxes, scores, iou_thres):
    if len(boxes) == 0: return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1); order = scores.argsort()[::-1]; keep = []
    while order.size > 0:
        i = order[0]; keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]]); yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]]); yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1); h = np.maximum(0.0, yy2 - yy1); inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= iou_thres)[0]; order = order[inds + 1]
    return keep
def postprocess_decode(outputs, conf_thres, nms_thres, num_classes):
    all_boxes, all_scores, all_class_ids = [], [], []
    for i, output in enumerate(outputs):
        data = output.buffer
        if len(data.shape) == 4: data = data.transpose(0, 2, 3, 1)
        _, h, w, _ = data.shape
        stride = STRIDES[i]; anchor_grid = np.array(ANCHORS[i]).reshape(3, 2)
        try: pred = data.reshape(1, h, w, 3, 5 + num_classes)
        except: continue
        pred = sigmoid(pred)
        mask = pred[..., 4] > conf_thres
        if not np.any(mask): continue
        nz = np.nonzero(mask)
        grid_x = nz[2]; grid_y = nz[1]; anchor_idx = nz[3]
        row = pred[0, grid_y, grid_x, anchor_idx]
        if num_classes > 1:
            cls_scores = row[:, 5:]; class_ids = np.argmax(cls_scores, axis=1); scores = row[:, 4] * np.max(cls_scores, axis=1)
        else:
            class_ids = np.zeros(len(row), dtype=int); scores = row[:, 4] * row[:, 5]
        valid = scores > conf_thres
        row = row[valid]; scores = scores[valid]; class_ids = class_ids[valid]
        pred_x = (row[:, 0]*2 - 0.5 + grid_x[valid])*stride; pred_y = (row[:, 1]*2 - 0.5 + grid_y[valid])*stride
        pred_w = (row[:, 2]*2)**2 * anchor_grid[anchor_idx[valid], 0]; pred_h = (row[:, 3]*2)**2 * anchor_grid[anchor_idx[valid], 1]
        x1 = pred_x - pred_w/2; y1 = pred_y - pred_h/2; x2 = pred_x + pred_w/2; y2 = pred_y + pred_h/2
        all_boxes.append(np.stack([x1, y1, x2, y2], axis=1)); all_scores.append(scores); all_class_ids.append(class_ids)
    if not all_boxes: return []
    all_boxes = np.concatenate(all_boxes); all_scores = np.concatenate(all_scores); all_class_ids = np.concatenate(all_class_ids)
    keep = nms(all_boxes, all_scores, nms_thres)
    return [[*all_boxes[k], all_scores[k], all_class_ids[k]] for k in keep]
def fix_dead_zone(speed):
    if speed == 0: return 0
    if 0 < abs(speed) < MIN_MOTOR_LIMIT: return MIN_MOTOR_LIMIT if speed > 0 else -MIN_MOTOR_LIMIT
    return speed
def perform_crude_avoidance(bot, cam_reader):
    print("🚧 [避障] 执行中...")
    def ms(l, r, t): bot.move(l, r); time.sleep(t); bot.move(0, 0); time.sleep(0.15)
    ms(int(-AVOID_BACK_SPEED*LEFT_MOTOR_INVERT), int(-AVOID_BACK_SPEED*RIGHT_MOTOR_INVERT), T_BACK)
    ms(int(-AVOID_TURN_SPEED*LEFT_MOTOR_INVERT), int(AVOID_TURN_SPEED*RIGHT_MOTOR_INVERT), T_TURN_1)
    ms(int(AVOID_DRIVE_SPEED*LEFT_MOTOR_INVERT), int(AVOID_DRIVE_SPEED*RIGHT_MOTOR_INVERT), T_OUT)
    ms(int(AVOID_TURN_SPEED*LEFT_MOTOR_INVERT), int(-AVOID_TURN_SPEED*RIGHT_MOTOR_INVERT), T_TURN_2)
    ms(int(AVOID_DRIVE_SPEED*LEFT_MOTOR_INVERT), int(AVOID_DRIVE_SPEED*RIGHT_MOTOR_INVERT), T_PASS)
    ms(int(AVOID_TURN_SPEED*LEFT_MOTOR_INVERT), int(-AVOID_TURN_SPEED*RIGHT_MOTOR_INVERT), T_TURN_3)
    ms(int(AVOID_DRIVE_SPEED*LEFT_MOTOR_INVERT), int(AVOID_DRIVE_SPEED*RIGHT_MOTOR_INVERT), T_RETURN)
    ms(int(-AVOID_TURN_SPEED*LEFT_MOTOR_INVERT), int(AVOID_TURN_SPEED*RIGHT_MOTOR_INVERT), T_TURN_4)
    cam_reader.clear_buffer(); time.sleep(0.1)

# ================= 🚀 5. 主程序逻辑 =================

def main():
    bot = RobotDriver(SERIAL_PORT)
    arm = ArmController(bot)
    
    try:
        print("📥 加载模型..."); model_block = pyeasy_dnn.load(MODEL_BLOCK_PATH); model_obstacle = pyeasy_dnn.load(MODEL_OBSTACLE_PATH)
    except Exception as e: print(f"❌ {e}"); return

    cam = CameraReader(0); time.sleep(1.0)
    current_state = 'GO_TO_BLOCK'
    carry_start_time = 0; obstacle_counter = 0; miss_target_count = 0
    lost_line_timer = 0 # 寻线计时器
    kernel = np.ones((5,5), np.uint8)
    
    print(f"🚀 系统启动 | 初始状态: {current_state}")

    try:
        while True:
            frame = cam.read()
            h, w = frame.shape[:2]

            # ================= [阶段1] 前往物块区 =================
            if current_state == 'GO_TO_BLOCK':
                roi_red = frame[int(h/2):h, 0:w]
                hsv = cv2.cvtColor(roi_red, cv2.COLOR_BGR2HSV)
                mask = cv2.bitwise_or(cv2.inRange(hsv, LOWER_RED1, UPPER_RED1), cv2.inRange(hsv, LOWER_RED2, UPPER_RED2))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                is_red_line = False
                if len(cnts) > 0:
                    max_r = max(cnts, key=cv2.contourArea)
                    if cv2.contourArea(max_r) > RED_STOP_AREA:
                        _, _, wr, hr = cv2.boundingRect(max_r)
                        if wr / hr > 2.5: is_red_line = True

                if is_red_line:
                    print("🛑 检测到红线 -> 切换到抓取模式")
                    bot.stop(); time.sleep(0.5)
                    bot.move(15 * LEFT_MOTOR_INVERT, 15 * RIGHT_MOTOR_INVERT)
                    time.sleep(0.8); bot.stop()
                    current_state = 'SEARCH_AND_GRAB'
                    cam.clear_buffer()
                    continue

                nv12 = preprocess_nv12(frame, INPUT_SIZE)
                outputs = model_obstacle[0].forward(nv12)
                dets = postprocess_decode(outputs, 0.70, 0.45, 1)
                trigger_avoid = False
                for d in dets:
                    if d[3] > TRIGGER_Y_THRESHOLD: trigger_avoid = True; break
                
                if trigger_avoid: obstacle_counter += 1
                else: obstacle_counter = 0

                if obstacle_counter >= OBSTACLE_FRAME_THRESHOLD:
                    bot.stop(); perform_crude_avoidance(bot, cam); obstacle_counter = 0; continue

                # PID 巡线
                roi_h = int(h / ROI_VERTICAL_DIVIDER); roi = frame[h-roi_h:h, 0:w]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY); blur = cv2.GaussianBlur(gray, (5, 5), 0)
                _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
                contours, _ = cv2.findContours(clean, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) > 0:
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > 500:
                        x, y, rect_w, rect_h = cv2.boundingRect(c)
                        if rect_w > (w - 2 * ROI_CROP_PERCENT * w) * CROSSROAD_WIDTH_RATIO: error = 0
                        else:
                            M = cv2.moments(c)
                            if M["m00"] != 0: cx = int(M["m10"] / M["m00"]); error = cx - ((w - 2 * int(w*ROI_CROP_PERCENT)) // 2)
                            else: error = 0
                        turn = error * KP * KP_SIGN
                        sl = fix_dead_zone(int(SPEED_TO_BLOCK_BASE + turn) * LEFT_MOTOR_INVERT)
                        sr = fix_dead_zone(int(SPEED_TO_BLOCK_BASE - turn) * RIGHT_MOTOR_INVERT)
                        bot.move(int(np.clip(sl, -SPEED_TO_BLOCK_MAX, SPEED_TO_BLOCK_MAX)), int(np.clip(sr, -SPEED_TO_BLOCK_MAX, SPEED_TO_BLOCK_MAX)))
                    else: bot.move(0, 0)
                else: bot.move(0, 0)

            # ================= [阶段2] 搜索并夹取 (🎯 柔和对准 + 盲走) =================
            elif current_state == 'SEARCH_AND_GRAB':
                nv12 = preprocess_nv12(frame, INPUT_SIZE)
                outputs = model_block[0].forward(nv12)
                detections = postprocess_decode(outputs, 0.70, 0.45, 4) 
                
                target_found = False
                if len(detections) > 0:
                    best_det = None
                    max_area = 0
                    for det in detections:
                        if int(det[5]) == TARGET_CLASS_ID:
                            box_w = det[2] - det[0]; box_h = det[3] - det[1]
                            area = box_w * box_h
                            if area > max_area:
                                max_area = area
                                best_det = det
                    
                    if best_det is not None:
                        target_found = True
                        miss_target_count = 0
                        cx = int((best_det[0] + best_det[2]) / 2)
                        cv2.rectangle(frame, (int(best_det[0]), int(best_det[1])), (int(best_det[2]), int(best_det[3])), (0,255,0), 2)
                        err = cx - 320 
                        
                        # [Step 1] 抓取判断：面积达标 -> 盲走 -> 抓
                        if max_area > BLOCK_GRAB_AREA:
                            if abs(err) < CENTER_TOLERANCE:
                                print(f"📦 视觉确认 (面积:{max_area:.0f}) -> 盲走补距")
                                bot.move(int(SPEED_BLIND_APPROACH * LEFT_MOTOR_INVERT), int(SPEED_BLIND_APPROACH * RIGHT_MOTOR_INVERT))
                                time.sleep(TIME_BLIND_FORWARD) 
                                print("📦 停车抓取")
                                bot.stop(); time.sleep(0.5)
                                arm.grab_and_hold()
                                print("🔄 抓取完成 -> 回程")
                                cam.clear_buffer(); current_state = 'CARRY_FAST'; carry_start_time = time.time()
                            else:
                                print("⚠️ 距离够但未对准 -> 微调")
                                spin_l = int(SPEED_ALIGN_SPIN * LEFT_MOTOR_INVERT) if err > 0 else int(-SPEED_ALIGN_SPIN * LEFT_MOTOR_INVERT)
                                spin_r = int(-SPEED_ALIGN_SPIN * RIGHT_MOTOR_INVERT) if err > 0 else int(SPEED_ALIGN_SPIN * RIGHT_MOTOR_INVERT)
                                bot.move(spin_l, spin_r)

                        # [Step 2] 距离远，角度偏 -> 柔和对准
                        elif abs(err) > CENTER_TOLERANCE:
                            turn_speed = abs(err) * KP_ALIGNMENT
                            turn_speed = max(turn_speed, MIN_MOTOR_LIMIT) 
                            turn_speed = min(turn_speed, SPEED_ALIGN_SPIN + 5)
                            spin_l = int(turn_speed * LEFT_MOTOR_INVERT) if err > 0 else int(-turn_speed * LEFT_MOTOR_INVERT)
                            spin_r = int(-turn_speed * RIGHT_MOTOR_INVERT) if err > 0 else int(turn_speed * RIGHT_MOTOR_INVERT)
                            bot.move(spin_l, spin_r)
                            
                        # [Step 3] 已对准 -> 直线靠近
                        else:
                            sl = fix_dead_zone(int(SPEED_SEARCH_APPROACH * LEFT_MOTOR_INVERT))
                            sr = fix_dead_zone(int(SPEED_SEARCH_APPROACH * RIGHT_MOTOR_INVERT))
                            bot.move(sl, sr)
                
                if not target_found:
                    miss_target_count += 1
                    if miss_target_count > 5: # 连续丢失5帧才转
                        bot.move(int(SPEED_SEARCH_SPIN * LEFT_MOTOR_INVERT), int(-SPEED_SEARCH_SPIN * RIGHT_MOTOR_INVERT))
                    else:
                        bot.move(0, 0)

            # ================= [阶段3] 回程-高速段 (🆕 自动找线) =================
            elif current_state == 'CARRY_FAST':
                if time.time() - carry_start_time > TIME_CARRY_FAST_DURATION:
                    print("🐢 计时结束 -> 降速避障"); current_state = 'CARRY_SLOW'; continue 

                roi_h = int(h / ROI_VERTICAL_DIVIDER); roi = frame[h-roi_h:h, 0:w]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY); blur = cv2.GaussianBlur(gray, (5, 5), 0)
                _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
                contours, _ = cv2.findContours(clean, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) > 0:
                    lost_line_timer = 0
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > 500:
                        x, y, rect_w, rect_h = cv2.boundingRect(c)
                        if rect_w > (w - 2 * ROI_CROP_PERCENT * w) * CROSSROAD_WIDTH_RATIO: error = 0
                        else:
                            M = cv2.moments(c); 
                            if M["m00"] != 0: cx = int(M["m10"] / M["m00"]); error = cx - ((w - 2 * int(w*ROI_CROP_PERCENT)) // 2)
                            else: error = 0
                        turn = error * KP * KP_SIGN
                        sl = fix_dead_zone(int(SPEED_CARRY_FAST_BASE + turn) * LEFT_MOTOR_INVERT)
                        sr = fix_dead_zone(int(SPEED_CARRY_FAST_BASE - turn) * RIGHT_MOTOR_INVERT)
                        bot.move(int(np.clip(sl, -SPEED_CARRY_FAST_MAX, SPEED_CARRY_FAST_MAX)), int(np.clip(sr, -SPEED_CARRY_FAST_MAX, SPEED_CARRY_FAST_MAX)))
                    else: 
                        lost_line_timer += 1
                else:
                    # 🆕 没看到线 -> 后退/旋转找回
                    lost_line_timer += 1
                    if lost_line_timer < 20: bot.move(0, 0)
                    elif lost_line_timer < 60:
                        bot.move(int(-SPEED_BACK_FIND_LINE * LEFT_MOTOR_INVERT), int(-SPEED_BACK_FIND_LINE * RIGHT_MOTOR_INVERT))
                    else:
                        bot.move(int(SPEED_SPIN_FIND_LINE * LEFT_MOTOR_INVERT), int(-SPEED_SPIN_FIND_LINE * RIGHT_MOTOR_INVERT))

            # ================= [阶段4] 回程-低速段 (🆕 自动找线) =================
            elif current_state == 'CARRY_SLOW':
                nv12 = preprocess_nv12(frame, INPUT_SIZE)
                outputs = model_obstacle[0].forward(nv12)
                dets = postprocess_decode(outputs, 0.70, 0.45, 1)
                trigger_avoid = False
                for d in dets:
                    if d[3] > TRIGGER_Y_THRESHOLD: trigger_avoid = True; break
                if trigger_avoid: obstacle_counter += 1
                else: obstacle_counter = 0
                if obstacle_counter >= OBSTACLE_FRAME_THRESHOLD:
                    bot.stop(); perform_crude_avoidance(bot, cam); obstacle_counter = 0; continue

                roi_red = frame[int(h/2):h, 0:w]
                hsv = cv2.cvtColor(roi_red, cv2.COLOR_BGR2HSV)
                mask = cv2.bitwise_or(cv2.inRange(hsv, LOWER_RED1, UPPER_RED1), cv2.inRange(hsv, LOWER_RED2, UPPER_RED2))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(cnts) > 0:
                    max_r = max(cnts, key=cv2.contourArea)
                    if cv2.contourArea(max_r) > RED_STOP_AREA:
                        _, _, wr, hr = cv2.boundingRect(max_r)
                        if wr / hr > 2.5:
                            print("🛑 终点红线 -> 卸货"); bot.stop(); current_state = 'FINISH'; continue

                roi_h = int(h / ROI_VERTICAL_DIVIDER); roi = frame[h-roi_h:h, 0:w]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY); blur = cv2.GaussianBlur(gray, (5, 5), 0)
                _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
                contours, _ = cv2.findContours(clean, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                if len(contours) > 0:
                    lost_line_timer = 0
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > 500:
                        x, y, rect_w, rect_h = cv2.boundingRect(c)
                        if rect_w > (w - 2 * ROI_CROP_PERCENT * w) * CROSSROAD_WIDTH_RATIO: error = 0
                        else:
                            M = cv2.moments(c); 
                            if M["m00"] != 0: cx = int(M["m10"] / M["m00"]); error = cx - ((w - 2 * int(w*ROI_CROP_PERCENT)) // 2)
                            else: error = 0
                        turn = error * KP * KP_SIGN
                        sl = fix_dead_zone(int(SPEED_CARRY_SLOW_BASE + turn) * LEFT_MOTOR_INVERT)
                        sr = fix_dead_zone(int(SPEED_CARRY_SLOW_BASE - turn) * RIGHT_MOTOR_INVERT)
                        bot.move(int(np.clip(sl, -SPEED_CARRY_SLOW_MAX, SPEED_CARRY_SLOW_MAX)), int(np.clip(sr, -SPEED_CARRY_SLOW_MAX, SPEED_CARRY_SLOW_MAX)))
                    else: lost_line_timer += 1
                else:
                    # 🆕 低速段也加寻线
                    lost_line_timer += 1
                    if lost_line_timer < 20: bot.move(0, 0)
                    elif lost_line_timer < 60:
                        bot.move(int(-SPEED_BACK_FIND_LINE * LEFT_MOTOR_INVERT), int(-SPEED_BACK_FIND_LINE * RIGHT_MOTOR_INVERT))
                    else:
                        bot.move(int(SPEED_SPIN_FIND_LINE * LEFT_MOTOR_INVERT), int(-SPEED_SPIN_FIND_LINE * RIGHT_MOTOR_INVERT))

            # ================= [阶段5] 终点卸货 =================
            elif current_state == 'FINISH':
                time.sleep(0.5); bot.move(18 * LEFT_MOTOR_INVERT, 18 * RIGHT_MOTOR_INVERT)
                time.sleep(1.0); bot.stop()
                arm.finish_drop(); print("🎉 任务完成"); break
            
    except KeyboardInterrupt: print("\n⛔ 中断")
    finally: bot.close(); cam.release(); print("👋 Bye")

if __name__ == "__main__":
    main()