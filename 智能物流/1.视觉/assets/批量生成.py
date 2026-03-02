import cv2
import numpy as np
import os
import random
import shutil
import math

# ================= 🎛️ 数据集配置中心 =================
CONFIG = {
    # 📁 路径与数量
    "save_root": "yolo_dataset",
    "num_train": 1000,           # 训练集数量 (建议 1000+)
    "num_val": 200,              # 验证集数量
    "img_size": 640,             # 图片尺寸 640x640

    # 🎯 目标设置 (大圆、清晰)
    "circle_count": (1, 4),      # 每张图 1-4 个圆
    "radius_range": (50, 130),   # 半径范围 (大尺寸)
    "deformation": (0.9, 1.0),   # 0.9-1.0 轻微椭圆
    "color_variance": 50,        # 颜色波动幅度

    # ⚡ 干扰设置 (复刻第一版的噪点 + 几何干扰)
    "noise": {
        "sensor_grain_level": 25, # 传感器颗粒噪点强度 (0-255)
        
        # 几何干扰 (线、矩形、锯齿)
        "lines_count": (5, 15),
        "rects_count": (3, 8),
        "scratches_count": (3, 8), # 锯齿/划痕
        "interference_alpha": 0.6  # 干扰层的透明度 (越小越透明)
    },

    # 💡 光照设置 (第二版的光照)
    "lighting": {
        "enabled": True,
        "strength": 0.5
    }
}

# 🏷️ 类别映射
CLASSES = ['Red', 'Green', 'Blue']

# ================= 🛠️ 辅助函数 =================

def setup_directories():
    """初始化目录结构"""
    root = CONFIG["save_root"]
    if os.path.exists(root):
        shutil.rmtree(root) # 清空旧数据
    
    # 创建 images/train, labels/train 等目录
    for t in ['train', 'val']:
        os.makedirs(os.path.join(root, 'images', t), exist_ok=True)
        os.makedirs(os.path.join(root, 'labels', t), exist_ok=True)

def get_varied_color(class_id):
    """生成带波动的 RGB 颜色"""
    # 基础色 (BGR)
    bases = {
        0: (20, 20, 240), # Red
        1: (20, 240, 20), # Green
        2: (240, 20, 20)  # Blue
    }
    b, g, r = bases[class_id]
    var = CONFIG["color_variance"]
    
    # 加入波动并截断到 0-255
    b = np.clip(b + random.randint(-var, var), 0, 255)
    g = np.clip(g + random.randint(-var, var), 0, 255)
    r = np.clip(r + random.randint(-var, var), 0, 255)
    
    return (int(b), int(g), int(r))

def check_overlap(new_circle, objects, padding=10):
    """防重叠检测"""
    nx, ny, na, nb = new_circle
    nr = max(na, nb)
    for ox, oy, oa, ob, _, _ in objects:
        or_rad = max(oa, ob)
        dist = np.sqrt((nx-ox)**2 + (ny-oy)**2)
        if dist < (nr + or_rad + padding):
            return True
    return False

def get_rotated_bbox(cx, cy, a, b, angle_deg):
    """
    计算旋转椭圆的轴对齐外接矩形 (Axis-Aligned Bounding Box)
    用于 YOLO 标签生成 (cx, cy, w, h)
    """
    rad = math.radians(angle_deg)
    sin_a = math.sin(rad)
    cos_a = math.cos(rad)
    
    # 椭圆参数方程推导出的外接矩形半宽/半高
    # Width/2 = sqrt( (a*cos)^2 + (b*sin)^2 )
    half_w = math.sqrt((a * cos_a)**2 + (b * sin_a)**2)
    half_h = math.sqrt((a * sin_a)**2 + (b * cos_a)**2)
    
    return int(half_w * 2), int(half_h * 2)

# ================= 🎨 图像增强核心 =================

def add_geometric_noise(image):
    """添加几何干扰 (线、矩形、锯齿)"""
    h, w, c = image.shape
    overlay = image.copy()
    
    # 1. 随机直线 (模拟电线/杆子)
    for _ in range(random.randint(*CONFIG["noise"]["lines_count"])):
        pt1 = (random.randint(0, w), random.randint(0, h))
        pt2 = (random.randint(0, w), random.randint(0, h))
        color = (random.randint(50, 200), random.randint(50, 200), random.randint(50, 200))
        thickness = random.randint(1, 3)
        cv2.line(overlay, pt1, pt2, color, thickness)

    # 2. 空心矩形 (模拟框框/背景杂物)
    for _ in range(random.randint(*CONFIG["noise"]["rects_count"])):
        pt1 = (random.randint(0, w), random.randint(0, h))
        pt2 = (pt1[0] + random.randint(20, 100), pt1[1] + random.randint(20, 100))
        color = (random.randint(50, 200), random.randint(50, 200), random.randint(50, 200))
        cv2.rectangle(overlay, pt1, pt2, color, random.randint(1, 4)) # 空心

    # 3. 锯齿/划痕 (模拟撕裂干扰)
    for _ in range(random.randint(*CONFIG["noise"]["scratches_count"])):
        start_x, start_y = random.randint(0, w), random.randint(0, h)
        curr_x, curr_y = start_x, start_y
        for _ in range(random.randint(5, 15)): # 一条划痕由多段组成
            next_x = curr_x + random.randint(-10, 10)
            next_y = curr_y + random.randint(-10, 10)
            cv2.line(overlay, (curr_x, curr_y), (next_x, next_y), (200, 200, 200), 1)
            curr_x, curr_y = next_x, next_y

    # 融合干扰层 (保持半透明，不完全遮挡物体)
    alpha = CONFIG["noise"]["interference_alpha"]
    return cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

def apply_final_effects(image):
    """应用光照和传感器底噪"""
    h, w, c = image.shape
    img_float = image.astype(np.float32)

    # 1. 光照梯度
    if CONFIG["lighting"]["enabled"]:
        mask = np.zeros((h, w), dtype=np.float32)
        direction = random.choice(['h', 'v'])
        if direction == 'h': mask[:] = np.linspace(0.6, 1.1, w)
        else: mask = np.linspace(0.6, 1.1, h)[:, None] * np.ones((1, w))
        img_float *= np.dstack([mask] * 3)

    # 2. 传感器颗粒噪点 (Sensor Grain) - 这里的噪点是像素级的
    grain = np.random.normal(0, CONFIG["noise"]["sensor_grain_level"], image.shape)
    img_float += grain

    return np.clip(img_float, 0, 255).astype(np.uint8)

# ================= 🚀 生成主循环 =================

def generate_image_and_label(index, subset):
    # 1. 基础背景
    bg_gray = random.randint(60, 180)
    img = np.full((CONFIG["img_size"], CONFIG["img_size"], 3), bg_gray, dtype=np.uint8)
    
    objects = []
    labels = [] # YOLO labels
    
    # 2. 生成圆形目标
    for _ in range(random.randint(*CONFIG["circle_count"])):
        for _ in range(50): # 尝试 50 次放置
            major = random.randint(*CONFIG["radius_range"])
            ratio = random.uniform(*CONFIG["deformation"])
            minor = int(major * ratio)
            
            # 保证圆在画面内
            margin = major + 5
            x = random.randint(margin, CONFIG["img_size"] - margin)
            y = random.randint(margin, CONFIG["img_size"] - margin)
            
            if not check_overlap((x, y, major, minor), objects):
                class_id = random.randint(0, 2)
                color = get_varied_color(class_id)
                angle = random.randint(0, 360)
                
                # 画圆 (实心)
                cv2.ellipse(img, (x, y), (major, minor), angle, 0, 360, color, -1)
                
                # 随机描边 (增加对比度)
                if random.random() > 0.3:
                    outline_color = [max(0, c - 60) for c in color]
                    cv2.ellipse(img, (x, y), (major, minor), angle, 0, 360, outline_color, random.randint(1, 3))
                
                objects.append((x, y, major, minor, class_id, color))
                
                # --- 计算 YOLO 标签 ---
                # 1. 计算外接矩形宽高
                bbox_w, bbox_h = get_rotated_bbox(x, y, major, minor, angle)
                # 2. 归一化 (x_center, y_center, w, h)
                norm_x = x / CONFIG["img_size"]
                norm_y = y / CONFIG["img_size"]
                norm_w = bbox_w / CONFIG["img_size"]
                norm_h = bbox_h / CONFIG["img_size"]
                
                labels.append(f"{class_id} {norm_x:.6f} {norm_y:.6f} {norm_w:.6f} {norm_h:.6f}")
                break
    
    # 3. 施加几何干扰 (线、框、锯齿)
    img = add_geometric_noise(img)
    
    # 4. 施加环境特效 (光照、噪点)
    img = apply_final_effects(img)
    
    # 5. 保存文件
    filename = f"sim_{subset}_{index:05d}"
    
    img_path = os.path.join(CONFIG["save_root"], 'images', subset, filename + ".jpg")
    txt_path = os.path.join(CONFIG["save_root"], 'labels', subset, filename + ".txt")
    
    cv2.imwrite(img_path, img)
    with open(txt_path, 'w') as f:
        f.write("\n".join(labels))

def create_yaml():
    """生成 dataset.yaml"""
    content = f"""
path: ../{CONFIG["save_root"]} # dataset root dir
train: images/train
val: images/val

# Classes
nc: 3
names: {CLASSES}
    """
    with open(os.path.join(CONFIG["save_root"], "dataset.yaml"), 'w') as f:
        f.write(content.strip())

def main():
    print(f"🚀 开始生成数据集: {CONFIG['save_root']}")
    setup_directories()
    
    # 生成训练集
    print(f"Generating {CONFIG['num_train']} training images...")
    for i in range(CONFIG['num_train']):
        generate_image_and_label(i, 'train')
        if (i+1) % 100 == 0: print(f"  Train: {i+1}/{CONFIG['num_train']}")
        
    # 生成验证集
    print(f"Generating {CONFIG['num_val']} validation images...")
    for i in range(CONFIG['num_val']):
        generate_image_and_label(i, 'val')
        
    # 生成配置文件
    create_yaml()
    
    print("\n✅ 数据集生成完毕！")
    print(f"📂 保存位置: {os.path.abspath(CONFIG['save_root'])}")
    print("💡 包含了 dataset.yaml，可以直接用于 YOLOv5/v8 训练。")
    print("💡 建议：打开几张图片和对应的 txt 文件，用 LabelImg 检查一下标注框是否准确。")

if __name__ == "__main__":
    main()