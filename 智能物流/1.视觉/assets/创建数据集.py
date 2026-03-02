import cv2
import numpy as np
import random
import matplotlib.pyplot as plt

# ================= 🎛️ 参数配置 (按需调整) =================
CONFIG = {
    "img_size": 640,
    "circle_count": (1, 5),      # 数量减少，保持简洁
    "radius_range": (60, 140),   # 半径调大 (直径 120-280，在640画面里很大了)
    
    # 🎨 颜色波动设置 (BGR格式)
    # 我们会动态生成颜色，而不是查表
    "color_variance": 40,        # 颜色的波动范围 (越大颜色越杂)
    
    # 📐 形状微调
    "deformation": {
        "min_ratio": 0.90,       # 0.9 - 1.0 极轻微的椭圆，接近正圆
        "max_ratio": 1.0,
    },

    # 💡 光照与噪点 (融合版)
    "lighting_strength": 0.6,    # 光照强度
    "noise_level": 15,           # 颗粒噪点强度 (模拟第一版效果)
    
    # ✏️ 描边设置
    "outline": {
        "prob": 0.7,             # 70%的概率有描边
        "thickness": (1, 3),     # 描边很细，不粗
        "color_darkness": 0.6    # 描边颜色比本体颜色暗多少 (0.6倍)
    }
}

# ================= 🎨 核心功能函数 =================

def get_varied_color(class_id):
    """
    生成带有随机波动的红/绿/蓝颜色
    class_id: 0=红, 1=绿, 2=蓝 (YOLO用)
    返回: (B, G, R)
    """
    base_val = random.randint(180, 255) # 主通道要亮
    noise_val = random.randint(0, 60)   # 其他通道要暗
    
    # 基础颜色构造
    if class_id == 0:   # Red (R通道高)
        color = [noise_val, noise_val, base_val]
    elif class_id == 1: # Green (G通道高)
        color = [noise_val, base_val, noise_val]
    else:               # Blue (B通道高)
        color = [base_val, noise_val, noise_val]
        
    # 加入随机波动 (让颜色不那么死板)
    for i in range(3):
        offset = random.randint(-CONFIG["color_variance"], CONFIG["color_variance"])
        color[i] = np.clip(color[i] + offset, 0, 255)
        
    return tuple(map(int, color))

def apply_fusion_effects(image):
    """
    融合：第一版的噪点 + 第二版的光照
    """
    h, w, c = image.shape
    img_float = image.astype(np.float32)

    # 1. 光照效果 (梯度光)
    mask = np.zeros((h, w), dtype=np.float32)
    # 随机生成光照方向
    direction = random.choice(['h', 'v', 'd'])
    if direction == 'h': 
        mask[:] = np.linspace(0.6, 1.2, w)
    elif direction == 'v': 
        mask = np.linspace(0.6, 1.2, h)[:, None] * np.ones((1, w))
    else: 
        X, Y = np.meshgrid(np.arange(w), np.arange(h))
        mask = (X + Y) / (w + h) + 0.4
    
    # 叠加光照
    mask = np.dstack([mask] * 3)
    img_float = img_float * mask
    
    # 2. 颗粒噪点 (Sensor Noise - 第一版风格)
    # 生成高斯噪声
    noise = np.random.normal(0, CONFIG["noise_level"], image.shape)
    img_float = img_float + noise
    
    return np.clip(img_float, 0, 255).astype(np.uint8)

def check_overlap(new_circle, objects, padding=10):
    """防重叠检测"""
    nx, ny, na, nb = new_circle
    # 简化计算：用最大半径作为碰撞体积
    nr = max(na, nb)
    
    for ox, oy, oa, ob, _, _ in objects:
        or_rad = max(oa, ob)
        dist = np.sqrt((nx-ox)**2 + (ny-oy)**2)
        # 判定距离是否小于两者半径之和
        if dist < (nr + or_rad + padding):
            return True
    return False

def generate_clean_sample():
    # 1. 背景：灰度 + 基础纹理 (不干扰主体)
    bg_color = random.randint(60, 180)
    img = np.full((CONFIG["img_size"], CONFIG["img_size"], 3), bg_color, dtype=np.uint8)
    
    # 2. 生成物体 (确保不被遮挡，所以在光照前绘制)
    objects = [] # (x, y, major, minor, class_id, color)
    
    count = random.randint(*CONFIG["circle_count"])
    
    for _ in range(count):
        for _ in range(50): # 尝试多次防止重叠
            # 随机尺寸
            major = random.randint(*CONFIG["radius_range"])
            # 随机轻微形变
            ratio = random.uniform(CONFIG["deformation"]["min_ratio"], CONFIG["deformation"]["max_ratio"])
            minor = int(major * ratio)
            
            # 随机位置 (确保完整在画面内)
            margin = major + 5
            x = random.randint(margin, CONFIG["img_size"] - margin)
            y = random.randint(margin, CONFIG["img_size"] - margin)
            
            # 检查重叠
            if not check_overlap((x, y, major, minor), objects):
                class_id = random.randint(0, 2) # 0:R, 1:G, 2:B
                color = get_varied_color(class_id)
                angle = random.randint(0, 360)
                
                # --- 绘制逻辑 ---
                # 1. 绘制实心圆/椭圆
                cv2.ellipse(img, (x, y), (major, minor), angle, 0, 360, color, -1)
                
                # 2. 随机描边 (Outline)
                if random.random() < CONFIG["outline"]["prob"]:
                    # 描边颜色比本体深一点
                    outline_color = [int(c * CONFIG["outline"]["color_darkness"]) for c in color]
                    thickness = random.randint(*CONFIG["outline"]["thickness"])
                    cv2.ellipse(img, (x, y), (major, minor), angle, 0, 360, outline_color, thickness)
                
                objects.append((x, y, major, minor, class_id, color))
                break
    
    # 3. 最后应用环境特效 (光照 + 噪点)
    # 这样光照会同时作用于背景和物体，看起来更融合
    img = apply_fusion_effects(img)
    
    return img

def preview():
    plt.figure(figsize=(15, 10))
    for i in range(6):
        img = generate_clean_sample()
        # 预览时转回 RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        plt.subplot(2, 3, i + 1)
        plt.imshow(img_rgb)
        plt.title(f"Sample {i+1}")
        plt.axis('off')
        
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    preview()