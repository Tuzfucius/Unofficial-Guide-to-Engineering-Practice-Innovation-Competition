from ultralytics import YOLO

# 加载你训练好的模型
model = YOLO("E:\littleCar2\CanMVProject\model\RGB_circle.pt")

# 导出为 ONNX 格式
# imgsz 必须与你训练时一致（如 320 或 640）
# simplify=True 会自动简化算子，提高兼容性
model.export(format="onnx", imgsz=320, simplify=True, opset=12)

# 之后操作：https://www.kendryte.com/k230_canmv/zh/main/zh/example/ai/YOLO%E5%A4%A7%E4%BD%9C%E6%88%98.html



'''
ONNX -> Kmodel 转换步骤（注意：检测模型必须使用 detect 目录下的脚本）：
1. E:\littleCar2\CanMVProject\test_yolov5\detect 这个文件夹下进行cmd，test_yolov5是转换脚本包
2. conda activate low_numpy (依赖包在里面)
3. python to_kmodel.py --target k230 --model E:\littleCar2\CanMVProject\model\RGB_circle.onnx --dataset ../test --input_width 320 --input_height 320 --ptq_option 0
   输出文件：E:\littleCar2\CanMVProject\model\RGB_circle_det.kmodel
   注意：input_width/height 必须与上面 imgsz 一致（320），使用 detect 脚本（mean=[0,0,0] std=[1,1,1]）
'''