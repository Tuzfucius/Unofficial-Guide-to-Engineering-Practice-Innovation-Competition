from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from libs.Utils import *
import os,sys,gc
import ulab.numpy as np
import image
import time

if __name__=="__main__":
    # 模型路径（使用 detect/to_kmodel.py 转换后的检测模型）
    kmodel_path="/data/RGB_circle_det.kmodel"
    labels = ['Red', 'Green', 'Blue']
    model_input_size=[320,320]

    # 添加显示模式，默认hdmi，可选hdmi/lcd/lt9611/st7701/hx8399,其中hdmi默认置为lt9611，分辨率1920*1080；lcd默认置为st7701，分辨率800*480
    display_mode="lcd"
    rgb888p_size=[640,360]
    confidence_threshold = 0.5
    nms_threshold=0.45

    # 初始化PipeLine
    pl=PipeLine(rgb888p_size=rgb888p_size,display_mode=display_mode)
    pl.create()
    display_size=pl.get_display_size()

    # 初始化YOLOv8实例
    yolo=YOLOv8(task_type="detect",mode="video",kmodel_path=kmodel_path,labels=labels,rgb888p_size=rgb888p_size,model_input_size=model_input_size,display_size=display_size,conf_thresh=confidence_threshold,nms_thresh=nms_threshold,max_boxes_num=50,debug_mode=0)
    yolo.config_preprocess()

    # FPS 计算变量
    fps = 0.0
    frame_count = 0
    fps_update_interval = 10  # 每10帧更新一次FPS显示

    try:
        while True:
            t_start = time.ticks_ms()

            with ScopedTiming("total",1):
                # 逐帧推理
                img=pl.get_frame()
                res=yolo.run(img)
                yolo.draw_result(res,pl.osd_img)

                # 计算并显示FPS
                t_end = time.ticks_ms()
                frame_time = time.ticks_diff(t_end, t_start)
                if frame_time > 0:
                    current_fps = 1000.0 / frame_time
                    # 平滑FPS（指数移动平均）
                    fps = fps * 0.7 + current_fps * 0.3 if fps > 0 else current_fps

                # 在左上角绘制FPS
                fps_text = "FPS: {:.1f}".format(fps)
                pl.osd_img.draw_string_advanced(8, 8, 32, fps_text, color=(255, 255, 0, 255))

                pl.show_image()
                gc.collect()
    except Exception as e:
        print("Error: ", e)
    finally:
        yolo.deinit()
        pl.destroy()