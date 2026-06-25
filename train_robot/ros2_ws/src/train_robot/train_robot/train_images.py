import numpy as np
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ultralytics import YOLO

def add_yolov8_obb_detections(
    yolo_model,
    image,
    conf_threshold=0.5, 
    iou_threshold=0.7,
    line_thickness=2,
    font_scale=0.6,
    color=(0, 255, 0)
):
    # 确保输入图像是BGR格式
    if len(image.shape) != 3 or image.shape[2] != 3:
        raise ValueError("输入图像必须是BGR格式的3通道图像")

    detected_image = image.copy()
    results = yolo_model.predict(
        detected_image, 
        imgsz=640, 
        conf=conf_threshold, 
        iou=iou_threshold
    )
    
    # 绘制检测结果
    for result in results:
        if hasattr(result, 'obb'):
            for obb in result.obb:
                # 获取旋转框的四个角点坐标
                rbox = obb.xyxyxyxy.cpu().numpy().reshape(4, 2).astype(int)
                
                # 绘制旋转矩形
                cv2.polylines(detected_image, [rbox], isClosed=True, color=color, thickness=line_thickness)
                
                # 添加标签 (类别+置信度)
                label = f"{result.names[int(obb.cls)]} {float(obb.conf):.2f}"
                
                # 计算文字位置(左上角点上方)
                text_org = (rbox[0][0], rbox[0][1] - 10)
                
                # 确保文字不会超出图像边界
                if text_org[1] < 20:
                    text_org = (text_org[0], rbox[0][1] + 20)
                
                # 绘制文字背景(增强可读性)
                (text_width, text_height), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
                
                cv2.rectangle(
                    detected_image,
                    (text_org[0], text_org[1] - text_height - 5),
                    (text_org[0] + text_width, text_org[1] + 5),
                    color, -1
                )
                
                # 绘制文字
                cv2.putText(
                    detected_image, 
                    label, 
                    text_org,
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    font_scale, 
                    (0, 0, 0),  # 黑色文字
                    line_thickness
                )
    
    return detected_image

def train_visualize_images(
    picture_rgb_down,
    picture_dp_down,
    picture_rgb_hand,
    picture_dp_hand,
    picture_rgb_up,
    picture_dp_up,
    yolo_model
):
    """将9张图片（3个相机的RGB+深度）按Up/Down/Hand顺序合并显示在3x3网格中"""
    # 检查所有图像数据是否已接收
    if (picture_rgb_down is None or picture_dp_down is None or 
        picture_rgb_hand is None or picture_dp_hand is None or 
        picture_rgb_up is None or picture_dp_up is None):
        return

    # ========== 预处理所有图像 ==========
    # 统一尺寸（以down相机RGB为基准）
    target_h, target_w = picture_rgb_down.shape[:2]
    
    # 定义图像列表和标题（按Up/Down/Hand分组）
    use_yolo = True
    if use_yolo:
        images = [
            # 第一排：Up相机
            add_yolov8_obb_detections(yolo_model, picture_rgb_up),
            cv2.cvtColor(picture_dp_up, cv2.COLOR_GRAY2BGR),
            # 第二排：Down相机
            add_yolov8_obb_detections(yolo_model, picture_rgb_down),
            cv2.cvtColor(picture_dp_down, cv2.COLOR_GRAY2BGR),
            # 第三排：Hand相机
            add_yolov8_obb_detections(yolo_model, picture_rgb_hand),
            cv2.cvtColor(picture_dp_hand, cv2.COLOR_GRAY2BGR)
        ]
    else:
        images = [
            # 第一排：Up相机
            picture_rgb_up,
            cv2.cvtColor(picture_dp_up, cv2.COLOR_GRAY2BGR),
            # 第二排：Down相机
            picture_rgb_down,
            cv2.cvtColor(picture_dp_down, cv2.COLOR_GRAY2BGR),
            # 第三排：Hand相机
            picture_rgb_hand,
            cv2.cvtColor(picture_dp_hand, cv2.COLOR_GRAY2BGR)
        ]

    titles = [
        "Up RGB", "Up Depth",
        "Down RGB", "Down Depth",
        "Hand RGB", "Hand Depth"
    ]

    # ========== 创建3x3网格画布 ==========
    # 统一调整图像尺寸
    resized_imgs = [cv2.resize(img, (target_w, target_h)) for img in images]
    
    # 计算画布尺寸（3行3列，含间隔）
    border = 15  # 增大间隔像素
    canvas_h = target_h * 3 + border * 2
    canvas_w = target_w * 2 + border * 1
    canvas = np.full((canvas_h, canvas_w, 3), 240, dtype=np.uint8)  # 浅灰色背景

    # 填充网格（按新顺序）
    for i in range(6):
        row, col = i // 2, i % 2
        y = row * (target_h + border)
        x = col * (target_w + border)
        canvas[y:y+target_h, x:x+target_w] = resized_imgs[i]
        
        # 添加标题（增大字体，白色背景框增强可读性）
        cv2.putText(canvas, titles[i], (x+5, y+25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # ========== 显示窗口 ==========
    # 计算合适的窗口大小
    max_width = 1600  # 设置最大宽度
    max_height = 900  # 设置最大高度
    
    # 计算缩放比例
    scale_w = max_width / canvas_w
    scale_h = max_height / canvas_h
    scale = min(scale_w, scale_h, 1.0)
    display_w = int(canvas_w * scale)
    display_h = int(canvas_h * scale)
    cv2.namedWindow("All Cameras (Up/Down/Hand)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("All Cameras (Up/Down/Hand)", display_w, display_h)
    cv2.imshow("All Cameras (Up/Down/Hand)", canvas)
    cv2.waitKey(1)

