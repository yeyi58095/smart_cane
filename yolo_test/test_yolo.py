from ultralytics import YOLO

model = YOLO("yolov8n.pt")  # 先用官方小模型
results = model("motorcyle.jpg", imgsz=640)
results[0].show()  # 看看有沒有跳出視窗
