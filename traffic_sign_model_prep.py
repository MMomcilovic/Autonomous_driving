from ultralytics import YOLO

if __name__ == '__main__':
    model = YOLO("yolov8n.pt")

    model.train(data="./datasets/data.yaml", epochs=7, imgsz=640, device=0)

    metrics = model.val()

    print(metrics)

    model.export()