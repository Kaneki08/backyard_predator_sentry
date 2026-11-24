from ultralytics import YOLO
from pathlib import Path


model = YOLO("yolo26n.pt")

def detect(image_path):
    results = model(image_path)
    boxes = results[0].boxes
    detections = []

    for box in boxes:
        corners = box.xyxy[0].tolist()
        corners = [round(n, 2) for n in corners]

        detections.append({
            'label': results[0].names[int(box.cls[0])],
            'confidence': round(float(box.conf[0]), 2),
            'bounding_box': corners
        })
    return detections

if __name__ == "__main__":
    image_path = Path(__file__).resolve().parent.parent / "IMG_3044.jpg"
    print(detect(image_path))