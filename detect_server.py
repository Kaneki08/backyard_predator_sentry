from flask import Flask, request, jsonify
from PIL import Image
from ml.detect import detect

app = Flask(__name__)

@app.route("/detect", methods=["POST"])
def detect_endpoint():
    image = Image.open(request.files["image"].stream)
    detections = detect(image)
    return jsonify(detections)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=4000)