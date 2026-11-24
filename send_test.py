import requests

with open("IMG_3044.jpg", "rb") as f:
    reply = requests.post("http://localhost:4000/detect", files={"image": f})

print(reply.json())