import cv2
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

# yolo_weights_path = os.path.join(get_package_share_directory("follow_cart"), "yolo",
#                                  "yolov3.weights")
#
# yolo_cfg_path = os.path.join(get_package_share_directory("follow_cart"), "yolo",
#                                  "yolov3.cfg")

yolo_weights_path = "/home/bluevery8/workspace/follow_cart_ws/src/follow_cart/yolo/yolov3.weights"
yolo_cfg_path = "/home/bluevery8/workspace/follow_cart_ws/src/follow_cart/yolo/yolov3.cfg"
coco_names_path = "/home/bluevery8/workspace/follow_cart_ws/src/follow_cart/yolo/coco.names"
image_path = "/home/bluevery8/workspace/follow_cart_ws/src/follow_cart/yolo/convoy.jpg"
# Load Yolo
net = cv2.dnn.readNet(yolo_weights_path, yolo_cfg_path)
classes = []
with open(coco_names_path, "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0]] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Loading image
img = cv2.imread(image_path)
#
# img = cv2.resize(img, None, fx=0.4, fy=0.4)
h, w, channels = img.shape

# Detecting objects
blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
net.setInput(blob)
outs = net.forward(output_layers)

# Showing informations on the screen
class_ids = []
confidences = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5:
            # Object detected
            center_x = int(detection[0] * w)
            center_y = int(detection[1] * h)
            w = int(detection[2] * w)
            h = int(detection[3] * h)
            # Rectangle coordinates
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

font = cv2.FONT_HERSHEY_PLAIN
for i in range(len(boxes)):
    if i in indexes:
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        color = colors[i]
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
cv2.imshow("Image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()