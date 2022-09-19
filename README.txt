Get started:

1. Clone git repository:
$ git clone https://github.com/saibot1997/LaserPointerRobot_yolov5.git

2. Enter the repository root directory
$ cd LaserPointerRobot_yolov5

3. Install the required packages from your cloned repository root directory
$ pip install -r requirements.txt

4. Test Yolov5 with the following code:
import torch

# Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

# Images
dir = 'https://github.com/ultralytics/yolov5/raw/master/data/images/'
imgs = [dir + f for f in ('zidane.jpg', 'bus.jpg')]  # batch of images

# Inference
results = model(imgs)
results.print()  # or .show(), .save()

5. The file 'LaserPointer_yolov5s.py' contains the code to control the Robot based on the detected objects.
You can find further information in the comments.
