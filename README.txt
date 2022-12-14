Get started:
1. Install PyTorch with Anaconda

2. Clone git repository:
$ git clone https://github.com/saibot1997/LaserPointerRobot_yolov5.git

3. Enter the repository root directory
$ cd LaserPointerRobot_yolov5

4. Install the required packages from your cloned repository root directory
$ pip install -r requirements.txt

5. Test Yolov5 with the following code:
import torch

# Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

# Images
dir = 'https://github.com/ultralytics/yolov5/raw/master/data/images/'
imgs = [dir + f for f in ('zidane.jpg', 'bus.jpg')]  # batch of images

# Inference
results = model(imgs)
results.print()  # or .show(), .save()

6. Install the control software on the robot control board (dir: Laser_pointer_V4)

7. The file 'LaserPointer_yolov5s.py' contains the code to control the Robot based on the detected objects.
You can find further information in the comments.
