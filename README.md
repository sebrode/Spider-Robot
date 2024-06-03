# Spider-Robot

This is the bachelor project made by Gustav Brandsen and Sebastian Rode. This repository contains the code for the modified RAINBOW API, developed at DIKU, along with the Python code that makes a four-legged spider robot move.

## How to run:

### Generating IK matrices
Navigate to ```RAINBOW/python/bachelor.py``` and modify the position matrices corresponding to each leg in order to generate the necessary matrices for your desired movement.

### Movement on the Spider Robot

Connect to the same network as the raspberry pi. SSH into the pi and Navigate to ```/home/pi5/spiderrobot/webservo.py``` and run the file. Go to your web browser and navigate to the designated port, i.e: ```localhost:8000```

From here, you can control the individual legs and copy your generated matrices into the input fields to make it walk as intended.
