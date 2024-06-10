from flask import Flask, render_template_string, redirect, request
from time import sleep
from adafruit_servokit import ServoKit
import numpy as np
import math
import time

import board
import adafruit_mpu6050
import threading

mpu = adafruit_mpu6050.MPU6050(board.I2C())
roll = 0.0
pitch = 0.0

start_time = time.time()

kit = ServoKit(channels=16)
leg0 = [0, 1, 2]
leg1 = [4, 5, 6]
leg2 = [8, 9, 10]
leg3 = [12, 13, 14]
legs = [leg0, leg1, leg2, leg3, [leg0, leg1, leg2, leg3]]


def calculate_tilt(ax, ay, az):
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return roll, pitch

def tilt_calculation():
    global roll, pitch
    while True:
        accel_x, accel_y, accel_z = mpu.acceleration
        roll, pitch = calculate_tilt(accel_x, accel_y, accel_z)
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        time.sleep(0.1)

tilt_thread = threading.Thread(target=tilt_calculation)
tilt_thread.daemon = True
tilt_thread.start()

def initialize_servos():
    for i in range(16):
        kit.servo[i].set_pulse_width_range(530, 2470)
        if i in [0, 12]:
            kit.servo[i].angle = 60
        elif i in [4, 8]:
            kit.servo[i].angle = 120
        else:
            kit.servo[i].angle = 90


def SetAngle(num, ang):
    kit.servo[num].angle = ang

def GetAngle(num):
    return kit.servo[num].angle


app = Flask(__name__)
app.config['DEBUG'] = True


TPL = '''
<html>
    <head>
        <title>Web Application to control Servos</title>
        <style>
            body {
                background-color: #e0f7fa; /* Very light blue background */
            }
        </style>
    </head>
    <body>
        <h1>Web Application to Control Servos</h1>
        <form method="POST" action="reset">
            <input type="submit" value="Reset all servos" />
        </form>
        <hr>
        <form method="POST" action="test">
            <select name="selected_leg">
                <option value="0">Grey Leg</option>
                <option value="1">Black Leg</option>
                <option value="2" selected>Red Leg</option>
                <option value="3">Blue Leg</option>
            </select>

            <p>Slider 1 <input type="range" min="1" max="180" name="slider1" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output1">90</output></p>
            <p>Slider 2 <input type="range" min="1" max="180" name="slider2" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output2">90</output></p>
            <p>Slider 3 <input type="range" min="1" max="180" name="slider3" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output3">90</output></p>
            <input type="submit" value="submit" />
        </form>
        <hr>
        <h1>IK Matrix All Legs</h1>
        <form method="POST" action="ik">
            <textarea id="ik_matrix_full" placeholder="Full Matrix" name="ik_matrix_full" rows="10" cols="50"></textarea>
            <br>
            Step time: <input type="number" id="ik_step_time" name="ik_step_time" value="2" min="0" max="10" step="0.1">
            <br>
            Loop count: <input type="number" id="ik_loop_count" name="ik_loop_count" value="1" min="0" max="100" step="1">
            <br>
            <input type="submit" value="submit" />
        </form>
        <hr>
        <h1>IK Matrix Single Leg</h1>
        <form method="POST" action="iktest">
            <textarea id="ik_matrix" name="ik_matrix" rows="10" cols="50"></textarea>
            <input type="number" id="ik_step_time" name="ik_step_time" value="1" min="0" max="10" step="0.1">
            <select name="ik_leg" id="ik_leg">
                <option value="0">Grey Leg</option>
                <option value="1">Black Leg</option>
                <option value="2" selected>Red Leg</option>
                <option value="3">Blue Leg</option>
            </select>
            <input type="submit" value="submit" />
        </form>
        <hr>
        <h1>Movement</h1>
        <form method="POST" action="move">
            <button type="submit" name="direction" value="forward">Forward</button>
        </form>
    </body>
    <script>
        function handleOutputClick(output, sliderName) {
            const input = document.createElement('input');
            input.type = 'number';
            input.value = output.textContent;
            input.addEventListener('change', function() {
                const newValue = parseInt(this.value);
                if (!isNaN(newValue)) {
                    document.querySelector(`input[name="${sliderName}"]`).value = newValue;
                    output.textContent = newValue;
                }
                input.replaceWith(output);
            });
            input.addEventListener('blur', function() {
                input.replaceWith(output);
            });
            output.replaceWith(input);
            input.focus();
        }

        document.getElementById('output1').addEventListener('click', function() {
            handleOutputClick(this, 'slider1');
        });

        document.getElementById('output2').addEventListener('click', function() {
            handleOutputClick(this, 'slider2');
        });

        document.getElementById('output3').addEventListener('click', function() {
            handleOutputClick(this, 'slider3');
        });
    </script>
</html>
'''


@app.route("/")
def home():
    return render_template_string(TPL)


@app.route("/reset", methods=["POST"])
def reset():
    initialize_servos()
    return render_template_string(TPL)


@app.route("/iktest", methods=["POST"])
def iktest():
    ik_matrix = request.form["ik_matrix"]
    ik_step_time = request.form["ik_step_time"]
    ik_leg = request.form["ik_leg"]
    leg = legs[int(ik_leg)]
    matrix = np.array(eval(ik_matrix))
    step_time = float(ik_step_time)

    print(matrix.shape[1])
    for i in range(matrix.shape[1]):
        angle1 = 90+math.degrees(matrix[0][i])
        angle2 = 90+math.degrees(matrix[1][i])
        angle3 = math.degrees(abs(matrix[2][i]))


        SetAngle(int(leg[0]), angle1)
        SetAngle(int(leg[1]), angle2)
        SetAngle(int(leg[2]), angle3)
        sleep(step_time)
        print("slept:", step_time)
    return redirect("/")


@app.route("/ik", methods=["POST"])
def ik():
    ik_matrix_full = request.form["ik_matrix_full"]
    ik_step_time = request.form["ik_step_time"]
    ik_loop_count = request.form["ik_loop_count"]
    allLegs = [legs[0], legs[1], legs[2], legs[3]]
    matrix_full = np.array(
        eval(ik_matrix_full.replace('\n', '').replace(' ', '')))
    loop_count = int(ik_loop_count)
    if loop_count < 1:
        loop_count = 1
    step_time = float(ik_step_time)

    for j in range(loop_count):
        print(f"Roll: {roll:.2f} degrees, Pitch: {pitch:.2f} degrees")
        for i in range(max(matrix_full[0].shape[1], matrix_full[1].shape[1], matrix_full[2].shape[1], matrix_full[3].shape[1])):
            greyAngle1 = GetAngle(int(allLegs[0][0]))
            greyAngle2 = GetAngle(int(allLegs[0][1]))
            greyAngle3 = GetAngle(int(allLegs[0][2]))
            blackAngle1 = GetAngle(int(allLegs[1][0]))
            blackAngle2 = GetAngle(int(allLegs[1][1]))
            blackAngle3 = GetAngle(int(allLegs[1][2]))
            redAngle1 = GetAngle(int(allLegs[2][0]))
            redAngle2 = GetAngle(int(allLegs[2][1]))
            redAngle3 = GetAngle(int(allLegs[2][2]))
            blueAngle1 = GetAngle(int(allLegs[3][0]))
            blueAngle2 = GetAngle(int(allLegs[3][1]))
            blueAngle3 = GetAngle(int(allLegs[3][2]))

            if i < matrix_full[0].shape[1]:
                greyAngle1 = round(
                    90 + math.degrees(float(matrix_full[0][0][i])), 2)
                greyAngle2 = round(
                    90 + math.degrees(float(matrix_full[0][1][i])), 2)
                greyAngle3 = round(math.degrees(
                    abs(float(abs(matrix_full[0][2][i])))), 2)

            if i < matrix_full[1].shape[1]:
                blackAngle1 = round(
                    90+math.degrees(float(matrix_full[1][0][i])), 2)
                blackAngle2 = round(
                    90+math.degrees(float(matrix_full[1][1][i])), 2)
                blackAngle3 = round(math.degrees(
                    float(abs(matrix_full[1][2][i]))), 2)

            if i < matrix_full[2].shape[1]:
                redAngle1 = round(90+math.degrees(float(matrix_full[2][0][i])), 2)
                redAngle2 = round(90+math.degrees(float(matrix_full[2][1][i])), 2)
                redAngle3 = round(math.degrees(
                    float(abs(matrix_full[2][2][i]))), 2)

            if i < matrix_full[3].shape[1]:
                blueAngle1 = round(90+math.degrees(float(matrix_full[3][0][i])), 2)
                blueAngle2 = round(90+math.degrees(float(matrix_full[3][1][i])), 2)
                blueAngle3 = round(math.degrees(
                    float(abs(matrix_full[3][2][i]))), 2)

            SetAngle(int(allLegs[0][0]), greyAngle1)
            SetAngle(int(allLegs[0][1]), greyAngle2)
            SetAngle(int(allLegs[0][2]), greyAngle3)

            SetAngle(int(allLegs[1][0]), blackAngle1)
            SetAngle(int(allLegs[1][1]), blackAngle2)
            SetAngle(int(allLegs[1][2]), blackAngle3)

            SetAngle(int(allLegs[2][0]), redAngle1)
            SetAngle(int(allLegs[2][1]), redAngle2)
            SetAngle(int(allLegs[2][2]), redAngle3)

            SetAngle(int(allLegs[3][0]), blueAngle1)
            SetAngle(int(allLegs[3][1]), blueAngle2)
            SetAngle(int(allLegs[3][2]), blueAngle3)
            sleep(step_time)
    return redirect("/")


@app.route("/test", methods=["POST"])
def test():
    selected_leg_index = int(request.form["selected_leg"])
    leg = legs[selected_leg_index]

    # Get slider values
    slider1 = int(request.form["slider1"])
    slider2 = int(request.form["slider2"])
    slider3 = int(request.form["slider3"])

    # Set angles for the selected leg's servos
    SetAngle(leg[0], slider1)
    SetAngle(leg[1], slider2)
    SetAngle(leg[2], slider3)

    return redirect("/")


@app.route("/move", methods=["POST"])
def move():
    ik_matrix_full = "[[[ 0.00000000e+00, -4.63701429e-01, -4.63653134e-01, -4.63648714e-01," + "-4.63654642e-01, -1.03036939e+00, -1.03037475e+00,  1.67648366e-06," + " 3.55165330e-07, -2.81580390e-07, -5.86882774e-08]," + "[ 9.08676818e-01,  1.01239756e+00,  1.01240806e+00,  1.01244365e+00," + " 1.57079633e+00,  1.57079633e+00,  9.24552347e-01,  9.08745752e-01," + " 9.08735018e-01,  9.08729909e-01,  9.08722475e-01]," + "[-2.22232033e+00, -2.52574463e+00, -2.52578098e+00, -2.52576480e+00," + "-2.57288056e+00, -2.33253680e+00, -2.25895127e+00, -2.22237846e+00," + "-2.22238261e+00, -2.22236919e+00, -2.22237201e+00]," + "[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + " 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + " 0.00000000e+00,  0.00000000e+00,  0.00000000e+00]]," + "[[-5.86882774e-08,  6.92899790e-08,  9.27333890e-01,  9.27347828e-01," + " 9.27295220e-01,  9.27295218e-01,  9.27295218e-01, -1.88297767e-06," + "-7.65775039e-07, -7.65775039e-07, -7.65775039e-07]," + "[ 9.08722475e-01,  1.57079633e+00,  1.57079633e+00,  9.88477307e-01," + " 9.88474504e-01,  9.88445744e-01,  9.88449682e-01,  9.08700675e-01," + " 9.08694832e-01,  9.08694832e-01,  9.08694832e-01]," + "[-2.22237201e+00, -2.49346154e+00, -2.63885625e+00, -2.42865983e+00," + "-2.42863070e+00, -2.42864550e+00, -2.42862674e+00, -2.22234119e+00," + "-2.22234863e+00, -2.22234863e+00, -2.22234863e+00]," + "[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + " 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + " 0.00000000e+00,  0.00000000e+00,  0.00000000e+00]]," + "[[-7.65775039e-07,  1.97431223e-01,  1.97413685e-01,  1.97394835e-01," + " 1.97394835e-01,  1.97394835e-01,  1.32582082e+00,  1.52084421e+00," + " 1.52085928e+00,  1.97391582e-01,  1.97391582e-01]," + "[ 9.08694832e-01,  9.82203659e-01,  9.82229467e-01,  9.82222432e-01," + " 9.82222432e-01,  9.82222432e-01,  8.90403394e-01,  2.94015738e-01," + " 7.13129747e-01,  9.82189347e-01,  9.82189347e-01]," + "[-2.22234863e+00, -2.40938873e+00, -2.40934656e+00, -2.40935010e+00," + "-2.40935010e+00, -2.40935010e+00, -2.18157874e+00, -1.07688059e+00," + "-1.33234661e+00, -2.40932527e+00, -2.40932527e+00]," + "[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + " 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + " 0.00000000e+00,  0.00000000e+00,  0.00000000e+00]]," + "[[-1.65156815e-01, -1.65151517e-01, -1.65141361e-01, -1.40564026e+00," + "-1.40565120e+00, -1.40565120e+00, -1.40565120e+00, -1.40565120e+00," + "-1.40565120e+00, -1.40565120e+00, -1.40565120e+00]," + "[ 9.00765337e-01,  9.00729741e-01,  9.00692832e-01,  9.00631318e-01," + " 9.00629346e-01,  9.00629346e-01,  9.00629346e-01,  9.00629346e-01," + " 9.00629346e-01,  9.00629346e-01,  9.00629346e-01]," + "[-2.20424149e+00, -2.20426718e+00, -2.20421396e+00, -2.20418584e+00," + "-2.20418542e+00, -2.20418542e+00, -2.20418542e+00, -2.20418542e+00," + "-2.20418542e+00, -2.20418542e+00, -2.20418542e+00]," + "[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + " 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + " 0.00000000e+00,  0.00000000e+00,  0.00000000e+00]]]"
    ik_matrix_full_crooked = "[[[ 0.00000000e+00, -4.63701429e-01, -4.63653134e-01, -4.63648714e-01, -4.63654642e-01, -1.03036939e+00, -1.03037475e+00,  1.67648366e-06," + "3.55165330e-07, -2.81580390e-07, -5.86882774e-08]," + "[ 9.08676818e-01,  1.01239756e+00,  1.01240806e+00,  1.01244365e+00," + "1.57079633e+00,  1.57079633e+00,  9.24552347e-01,  9.08745752e-01," + "9.08735018e-01,  9.08729909e-01,  9.08722475e-01]," + "[-2.22232033e+00, -2.52574463e+00, -2.52578098e+00, -2.52576480e+00," + "-2.57288056e+00, -2.33253680e+00, -2.25895127e+00, -2.22237846e+00," + "-2.22238261e+00, -2.22236919e+00, -2.22237201e+00]," + "[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + "0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + "0.00000000e+00,  0.00000000e+00,  0.00000000e+00]]," + "[[ 6.92899790e-08,  5.43230806e-07,  9.27301357e-01,  9.27288663e-01," + "9.27288663e-01,  9.27288663e-01,  9.27288663e-01, -9.06709407e-08," + "-9.06709407e-08, -9.06709407e-08, -9.06709407e-08]," + "[ 1.57079633e+00,  1.57079633e+00,  1.57079633e+00,  1.57079633e+00," + "1.57079633e+00,  1.57079633e+00,  1.57079633e+00,  1.57079633e+00," + "1.57079633e+00,  1.57079633e+00,  1.57079633e+00]," + "[-2.49346154e+00, -2.22856392e+00, -2.38726103e+00, -2.63890853e+00," + "-2.63890853e+00, -2.63890853e+00, -2.63890853e+00, -2.49345705e+00," + "-2.49345705e+00, -2.49345705e+00, -2.49345705e+00]," + "[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + "0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + "0.00000000e+00,  0.00000000e+00,  0.00000000e+00]]," + "[[ 2.08791461e-06,  1.97431209e-01,  1.97413677e-01,  1.97394835e-01," + "1.97394835e-01,  1.97394835e-01,  1.32582082e+00,  1.52084421e+00," + "1.52085928e+00,  1.97391582e-01,  1.97391582e-01]," + "[ 9.08689217e-01,  9.82203708e-01,  9.82229503e-01,  9.82222465e-01," + "9.82222465e-01,  9.82222465e-01,  8.90403395e-01,  2.94015738e-01," + "7.13129747e-01,  9.82189347e-01,  9.82189347e-01]," + "[-2.22237458e+00, -2.40938874e+00, -2.40934659e+00, -2.40935012e+00," + "-2.40935012e+00, -2.40935012e+00, -2.18157874e+00, -1.07688059e+00," + "-1.33234661e+00, -2.40932527e+00, -2.40932527e+00]," + "[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + "0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + "0.00000000e+00,  0.00000000e+00,  0.00000000e+00]]," + "[[-1.65149256e-01, -1.65148400e-01, -1.65148607e-01, -1.40564761e+00," + "-1.40564761e+00, -1.40564761e+00, -1.40564761e+00, -1.40564761e+00," + "-1.40564761e+00, -1.40564761e+00, -1.40564761e+00]," + "[ 1.28070159e+00,  1.28072204e+00,  1.28071912e+00,  1.28077935e+00," + "1.28077935e+00,  1.28077935e+00,  1.28077935e+00,  1.28077935e+00," + "1.28077935e+00,  1.28077935e+00,  1.28077935e+00]," + "[-2.39599168e+00, -2.39598179e+00, -2.39599495e+00, -2.39603423e+00," + "-2.39603423e+00, -2.39603423e+00, -2.39603423e+00, -2.39603423e+00," + "-2.39603423e+00, -2.39603423e+00, -2.39603423e+00]," + "[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + "0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00," + "0.00000000e+00,  0.00000000e+00,  0.00000000e+00]]]" 
    ik_step_time = 0.2
    ik_loop_count = 2
    allLegs = [legs[0], legs[1], legs[2], legs[3]]
    
    loop_count = int(ik_loop_count)
    if loop_count < 1:
        loop_count = 1
    step_time = float(ik_step_time)

    global roll
    global pitch

    for j in range(loop_count):
        print(f"Roll: {roll:.2f} degrees, Pitch: {pitch:.2f} degrees")
        if pitch < -8:
            print("Crooked matrix chosen")
            matrix_full = np.array(eval(ik_matrix_full_crooked.replace('\n', '').replace(' ', '')))
        else:
            print("Normal matrix chosen")
            matrix_full = np.array(eval(ik_matrix_full.replace('\n', '').replace(' ', '')))
        for i in range(max(matrix_full[0].shape[1], matrix_full[1].shape[1], matrix_full[2].shape[1], matrix_full[3].shape[1])):
            greyAngle1 = GetAngle(int(allLegs[0][0]))
            greyAngle2 = GetAngle(int(allLegs[0][1]))
            greyAngle3 = GetAngle(int(allLegs[0][2]))
            blackAngle1 = GetAngle(int(allLegs[1][0]))
            blackAngle2 = GetAngle(int(allLegs[1][1]))
            blackAngle3 = GetAngle(int(allLegs[1][2]))
            redAngle1 = GetAngle(int(allLegs[2][0]))
            redAngle2 = GetAngle(int(allLegs[2][1]))
            redAngle3 = GetAngle(int(allLegs[2][2]))
            blueAngle1 = GetAngle(int(allLegs[3][0]))
            blueAngle2 = GetAngle(int(allLegs[3][1]))
            blueAngle3 = GetAngle(int(allLegs[3][2]))

            if i < matrix_full[0].shape[1]:
                greyAngle1 = round(
                    90 + math.degrees(float(matrix_full[0][0][i])), 2)
                greyAngle2 = round(
                    90 + math.degrees(float(matrix_full[0][1][i])), 2)
                greyAngle3 = round(math.degrees(
                    abs(float(abs(matrix_full[0][2][i])))), 2)

            if i < matrix_full[1].shape[1]:
                blackAngle1 = round(
                    90+math.degrees(float(matrix_full[1][0][i])), 2)
                blackAngle2 = round(
                    90+math.degrees(float(matrix_full[1][1][i])), 2)
                blackAngle3 = round(math.degrees(
                    float(abs(matrix_full[1][2][i]))), 2)

            if i < matrix_full[2].shape[1]:
                redAngle1 = round(90+math.degrees(float(matrix_full[2][0][i])), 2)
                redAngle2 = round(90+math.degrees(float(matrix_full[2][1][i])), 2)
                redAngle3 = round(math.degrees(
                    float(abs(matrix_full[2][2][i]))), 2)

            if i < matrix_full[3].shape[1]:
                blueAngle1 = round(90+math.degrees(float(matrix_full[3][0][i])), 2)
                blueAngle2 = round(90+math.degrees(float(matrix_full[3][1][i])), 2)
                blueAngle3 = round(math.degrees(
                    float(abs(matrix_full[3][2][i]))), 2)

            SetAngle(int(allLegs[0][0]), greyAngle1)
            SetAngle(int(allLegs[0][1]), greyAngle2)
            SetAngle(int(allLegs[0][2]), greyAngle3)

            SetAngle(int(allLegs[1][0]), blackAngle1)
            SetAngle(int(allLegs[1][1]), blackAngle2)
            SetAngle(int(allLegs[1][2]), blackAngle3)

            SetAngle(int(allLegs[2][0]), redAngle1)
            SetAngle(int(allLegs[2][1]), redAngle2)
            SetAngle(int(allLegs[2][2]), redAngle3)

            SetAngle(int(allLegs[3][0]), blueAngle1)
            SetAngle(int(allLegs[3][1]), blueAngle2)
            SetAngle(int(allLegs[3][2]), blueAngle3)
            sleep(step_time)
    return redirect("/")



if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8000, debug=True)
