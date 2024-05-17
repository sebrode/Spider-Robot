# Importing the Flask modules required for this project
from flask import Flask, render_template_string, redirect, request
# Importing the GPIO library to control GPIO pins of Raspberry Pi
import RPi.GPIO as GPIO
# Importing the Flask modules required for this project
from flask import Flask, render_template_string, redirect, request
from time import sleep      # Import sleep module from time library to add delays
from adafruit_servokit import ServoKit
import numpy as np
import math

kit = ServoKit(channels=16)
leg0 = [0, 1, 2]
leg1 = [4, 5, 6]
leg2 = [8, 9, 10]
leg3 = [12, 13, 14]
legs = [leg0, leg1, leg2, leg3, [leg0, leg1, leg2, leg3]]


def initialize_servos():
    for i in range(16):
        kit.servo[i].set_pulse_width_range(530, 2470)
        if i in [0, 12]:
            kit.servo[i].angle = 60
        elif i in [4, 8]:
            kit.servo[i].angle = 120
        else:
            kit.servo[i].angle = 90
# initialize_servos()


def SetAngle(num, ang):
    kit.servo[num].angle = ang


# Flask constructor takes the name of current module (__name__) as argument.
app = Flask(__name__)
# Enable debug mode
app.config['DEBUG'] = True


# Store HTML code
TPL = '''
<html>
    <head><title>Web Application to control Servos </title></head>
    <body>
    <h1> Web Application to Control Servos</h1>
        <form method="POST" action="reset">
            <input type="submit" value="Reset all servos" />
        </form>
        <form method="POST" action="test">
        <select name="selected_leg">
                <option value="0">Grey Leg</option>
                <option value="1">Black Leg</option>
                <option value="2" selected>Red Leg</option>
                <option value="3">Blue Leg</option>
            </select>

        <p>Slider 1 <input type="range" min="1" max="180" name="slider1" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output1">90</output></p>
        <p>Slider 2 <input type="range" min="1" max="180" name="slider2" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output2">90</output></p>
        <p>Slider 3 <input type="range" min="1" max="180" name="slider3" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output2">90</output></p>
                    <input type="submit" value="submit" />
        </form>
        <h1>IK Matrix All Legs</h1>
        <form method="POST" action="ik">
            <textarea id="ik_matrix_full" placeholder="Full Matrix" name="ik_matrix_full" rows="10" cols="50"></textarea>
            <input type="number" id="ik_step_time" name="ik_step_time" value="0.2" min="0" max="10" step="0.1">
            <input type="submit" value="submit" />
        </form>
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
        <h1>Movement</h1>
        <form method="POST" action="move">
            <button type="submit" name="direction" value="forward">^</button>
            <button type="submit" name="direction" value="left"><</button>
            <button type="submit" name="direction" value="right">></button>
            <button type="submit" name="direction" value="back">v</button>
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
</script>
</html>
'''

# which URL should call the associated function.


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

    for i in range(matrix.shape[0]):
        angle1 = 90
        angle2 = 90-math.degrees(matrix[i][1])
        angle3 = 45-math.degrees(matrix[i][2])
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
    allLegs = [legs[0], legs[1], legs[2], legs[3]]
    matrix_full = np.array(
        eval(ik_matrix_full.replace('\n', '').replace(' ', '')))
    step_time = float(ik_step_time)

    greyAngle1 = 0
    greyAngle2 = 0
    greyAngle3 = 0
    blackAngle1 = 0
    blackAngle2 = 0
    blackAngle3 = 0
    redAngle1 = 0
    redAngle2 = 0
    redAngle3 = 0
    blueAngle1 = 0
    blueAngle2 = 0
    blueAngle3 = 0
    for i in range(max(matrix_full[0].shape[1], matrix_full[1].shape[1], matrix_full[2].shape[1], matrix_full[3].shape[1])):
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

        print(greyAngle1, greyAngle2, greyAngle3,
              blackAngle1, blackAngle2, blackAngle3)
        print(redAngle1, redAngle2, redAngle3,
              blueAngle1, blueAngle2, blueAngle3)

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
    # Get selected leg index from the form
    selected_leg_index = int(request.form["selected_leg"])
    leg = legs[selected_leg_index]  # Get the servo IDs for the selected leg

    # Get slider values
    slider1 = int(request.form["slider1"])
    slider2 = int(request.form["slider2"])
    slider3 = int(request.form["slider3"])

    # Set angles for the selected leg's servos
    SetAngle(leg[0], slider1)
    SetAngle(leg[1], slider2)
    SetAngle(leg[2], slider3)

    # Change duty cycle
    return redirect("/")


@app.route("/move", methods=["POST"])
def move():
    direction = request.form["direction"]
    print(direction)
    match(direction):
        case "forward":
            kit.servo[0].angle = 60
            kit.servo[1].angle = 120
            sleep(0.2)
            kit.servo[0].angle = 120
            kit.servo[1].angle = 60
        case "back":
            kit.servo[0].angle = 120
            kit.servo[1].angle = 60
            sleep(0.2)
            kit.servo[0].angle = 60
            kit.servo[1].angle = 120
    return redirect("/")


# Run the app on the local development server
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8000, debug=True)
