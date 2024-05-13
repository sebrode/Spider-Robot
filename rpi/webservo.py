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
legs = [leg0, leg1, leg2, leg3]


def reset():
    for i in range(16):
        kit.servo[i].set_pulse_width_range(530, 2470)
        if i in [0, 12]:
            kit.servo[i].angle = 60
        elif i in [4, 8]:
            kit.servo[i].angle = 120
        else:
            kit.servo[i].angle = 90


reset()


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
<p>Slider 1 <input type="range" min="1" max="180" name="slider1" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output1">90</output></p>
<p>Slider 2 <input type="range" min="1" max="180" name="slider2" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output2">90</output></p>
<p>Slider 3 <input type="range" min="1" max="180" name="slider3" value="90" oninput="this.nextElementSibling.value = this.value"/> <output id="output2">90</output></p>
            <input type="submit" value="submit" />
        </form>
        <h1>IK Matrix</h1>
        <form method="POST" action="ik">
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
    reset()
    return render_template_string(TPL)


@app.route("/ik", methods=["POST"])
def ik():
    ik_matrix = request.form["ik_matrix"]
    ik_step_time = request.form["ik_step_time"]
    ik_leg = request.form["ik_leg"]

    leg = legs[int(ik_leg)]
    matrix = np.array(eval(ik_matrix))
    step_time = float(ik_step_time)
    offset = 3

    print("PUHAda", matrix)
    for i in range(matrix.shape[1]):
        for j in range(len(matrix[i])):
            angle = matrix[j*offset, i]
            degAngle = math.degrees(angle)
            editedAngle = degAngle+90
            print("ANGLE:", degAngle)
            SetAngle(int(leg[j]), editedAngle)
        sleep(step_time)
        print("slept:", step_time)
    return redirect("/")


@app.route("/test", methods=["POST"])
def test():
    # Get slider Values
    slider1 = request.form["slider1"]
    slider2 = request.form["slider2"]
    slider3 = request.form["slider3"]
    SetAngle(0, int(slider1))
    SetAngle(1, int(slider2))
    SetAngle(2, int(slider3))
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
