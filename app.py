from asyncio.windows_events import NULL
from flask import Flask, request, render_template, jsonify
import main

app = Flask(__name__)

@app.route('/getdata', methods= ['GET'])
def stuff():
    mass = main.readLS()
    
    tx = 
    ty=
    tz=
    ax=
    ay=
    az=
    return jsonify(mass=mass,tx=tx,ty=ty,tz=tz,ax=ax,ay=ay,az=az)

@app.route('/partA', methods=["POST"])
def partA():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Tare":
            main.tareLS()
        if btn == "Calibrate LC":
            main.calibrateLS(request.form['amount'])
        if btn == "Go to angle":
            main.setAngle(request.form['targetA'])
        if btn == "Calibrate IMU":
            main.MPU_calibrate()
        if btn == "Capture Image":
            main.takePic()
    return render_template('partA.html')

if __name__ == "__main__":
    app.run()