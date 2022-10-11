from cmath import sqrt
from flask import Flask, request, render_template, jsonify

app = Flask(__name__)

@app.route('/getdata', methods= ['GET'])
def stuff():
    mass = 0#main.readLS()
    
    tx=0#main.MPU_getValue('wX')
    ty=0#main.MPU_getValue('wY')
    ax=0#main.MPU_getValue('aX')
    ay=0#main.MPU_getValue('aY')
    az=0#main.MPU_getValue('aZ')
    return jsonify(mass=mass,tx=tx,ty=ty,ax=ax,ay=ay,az=az)

@app.route('/getdataB', methods= ['GET'])
def stuffB():
    speed = 0 #fastest speed
    tx=0#main.MPU_getValue('wX')
    ty=0#main.MPU_getValue('wY')
    ts=sqrt(tx^2 + ty^2)

    ax=0#main.MPU_getValue('aX')
    ay=0#main.MPU_getValue('aY')
    aS=sqrt(ax^2 + ay^2)

    return jsonify(speed=speed,tx=tx,ty=ty,ts=ts,ax=ax,ay=ay,aS=aS)

@app.route('/getdataC', methods= ['GET'])
def stuffC():
    mass = 0#main.readLS()
    fl = 0
    return jsonify(m=mass,fl=fl)

@app.route('/partA', methods=["POST"])
def partA():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Tare":
            0#main.tareLS()
        if btn == "Calibrate LC":
            0#main.calibrateLS(request.form['amount'])
        if btn == "Go to angle":
            0#main.setAngle(request.form['targetA'])
        if btn == "Calibrate IMU":
            0#main.MPU_calibrate()
        if btn == "Capture Image":
            0#main.takePic()
    return render_template('partA.html')

@app.route('/partB', methods=["POST"])
def partB():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Calibrate":
            pass #calibration function goes here
    return render_template('partB.html')

@app.route('/', methods=["POST"])
def partC():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Calibrate Load Cell":
            0#main.calibrateLS(request.form['amount'])
        if btn == "Calibrate IMU":
            0#main.MPU_calibrate()
        if btn == "Program Start":
            pass #function to start program here
    return render_template('partC.html')

if __name__ == "__main__":
    app.run(host="0.0.0.0",port=80)