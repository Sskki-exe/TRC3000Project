from asyncio.windows_events import NULL
from flask import Flask, request, render_template, jsonify
import main

app = Flask(__name__)

@app.route('/getdata', methods= ['GET'])
def stuff():
    mass = main.readLS()
    
    tx=main.MPU_getValue('wX')
    ty=main.MPU_getValue('wY')
    ax=main.MPU_getValue('aX')
    ay=main.MPU_getValue('aY')
    az=main.MPU_getValue('aZ')
    return jsonify(mass=mass,tx=tx,ty=ty,ax=ax,ay=ay,az=az)

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

@app.route('/partB', methods=["POST"])
def partB():
    return render_template('partB.html')

@app.route('/partC', methods=["POST"])
def partC():
    return render_template('partC.html')

if __name__ == "__main__":
    app.run()