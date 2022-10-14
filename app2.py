from cmath import sqrt
from ipaddress import collapse_addresses
from flask import Flask, request, render_template, jsonify
import main
import camerafunctions

app = Flask(__name__)

@app.route('/getdata', methods= ['GET'])
def stuff():
    mass = main.readLS()
    return jsonify(mass=mass)

@app.route('/getdataB', methods= ['GET'])
def stuffB():
    tx=main.MPU_getValue('wX')
    ty=main.MPU_getValue('wY')
    ts=sqrt(tx^2 + ty^2)

    ax=main.MPU_getValue('aX')
    ay=main.MPU_getValue('aY')
    az=main.MPU_getValue('aZ')
    aS=sqrt(ax^2 + ay^2)

    return jsonify(tx=tx,ty=ty,ts=ts,ax=ax,ay=ay,az=az,aS=aS)

@app.route('/getdataC', methods= ['GET'])
def stuffC():
    colour = camerafunctions.sample_color()
    foamLevel = camerafunctions.foam_height()
    dataLabels = ["a","b","c"] #replace with list of timestamps
    dataValues = [1,2,3] #replace with list of color values
    return jsonify(fl=foamLevel,l=dataLabels,v=dataValues)

@app.route('/lc', methods=["GET","POST"])
def partA():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Tare":
            main.tareLS()
        if btn == "Calibrate LC":
            main.calibrateLS(request.form['amount'])
    return render_template('partA.html')

@app.route('/move', methods=["GET","POST"])
def partB():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Go to angle":
            main.setAngle(request.form['targetA'])
        if btn == "Calibrate IMU":
            main.MPU_calibrate()
    return render_template('partB.html')

@app.route('/photo', methods=["GET","POST"])
def partC():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Take Photo":
            main.takePic()
        if btn == "Toggle Light":
            main.toggleLight()
        if btn == "Detect Colour":
            pass #main. ADD COLOUR DETECTION
    return render_template('partC.html')

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=80)
