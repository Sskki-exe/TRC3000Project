from flask import Flask, request, render_template, jsonify
#import main

app = Flask(__name__)

mass=0
tx=0
ty=0
tz=0
ax=0
ay=0
az=0 

@app.route('/getdata', methods= ['GET'])
def stuff():
    global mass,tx,ty,tz,ax,ay,az
    mass+=1
    tx+=1
    ty+=1
    tz+=1
    ax+=1
    ay+=1
    az+=1    
    return jsonify(mass=mass,tx=tx,ty=ty,tz=tz,ax=ax,ay=ay,az=az)

@app.route('/partA', methods=["GET","POST"])
def partA():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Tare":
            print(1)
        if btn == "Calibrate LC":
            print(2)
        if btn == "Go to angle":
            print(3)
        if btn == "Calibrate IMU":
            print(4)
        if btn == "Capture Image":
            print(5)
        print(request.form.get("runLS"))
    return render_template('partA.html')

if __name__ == "__main__":
    app.run()