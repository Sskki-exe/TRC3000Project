import random
import time
from flask import Flask, request, render_template, jsonify

app = Flask(__name__)

dataSize = 50
dataLabels = []
dataValues = []
def genData():
    dataLabels.append(time.strftime("%H:%M:%S", time.localtime()))
    dataValues.append(random.random()*100)
    while len(dataLabels) > dataSize:
        dataLabels.pop(0)    
    while len(dataValues) > dataSize:
        dataValues.pop(0)

@app.route('/getdataC', methods= ['GET'])
def stuffC():
    mass = int(random.random()*10)
    fl = int(random.random()*10)
    genData()
    return jsonify(m=mass,fl=fl,l=dataLabels,v=dataValues)

@app.route('/', methods=["GET","POST"])
def home():
    if request.method == "POST":
        pass
    return render_template('partC.html')

if __name__=="__main__":
    app.run(host="0.0.0.0", port=80)
