import random
from flask import Flask, request, render_template, jsonify

app = Flask(__name__)

@app.route('/getdataC', methods= ['GET'])
def stuffC():
    mass = random.random()
    fl = random.random()
    labels = ["a","b","c","d","e"]
    values = random.sample(range(1,10),5)
    return jsonify(m=mass,fl=fl,{'l':labels},{'v':values})

@app.route('/', methods=["GET","POST"])
def home():
    if request.method == "POST":
        pass
    return render_template('partC.html')

if __name__=="__main__":
    app.run(host="0.0.0.0", port=80)
