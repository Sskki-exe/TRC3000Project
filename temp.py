from asyncio.windows_events import NULL
from cmath import sqrt
from flask import Flask, request, render_template, jsonify

app = Flask(__name__)

@app.route('/getdataC', methods= ['GET'])
def stuffC():
    return jsonify(fl="1",m="1")

@app.route('/', methods=["POST"])
def partC():
    if request.method == "POST":
        btn = request.form['btn']
        if btn == "Program Start":
            pass #function to start program here
    return render_template('partC.html')

if __name__ == "__main__":
    app.run()