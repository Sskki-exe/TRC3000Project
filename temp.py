from flask import Flask, request, render_template

app = Flask(__name__)

@app.route('/', methods=["GET","POST"])
def clickPrint():
    if request.method == "POST":
        pass
    return render_template('partC.html')

if __name__=="__main__":
    app.run(host="0.0.0.0", port=80)
