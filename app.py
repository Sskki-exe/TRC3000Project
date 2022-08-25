from urllib import request
from flask import Flask, render_template

app = Flask(__name__)

@app.route('/partA')
def partA():
    if request.method == 'POST':
        print(request.form['btn'])        
    return render_template('partA.html')

if __name__ == "__main__":
    app.run()