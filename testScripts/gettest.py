from flask import Flask, jsonify
import time

app = Flask(__name__)

@app.route('/pid', methods=['GET'])
def respond_to_get_request():
    response_data = {
        "kp": 0.2,
        "ki": 0.8,
        "kd": 1.2,
        "timestamp": int(time.time())
    }
    return jsonify(response_data)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

