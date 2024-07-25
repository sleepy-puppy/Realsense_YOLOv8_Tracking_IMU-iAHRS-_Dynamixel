import openrouteservice
from flask import Flask, render_template, request, jsonify
import webbrowser
import threading

app = Flask(__name__)

# OpenRouteService API Key
ORS_API_KEY = '5b3ce3597851110001cf6248c095f2a04f244c2bb56e451d005b8712'

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/route', methods=['POST'])
def route():
    data = request.get_json()
    start = data['start']
    end = data['end']

    client = openrouteservice.Client(key=ORS_API_KEY)
    coords = (start, end)
    routes = client.directions(coordinates=coords, profile='driving-car', format='geojson')

    waypoints = routes['features'][0]['geometry']['coordinates']

    return jsonify({'route': routes, 'waypoints': waypoints})

def open_browser():
    webbrowser.open_new('http://127.0.0.1:5000/')

if __name__ == '__main__':
    threading.Timer(1, open_browser).start()
    app.run(use_reloader=False, debug=True)
