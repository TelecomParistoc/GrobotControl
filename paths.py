import json
from os import listdir

PATHS_FOLDER = "paths/"

def json_to_python(filename, color):
    """
        loads a .json and returns a list [(x0, y0), (x1, y1), ...]
        make sure color is green or orange
    """
    with open(PATHS_FOLDER + filename, "r") as f:
        result = json.loads(f.read())

    return [(p['x'], p['y']) for p in result[color][0]['points']]


def load_all_path(color):
    assert color == "green" or color == "orange"

    for file in listdir(PATHS_FOLDER):
        exec("global " + file + " = json_to_python(file, color)")
