import json

def get_config():
    config = json.loads(open('config.json', 'r').read())

    return config