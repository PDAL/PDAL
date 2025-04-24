import os

def setup(app):
    if os.environ.get('READTHEDOCS') == 'True':
        app.config["html_baseurl"] = os.environ['READTHEDOCS_CANONICAL_URL']