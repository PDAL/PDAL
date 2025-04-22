# script to add readthedocs canonical url
import os

def setup(app):
    if 'READTHEDOCS_CANONICAL_URL' in os.environ:
        app.config["html_baseurl"] = os.environ.get('READTHEDOCS_CANONICAL_URL', '')