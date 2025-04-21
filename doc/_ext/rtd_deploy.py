import os

def setup(app):
    if 'READTHEDOCS_CANONICAL_URL' in os.environ:
        print('\n\n\nrtd env set: ' + os.getenv("READTHEDOCS_CANONICAL_URL") + '\n\n\n')
        app.add_config_value('html_baseurl', os.environ.get('READTHEDOCS_CANONICAL_URL', ''), 'env')