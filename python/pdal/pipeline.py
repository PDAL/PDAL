
from pdal import libpdalpython

class Pipeline(object):
    """A PDAL pipeline object, defined by JSON. See http://www.pdal.io/pipeline.html for more
    information on how to define one"""

    def __init__(self, json):
        self.p = libpdalpython.PyPipeline(str(json))

    def get_metadata(self):
        return self.p.metadata
    metadata = property(get_metadata)

    def get_schema(self):
        return self.p.schema
    schema = property(get_schema)

    def get_pipeline(self):
        return self.p.pipeline
    pipeline = property(get_pipeline)

    def get_loglevel(self):
        return self.p.loglevel

    def set_loglevel(self, v):
        self.p.loglevel = v
    loglevel = property(get_loglevel, set_loglevel)

    def get_log(self):
        return self.p.log
    log = property(get_log)

    def execute(self):
        return self.p.execute()

    def get_arrays(self):
        return self.p.arrays
    arrays = property(get_arrays)
