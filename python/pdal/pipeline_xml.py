from xml.etree import ElementTree
import pdal


class XMLElement(object):

    def __init__(self):
        self.source = None

    def xml(self):
        element = ElementTree.Element(self.tag, self.attrib)
        if self.source:
            try:
                sources = iter(self.source)
            except TypeError:
                sources = [self.source]
            for source in sources:
                element.append(source.xml())
        return element


class Pipeline(XMLElement):

    tag = 'Pipeline'

    def __init__(self, customversion=None):
        super(Pipeline, self).__init__()
        self.attrib = {'version': customversion or pdal.__version__}

    def xml(self):
        element = super(Pipeline, self).xml()
        return ElementTree.ElementTree(element)


class PipelineComponent(XMLElement):

    def __init__(self, drivertype):
        super(PipelineComponent, self).__init__()
        self.attrib = {'type': self.typeformat % drivertype}


class Stage(PipelineComponent):
    pass


class Reader(Stage):
    tag = 'Reader'
    typeformat = 'readers.%s'


class Filter(Stage):
    tag = 'Filter'
    typeformat = 'filters.%s'


class MultiFilter(Filter):
    tag = 'MultiFilter'


class Writer(PipelineComponent):
    tag = 'Writer'
    typeformat = 'writers.%s'
