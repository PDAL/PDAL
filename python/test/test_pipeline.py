import unittest
import pdal
import pdal.pipeline_xml as pxml

class TestXML(unittest.TestCase):

    def test_simplest_xml(self):
        p = pxml.Pipeline()
        p.source = pxml.Reader('foo')
        xml = p.xml()
        self.assertEqual(xml.getroot().attrib['version'], pdal.__version__)
        self.assertEqual(xml.find('Reader').attrib['type'],
                         'readers.foo')

    def test_writer(self):
        xml = pxml.Writer('bar').xml()
        self.assertEqual(xml.tag, 'Writer')
        self.assertEqual(xml.attrib['type'], 'writers.bar')

    def test_pipeline_version(self):
        xml = pxml.Pipeline('custom version').xml()
        self.assertEqual(xml.getroot().attrib['version'], 'custom version')

    def test_filter(self):
        xml = pxml.Filter('foo').xml()
        self.assertEqual(xml.tag, 'Filter')
        self.assertEqual(xml.attrib['type'], 'filters.foo')

    def test_multifilter(self):
        xml = pxml.MultiFilter('foo').xml()
        self.assertEqual(xml.tag, 'MultiFilter')
        self.assertEqual(xml.attrib['type'], 'filters.foo')

    def test_writer_source(self):
        writer = pxml.Writer('foo')
        writer.source = pxml.Reader('bar')
        xml = writer.xml()
        self.assertTrue(xml.find('Reader') is not None)

    def test_multifilter_source(self):
        mfilter = pxml.MultiFilter('multi')
        mfilter.source = [pxml.Reader('foo'), pxml.Reader('bar')]
        xml = mfilter.xml()
        self.assertEqual(len(xml.findall('Reader')), 2)

def test_suite():
    return unittest.TestSuite(
        [TestXML])

if __name__ == '__main__':
    unittest.main()
