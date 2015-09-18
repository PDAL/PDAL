import pdal
from pdal import libpdalpython
# import pdal.libpdalpython
import unittest

def fetch_xml(filename):
    output = ''
    with open(filename, 'rb') as f:
        output = f.read().decode('UTF-8')
    return output

class TestPDALArray(unittest.TestCase):

  def test_construction(self):
    """Can we construct a PDAL pipeline"""
    xml = fetch_xml('../test/data/pipeline/pipeline_read.xml')
    r = libpdalpython.PyPipeline(xml)

  def test_execution(self):
    """Can we execute a PDAL pipeline"""
    xml = fetch_xml('../test/data/pipeline/pipeline_read.xml')
    r = libpdalpython.PyPipeline(xml)
    r.execute()
    self.assertEqual(len(r.xml), 2184)

  def test_array(self):
    """Can we fetch PDAL data as a numpy array"""
    xml = fetch_xml('../test/data/pipeline/pipeline_read.xml')
    r = libpdalpython.PyPipeline(xml)
    r.execute()
    arrays = r.arrays()
    self.assertEqual(len(arrays), 1)

    a = arrays[0]
    self.assertAlmostEqual(a[0][0], 637012.24, 7)
    self.assertAlmostEqual(a[1064][2], 423.92, 7)

  def test_merged_arrays(self):
    """Can we fetch merged PDAL data """
    xml = fetch_xml('../test/data/filters/chip.xml')
    r = libpdalpython.PyPipeline(xml)
    r.execute()
    arrays = r.arrays()
    self.assertEqual(len(arrays), 43)
    # data are going to all be a little different
    # due to sorting not being stable
#     for a in arrays:
#         self.assertAlmostEqual(a[0][0], 494057.30, 2)
#         self.assertAlmostEqual(a[0][2], 130.63, 2)
def test_suite():
    return unittest.TestSuite(
        [TestPDALArray])

if __name__ == '__main__':
    unittest.main()
