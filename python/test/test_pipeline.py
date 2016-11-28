import unittest
import pdal
import os

DATADIRECTORY = os.environ.get('PDAL_TEST_DIR')
if not DATADIRECTORY:
    DATADIRECTORY = "../test"

bad_json = u"""
{
  "pipeline": [
    "nofile.las",
    {
        "type": "filters.sort",
        "dimension": "X"
    }
  ]
}
"""

class TestPipeline(unittest.TestCase):

    DATADIRECTORY = os.environ.get('PDAL_TEST_DIR')
    if not DATADIRECTORY:
        DATADIRECTORY = "../test"
    def fetch_json(self, filename):
        import os
        fn = DATADIRECTORY + os.path.sep +  filename
        output = ''
        with open(fn, 'rb') as f:
            output = f.read().decode('UTF-8')
        return output

    @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, 'data/pipeline/sort.json')),
                         os.path.join(DATADIRECTORY, 'data/pipeline/sort.json'))
    def test_construction(self):
        """Can we construct a PDAL pipeline"""
        json = self.fetch_json('/data/pipeline/sort.json')
        r = pdal.Pipeline(json)

    @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, 'data/pipeline/sort.json')),
                           "missing test data")
    def test_execution(self):
        """Can we execute a PDAL pipeline"""
        x = self.fetch_json('/data/pipeline/sort.json')
        r = pdal.Pipeline(x)
        r.execute()
        self.assertGreater(len(r.pipeline), 200)

    def test_validate(self):
        """Do we complain with bad pipelines"""
        r = pdal.Pipeline(bad_json)
        with self.assertRaises(RuntimeError):
            r.validate()

    @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, 'data/pipeline/sort.json')),
                         "missing test data")
    def test_array(self):
        """Can we fetch PDAL data as a numpy array"""
        json = self.fetch_json('/data/pipeline/sort.json')
        r = pdal.Pipeline(json)
        r.execute()
        arrays = r.arrays
        self.assertEqual(len(arrays), 1)

        a = arrays[0]
        self.assertAlmostEqual(a[0][0], 635619.85, 7)
        self.assertAlmostEqual(a[1064][2], 456.92, 7)

    def test_metadata(self):
        """Can we fetch PDAL metadata"""
        json = self.fetch_json('/data/pipeline/sort.json')
        r = pdal.Pipeline(json)
        r.execute()
        metadata = r.metadata
        import json
        j = json.loads(metadata)
        self.assertEqual(j["metadata"]["readers.las"]["count"], 1065)


    def test_no_execute(self):
        """Does fetching arrays without executing throw an exception"""
        json = self.fetch_json('/data/pipeline/sort.json')
        r = pdal.Pipeline(json)
        with self.assertRaises(RuntimeError):
            r.arrays

    def test_logging(self):
        """Can we fetch log output"""
        json = self.fetch_json('/data/pipeline/reproject.json')
        r = pdal.Pipeline(json)
        r.loglevel = 8
        count = r.execute()
        self.assertEqual(count, 789)
        self.assertEqual(r.log.split()[0], '(pypipeline')

    def test_schema(self):
        """Fetching a schema works"""
        json = self.fetch_json('/data/pipeline/sort.json')
        r = pdal.Pipeline(json)
        r.execute()
        self.assertEqual(r.schema['schema']['dimensions'][0]['name'], 'X')

    @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, 'data/filters/chip.json')),
                           "missing test data")
    def test_merged_arrays(self):
        """Can we fetch multiple point views from merged PDAL data """
        json = self.fetch_json('/data/filters/chip.json')
        r = pdal.Pipeline(json)
        r.execute()
        arrays = r.arrays
        self.assertEqual(len(arrays), 43)

def test_suite():
    return unittest.TestSuite(
        [TestXML])

if __name__ == '__main__':
    unittest.main()
