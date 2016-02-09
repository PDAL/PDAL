import sys
DATADIRECTORY = sys.argv.pop()
print (DATADIRECTORY)
from test.test_libpdal import test_suite
from test.test_pipeline import test_suite
