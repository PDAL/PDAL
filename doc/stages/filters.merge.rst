.. _filters.merge:

filters.merge
===============================================================================

The merge filter combines input from multiple sources into a single output.
In most cases, this happens automatically on output and use of the merge
filter is unnecessary.
However, there may be special cases where merging points prior to a particular
filter or writer is necessary or desirable.  The merge filter will log a
warning if its input point sets are based on different spatial references.
No checks are made to ensure that points from various sources being merged
have similar dimensions or are generally compatible.  Notably, dimensions
are not initialized when points merged from various sources do not have
dimensions in common.

Example 1
---------

This pipeline will create an output file "output.las" that contcatantes
the points from "file1", "file2" and file3".  Note that the explicit
use of the merge filter is unnecessary in this case.

.. code-block:: json

    {
      "pipeline": [
        "file1",
        "file2",
        "file3",
        {
          "type": "filters.merge"
        },
        "output.las"
      ]
    }

Example 2
---------

.. code-block:: json

    {
      "pipeline": [
        
      ]
    }
