.. _filters.merge:

filters.merge
===============================================================================

The merge filter combines input from multiple sources into a single output.
In most cases, this happens automatically on output and use of the merge
filter is unnecessary.  However, there may be special cases where
merging points prior to a particular filter or writer is necessary
or desirable.

The merge filter will log a warning if its input point sets are based on
different spatial references.  No checks are made to ensure that points
from various sources being merged have similar dimensions or are generally
compatible.  Notably, dimensions are not initialized when points merged
from various sources do not have dimensions in common.

.. embed::

Example 1
---------

This pipeline will create an output file "output.las" that contcatenates
the points from "file1", "file2" and "file3".  Note that the explicit
use of the merge filter is unnecessary in this case (removing the merge
filter will yield the same result).

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

Here are a pair of unlikely pipelines that show one way in which a merge filter
might be used.  The first pipeline simply reads the input files "utm1.las",
"utm2.las" and "utm3.las".  Since the points from each input set are
carried separately through the pipeline, three files are created as output,
"out1.las", "out2.las" and "out3.las".  "out1.las" contains the points
in "utm1.las".  "out2.las" contains the points in "utm2.las" and "out3.las"
contains the points in "utm3.las".

.. code-block:: json

    {
      "pipeline": [
        "utm1.las"
        "utm2.las",
        "utm3.las",
        "out#.las"
      ]
    }

Here is the same pipeline with a merge filter added.  The merge filter will
combine the points in its input: "utm1.las" and "utm2.las".  Then the result
of the merge filter is passed to the writer along with "utm3.las".  This
results in two output files: "out1.las" contains the points from "utm1.las"
and "utm2.las", while "out2.las" contains the points from "utm3.las".

.. code-block:: json

    {
      "pipeline": [
        "utm1.las"
        "utm2.las",
        {
            "type" : "filters.merge"
        },
        "utm3.las",
        "out#.las"
      ]
    }

