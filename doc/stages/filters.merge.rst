.. _filters.merge:

filters.merge
===============================================================================

The merge filter combines input from multiple sources into a single output.
No checks are made to ensure that points from the various sources have similar
dimensions or are generally compatible.  Notably, dimensions are not
initialized when points merged from various sources do not have dimensions in
common.

Example
-------

.. code-block:: json

    {
      "pipeline": [
        {
          "filename": "/Users/hobu/dev/git/pdal/test/data/las/1.2-with-color.las",
          "tag": "A"
        },
        {
          "filename": "/Users/hobu/dev/git/pdal/test/data/las/1.2-with-color.las",
          "tag": "B"
        },
        {
          "type": "filters.merge",
          "inputs": ["A", "B"]
        },
        {
          "type": "writers.las",
          "filename": "output.las"
        }
      ]
    }

