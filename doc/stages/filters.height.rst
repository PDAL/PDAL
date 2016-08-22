.. _filters.height:

filters.height
===============================================================================

The Height filter takes as input a point cloud with a Classification dimension,
with ground points assigned the classification label of 2 (per LAS
specification). It returns a point cloud with a new dimension ``HeightAboveGround`` that
contains the normalized height value.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.height"
        },
        {
          "type":"filters.ferry",
          "dimensions":"HeightAboveGround = Z",
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }

Options
-------------------------------------------------------------------------------

None
