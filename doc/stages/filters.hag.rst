.. _filters.hag:

filters.hag
===============================================================================

The Height Above Ground (HAG) filter takes as input a point cloud with a 
Classification dimension, with ground points assigned the classification label 
of 2 (per LAS specification). It returns a point cloud with a new dimension 
``HeightAboveGround`` that contains the normalized height value.

The HAG filter works by iterating through all non-ground points, finding the
nearest neighbor (in XY only) amongst the ground points, and computing the
distance between the two Z values.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.hag"
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
