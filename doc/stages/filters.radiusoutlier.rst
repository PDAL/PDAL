.. _filters.radiusoutlier:

===============================================================================
filters.radiusoutlier
===============================================================================

The Radius Outlier filter passes data through the Point Cloud Library (`PCL`_)
RadiusOutlierRemoval algorithm.

RadiusOutlierRemoval filters points in a cloud based on the number of neighbors
they have. Iterates through the entire input once, and for each point, retrieves
the number of neighbors within a certain radius. The point will be considered an
outlier if it has too few neighbors, as determined by ``min_neighbors``. The
radius can be changed using ``radius``.

.. _`PCL`: http://www.pointclouds.org

Example
-------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.radiusoutlier",
          "min_neighbors":"4"
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }


Options
-------------------------------------------------------------------------------

min_neighbors
  Minimum number of neighbors in radius. [Default: **2**]

radius
  Radius. [Default: **1.0**]

classify
  Apply classification labels? [Default: **true**]

extract
  Extract ground returns? [Default: **false**]
