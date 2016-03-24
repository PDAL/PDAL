.. _filters.greedyprojection:

filters.greedyprojection
===============================================================================

The Greedy Projection filter passes data through the Point Cloud Library
(`PCL`_) GreedyProjectionTriangulation algorithm.

GreedyProjectionTriangulation is an implementation of a greedy triangulation
algorithm for 3D points based on local 2D projections. It assumes locally smooth
surfaces and relatively smooth transitions between areas with different point
densities.

.. _`PCL`: http://www.pointclouds.org

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.greedyprojection"
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }


Options
-------------------------------------------------------------------------------

None at the moment. Relying on defaults within PCL.
