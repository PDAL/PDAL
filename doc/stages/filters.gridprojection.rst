.. _filters.gridprojection:

===============================================================================
filters.gridprojection
===============================================================================

The Grid Projection filter passes data through the Point Cloud Library (`PCL`_)
GridProjection algorithm.

GridProjection is an implementation of the surface reconstruction method
described in [Li2010]_.

.. _`PCL`: http://www.pointclouds.org

.. plugin::

Example
-------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.gridprojection"
        },
        {
          "type":"writers.ply",
          "faces":true,
          "filename":"output.ply"
        }
      ]
    }


Options
-------------------------------------------------------------------------------

None at the moment. Relying on defaults within PCL.
