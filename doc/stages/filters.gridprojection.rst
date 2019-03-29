.. _filters.gridprojection:

===============================================================================
filters.gridprojection
===============================================================================

The **Grid Projection Filter** passes data through the
Point Cloud Library `GridProjection <http://docs.pointclouds.org/1.7.1/classpcl_1_1_grid_projection.html>`_ algorithm.

GridProjection is an implementation of the surface reconstruction method
described in [Li2010]_.

.. plugin::

Example
-------------------------------------------------------------------------------

.. code-block:: json

  [
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

Options
-------------------------------------------------------------------------------

None.
