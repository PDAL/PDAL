.. _filters.poisson:

===============================================================================
filters.poisson
===============================================================================

The poisson filter passes data Mischa Kazhdan's poisson surface reconstruction
algorithm. [Kazhdan2006]_  It creates a watertight surface from the original
point set by creating an entirely new point set representing the imputed
isosurface.  The algorithm requires normal vectors to each point in order
to run.  If the x, y and z normal dimensions are present in the input point
set, they will be used by the algorithm.  If they don't exist, the poisson
filter will invoke the PDAL normal filter to create them before running.

The poisson algorithm will usually create a larger output point set
than the input point set.  Because the algorithm constructs new points, data
associated with the original points set will be lost, as the algorithm has
limited ability to impute associated data.  However, if color dimensions
(red, green and blue) are present in the input, colors will be reconstruced
in the output point set.

This integration of the algorithm with PDAL only supports a limited set of
the options available to the implementation.  If you need support for further
options, please let us know.

.. embed::

Example
-------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "dense.las",
        {
          "type":"filters.poisson",
        },
        {
          "type":"writers.ply",
          "filename":"isosurface.ply",
        }
      ]
    }


.. note::
    The algorithm is slow.  On a reasonable desktop machine, the surface
    reconstruction shown below took about 15 minutes.

.. figure:: ../images/poisson_points.png

  Point cloud (800,000 points)

.. figure:: ../images/poisson_edges.png

  Reconstruction (1.8 million vertices, 3.7 million faces)


Options
-------------------------------------------------------------------------------

density
  Write an estimate of neighborhood density for each point in the output
  set.

depth
  Maximum depth of the tree used for reconstruction. The output is sentsitve
  to this parameter.  Increase if the results appear unsatisfactory.
  [Default: **8**]
