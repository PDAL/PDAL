.. _filters.poisson:

===============================================================================
filters.poisson
===============================================================================

The Poisson filter passes data through the Point Cloud Library (`PCL`_) Poisson
surface reconstruction algorithm.

Poisson is an implementation of the method described in [Kazhdan2006]_.

.. [Kazhdan2006] Kazhdan, Michael, Matthew Bolitho, and Hugues Hoppe. "Poisson surface reconstruction." Proceedings of the fourth Eurographics symposium on Geometry processing. Vol. 7. 2006.

.. _`PCL`: http://www.pointclouds.org

Example
-------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "dense.las",
        {
          "type":"filters.poisson",
          "depth":"8",
          "point_weight":"4"
        },
        {
          "type":"writers.las",
          "filename":"thinned.las",
        }
      ]
    }


Options
-------------------------------------------------------------------------------

depth
  Maximum depth of the tree used for reconstruction. [Default: **8**]

point_weight
  Importance of interpolation of point samples in the screened Poisson equation. [Default: **4.0**]
