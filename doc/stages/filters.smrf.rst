.. _filters.smrf:

filters.smrf
===============================================================================

Filter ground returns using the Simple Morphological Filter (SMRF) approach
outlined in [Pingel2013]_.

.. [Pingel2013] Pingel, T.J., Clarke, K.C., McBride, W.A., 2013. An improved simple morphological filter for the terrain classification of airborne LIDAR data. ISPRS J. Photogramm. Remote Sens. 77, 21–30.


.. embed::

Example
-------

The sample pipeline below uses ``filters.smrf`` to segment ground and non-ground
returns, writing only the ground returns to the output file.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.smrf"
        },
        {
          "type":"filters.range",
          "limits":"Classification[2:2]"
        },
        "output.laz"
      ]
    }

Options
-------------------------------------------------------------------------------

cell
  Cell size. [Default: **1.0**]

cut
  Cut net size (``cut=0`` skips the net cutting step). [Default: **0.0**]

dir
  Optional output directory for debugging intermediate rasters.

scalar
  Elevation scalar. [Default: **1.25**]

slope
  Slope (rise over run). [Default: **0.15**]

threshold
  Elevation threshold. [Default: **0.5**]

window
  Max window size. [Default: **18.0**]
