.. _filters.smrf:

filters.smrf
===============================================================================

The **Simple Morphological Filter (SMRF)** classifies ground points based
on the approach outlined in [Pingel2013]_.

.. embed::

Example #1
----------

The sample pipeline below uses the SMRF filter to segment ground and non-ground
returns, using default options, and writing only the ground returns to the
output file.

.. code-block:: json

  [
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

Example #2
----------

A more complete example, specifying some options. These match the
optimized parameters for Sample 1 given in Table 3 of [Pingel2013]_.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.smrf",
          "scalar":1.2,
          "slope":0.2,
          "threshold":0.45,
          "window":16.0
      },
      {
          "type":"filters.range",
          "limits":"Classification[2:2]"
      }
      "output.laz"
  ]

Options
-------------------------------------------------------------------------------

cell
  Cell size. [Default: 1.0]

classbits
  Selectively ignore points marked as "synthetic", "keypoint", or "withheld".
  [Default: empty string, use all points]

cut
  Cut net size (``cut=0`` skips the net cutting step). [Default: 0.0]

dir
  Optional output directory for debugging intermediate rasters.

ignore
  A :ref:`range <ranges>` of values of a dimension to ignore.

returns
  Return types to include in output.  Valid values are "first", "last",
  "intermediate" and "only". [Default: "last, only"]

scalar
  Elevation scalar. [Default: **1.25**]

slope
  Slope (rise over run). [Default: **0.15**]

threshold
  Elevation threshold. [Default: **0.5**]

window
  Max window size. [Default: **18.0**]

.. include:: filter_opts.rst

