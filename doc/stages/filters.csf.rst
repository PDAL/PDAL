.. _filters.csf:

filters.csf
===============================================================================

The **Cloth Simulation Filter (CSF)** classifies ground points based on the
approach outlined in [Zhang2016]_.

.. embed::

Example
-------

The sample pipeline below uses CSF to segment ground and non-ground returns,
using default options, and writing only the ground returns to the output file.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.csf"
      },
      {
          "type":"filters.range",
          "limits":"Classification[2:2]"
      },
      "output.laz"
  ]

Options
-------------------------------------------------------------------------------

resolution
  Cloth resolution. [Default: **1.0**]

ignore
  A :ref:`range <ranges>` of values of a dimension to ignore.

returns
  Return types to include in output.  Valid values are "first", "last",
  "intermediate" and "only". [Default: **"last, only"**]

threshold
  Classification threshold. [Default: **0.5**]

smooth
  Perform slope post-processing? [Default: **true**]

step
  Time step. [Default: **0.65**]

rigidness
  Rigidness. [Default: **3**]

iterations
  Maximum number of iterations. [Default: **500**]
