.. _filters.smrf:

filters.smrf
===============================================================================

Filter ground returns using the Simple Morphological Filter (SMRF) approach
outlined in [Pingel2013]_.

.. note::
  
  Our implmentation of SMRF is in an alpha state. We'd love to have you kick
  the tires and provide feedback, but do not plan on using this in production.
  
The current implementation of ``filters.smrf`` differs slightly from the
original paper. We weren't too happy with the performance of (our implementation
of) the inpainting routine, so we started exploring some other methods.

Some warts about the current implementation:

* It writes a bunch of intermediate/debugging outputs to the current directory
  while processing. This should be made optional and then eventually go away.
  
.. [Pingel2013] Pingel, T.J., Clarke, K.C., McBride, W.A., 2013. An improved simple morphological filter for the terrain classification of airborne LIDAR data. ISPRS J. Photogramm. Remote Sens. 77, 21–30.

Example
-------

The sample pipeline below uses ``filters.smrf`` to segment ground and non-ground
returns, writing only the ground returns to the output file.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.smrf",
          "extract":true
        },
        "output.laz"
      ]
    }

Options
-------------------------------------------------------------------------------

cell
  Cell size. [Default: **1.0**]

classify
  Apply classification labels (i.e., ground = 2)? [Default: **true**]

cut
  Cut net size (``cut=0`` skips the net cutting step). [Default: **0.0**]
  
extract
  Extract ground returns (non-ground returns are cropped)? [Default: **false**]
  
slope
  Slope (rise over run). [Default: **0.15**]
  
threshold
  Elevation threshold. [Default: **0.15**]
  
window
  Max window size. [Default: **21.0**]
