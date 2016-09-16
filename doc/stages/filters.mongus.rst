.. _filters.mongus:

filters.mongus
===============================================================================

Filter ground returns using the approach outlined in [Mongus2012]_.

.. note::
  
  Our implmentation of Mongus is in an alpha state. We'd love to have you kick
  the tires and provide feedback, but do not plan on using this in production.
  
The current implementation of ``filters.mongus`` differs slightly from the
original paper. We weren't too happy with the criteria for how control points at
the current level are compared against the TPS at the previous scale and were
exploring some alternate metrics.

Some warts about the current implementation:

* It writes a bunch of intermediate/debugging outputs to the current directory
  while processing. This should be made optional and then eventually go away.
  
* We require specification of a max level, whereas the original paper 
  automatically determined an appropriate max level.
  
.. [Mongus2012] Mongus, D., Zalik, B., 2012. Parameter-free ground filtering of LiDAR data for automatic DTM generation. ISPRS J. Photogramm. Remote Sens. 67, 1–12.

Example
-------

The sample pipeline below uses ``filters.mongus`` to segment ground and
non-ground returns, writing only the ground returns to the output file.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.mongus",
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
  
extract
  Extract ground returns (non-ground returns are cropped)? [Default: **false**]
  
k
  Standard deviation multiplier to be used when thresholding values. [Default: **3.0**]
  
l
  Maximum level in the hierarchical decomposition. [Default: **8**]
