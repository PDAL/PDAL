.. _filters.mongus:

filters.mongus
===============================================================================

The **mongus filter** determines ground returns using the approach
outlined in [Mongus2012]_.

.. note::

    The mongus filter is deprecated and has been replaced by
    :ref:`filters.pmf` and :ref:`filters.smrf`.

The current implementation of this filter differs slightly from the
original paper. We weren't too happy with the criteria for how control
points at
the current level are compared against the TPS at the previous scale and were
exploring some alternate metrics.

Some warts about the current implementation:

* It writes many intermediate/debugging outputs to the current directory
  while processing.

* We require the specification of a max level, whereas the original paper
  automatically determined an appropriate max level.

.. embed::

Example
-------

The sample pipeline below uses the filter to segment ground and
non-ground returns, writing only the ground returns to the output file.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.mongus",
          "extract":true
      },
      "output.laz"
  ]

Options
-------------------------------------------------------------------------------

cell
  Cell size. [Default: 1.0]

classify
  Apply classification labels (i.e., ground = 2)? [Default: true]

extract
  Extract ground returns (non-ground returns are cropped)? [Default: false]

k
  Standard deviation multiplier to be used when thresholding
  values. [Default: 3.0]

l
  Maximum level in the hierarchical decomposition. [Default: 8]
