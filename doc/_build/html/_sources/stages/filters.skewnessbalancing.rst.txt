.. _filters.skewnessbalancing:

filters.skewnessbalancing
===============================================================================

**Skewness Balancing** classifies ground points based on the approach outlined
in [Bartels2010]_.

.. embed::

.. note::

    For Skewness Balancing to work well, the scene being processed needs to be
    quite flat, otherwise many above ground features will begin to be included
    in the ground surface.

Example
-------

The sample pipeline below uses the Skewness Balancing filter to segment ground
and non-ground returns, using default options, and writing only the ground
returns to the output file.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.skewnessbalancing"
      },
      {
          "type":"filters.range",
          "limits":"Classification[2:2]"
      },
      "output.laz"
  ]

Options
-------------------------------------------------------------------------------

.. include:: filter_opts.rst

.. note::

    The Skewness Balancing method is touted as being threshold-free. We may
    still in the future add convenience parameters that are common to other
    ground segmentation filters, such as ``returns`` or ``ignore`` to limit the
    points under consideration for filtering.
