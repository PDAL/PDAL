.. _filters.zsmooth:

filters.zsmooth
===============================================================================

The **Zsmooth Filter** computes a new Z value as another dimension that is based
on the Z values of neighboring points.

All points within some distance in the X-Y plane from a reference point are ordered by Z value.
The reference point's new smoothed Z value is chosen to be that of the Nth median value of
the neighboring points, where N is specified as the _`medianpercent` option.

Use :ref:`filters.assign` to assign the smoothed Z value to the actual Z dimension if
desired.

Example
-------

Compute the smoothed Z value as the median Z value of the neighbors within 2 units and
assign the value back to the Z dimension.

.. code_block::json

    [
        "input.las",
        {
            "type": "filters.zsmooth",
            "radius": 2,
            "dim": "Zadj"
        },
        {
            "type": "filters.assign",
            "value": "Z = Zadj"
        },
        "output.las"
    ]

Options
-------------------------------------------------------------------------------

radius
  All points within `radius` units from the reference point in the X-Y plane are considered
  to determine the smoothed Z value. [Default: 1]

medianpercent
  A value between 0 and 100 that specifies the relative position of ordered Z values of neighbors
  to use as the new smoothed Z value. 0 specifies the minimum value. 100 specifies the
  maximum value. 50 specifies the mathematical median of the values. [Default: 50]

dim
  The name of a dimension to use for the adjusted Z value. Cannot be 'Z'. [Required]

.. include:: filter_opts.rst

