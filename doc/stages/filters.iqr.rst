.. _filters.iqr:

filters.iqr
===============================================================================

The ``filters.iqr`` filter automatically crops the input point cloud based on
the distribution of points in the specified dimension. Specifically, we choose
the method of Interquartile Range (IQR). The IQR is defined as the range between
the first and third quartile (25th and 75th percentile). Upper and lower bounds
are determined by adding 1.5 times the IQR to the third quartile or subtracting
1.5 times the IQR from the first quartile. The multiplier, which defaults to
1.5, can be adjusted by the user.

.. note::

  This method can remove real data, especially ridges and valleys in rugged
  terrain, or tall features such as towers and rooftops in flat terrain. While
  the number of deviations can be adjusted to account for such content-specific
  considerations, it must be used with care.

.. embed::

Example
-------

The sample pipeline below uses ``filters.iqr`` to automatically crop the Z
dimension and remove possible outliers. The multiplier to determine high/low
thresholds has been adjusted to be less agressive and to only crop those
outliers that are greater than the third quartile plus 3 times the IQR or are
less than the first quartile minus 3 times the IQR.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.iqr",
          "dimension":"Z",
          "k":3.0
        },
        "output.laz"
      ]
    }

Options
-------------------------------------------------------------------------------

k
  The IQR multiplier used to determine upper/lower bounds. [Default: **1.5**]

dimension
  The name of the dimension to filter.
