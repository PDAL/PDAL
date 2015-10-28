.. _filters.range:

filters.range
======================

The range filter applies rudimentary filtering to the input point cloud
based on a set of criteria on the given dimensions.

Pipeline Example
----------------

This example passes through all points whose Z value is in the range [0,100]
and whose classification equals 2 (corresponding to ground in LAS).

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        filtered.las
      </Option>
      <Filter type="filters.range">
        <Option name="limits">
          Z[0:100],Classification[2:2]
        </Option>
        <Reader type="readers.las">
          <Option name="filename">
            input.las
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

Command-line Example
--------------------

The equivalent pipeline invoked via the PDAL ``translate`` command would be

.. code-block:: bash

  $ pdal translate -i input.las -o filtered.las -f range --filters.range.limits="Z[0:100],Classification[2:2]"

Options
-------

limits
  A comma-separated list of :ref:'ranges'.  If more than one range is
  specified for a dimension, the criteria are treated as being logically
  ORed together.  Ranges for different dimensions are treated as being
  logically ANDed.

  Example:
  --------
    .. code-block:: bash

      Classification[1:2], Red[1:50], Blue[25:75], Red[75:255],
      Classification[6:7]

    This specification will select points that have the classification of
    1, 2, 6 or 7 and have a blue value or 25-75 and have a red value of
    1-50 or 75-255.  In this case, all values are inclusive.

