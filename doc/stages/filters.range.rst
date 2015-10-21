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
  A comma-separated list of dimension names and limits, given in the following
  form.

  .. code-block:: bash

    dimension-name(lower-bound:upper-bound)

  In this example, we have used parenthesis to enclose the bounds, thus
  indicating that the bounds shall be interpreted as exclusive. To mark either
  the upper or lower bound as inclusive, simply replace the corresponding
  parenthesis with a square bracket. For example,

  .. code-block:: bash

    dimension-name(lower-bound:upper-bound]

  where the lower bound remains exclusive, but the upper bound is inclusive.

  Either bound may also be omitted to indicate that there is no bound. In the
  following example, points with a Z value of 10 or greater are retained.

  .. code-block:: bash

    Z[10:]

  Currently, exact values can be matched by repeating the exact value as both
  the upper and lower bound. Below, only points with a Classification of 2 are
  retained.

  .. code-block:: bash

    Classification[2:2]

  Any range can be negated by prefacing with the '!' character.  The following
  will select all classifications that aren't equal to the value 2.

  .. code-block:: bash

    Classification![2:2]
