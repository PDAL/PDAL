.. _filters.range:

filters.range
======================

.. contents::

The range filter applies rudimentary filtering to the input point cloud
based on a set of criteria on the given dimensions.

.. embed::

.. streamable::

Pipeline Example
----------------

This example passes through all points whose Z value is in the range [0,100]
and whose classification equals 2 (corresponding to ground in LAS).


.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.range",
          "limits":"Z[0:100],Classification[2:2]"
        },
        {
          "type":"writers.las",
          "filename":"filtered.las"
        }
      ]
    }


Command-line Example
--------------------

The equivalent pipeline invoked via the PDAL ``translate`` command would be

.. code-block:: bash

  $ pdal translate -i input.las -o filtered.las -f range --filters.range.limits="Z[0:100],Classification[2:2]"

Options
-------

limits
  A comma-separated list of :ref:`ranges`.  If more than one range is
  specified for a dimension, the criteria are treated as being logically
  ORed together.  Ranges for different dimensions are treated as being
  logically ANDed.

  Example:

  ::

    Classification[1:2], Red[1:50], Blue[25:75], Red[75:255], Classification[6:7]

  This specification will select points that have the classification of
  1, 2, 6 or 7 and have a blue value or 25-75 and have a red value of
  1-50 or 75-255.  In this case, all values are inclusive.


.. _ranges:

Ranges
--------------------------------------------------------------------------------

A range specification is a dimension name, followed by an optional negation
character ('!'), and a starting and ending value separated by a colon,
surrounded by parentheses or square brackets.  Either the starting or ending
values can be omitted.  Parentheses indicate an open endpoint that doesn't
include the adjacent value.  Square brackets indicate a closed endpoint
that includes the adjacent value.

Example 1:
................................................................................

::

  Z[10:]

Selects all points with a Z value greater than or equal to 10.

Example 2:
................................................................................

::

  Classification[2:2]

Selects all points with a classification of 2.

Example 3:
................................................................................

::

  Red!(20:40]

Selects all points with red values less than or equal to 20 and those with
values greater than 40

Example 4:
................................................................................

::

  Blue[:255)

Selects all points with a blue value less than 255.

Example 5:
................................................................................

::

  Intensity![25:25]

Selects all points with an intensity not equal to 25.
