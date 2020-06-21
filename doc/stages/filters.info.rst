.. _filters.info:

filters.info
======================

The **Info filter** provides simple information on a point set as metadata.
It is usually invoked by the info command, rather than by user code.
The data provided includes bounds, a count of points, dimension names,
spatial reference, and points meeting a query criteria.

.. embed::

.. streamable::

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.info",
          "point":"1-5"
      }
  ]

Options
-------

_`point`
  A comma-separated list of single point IDs or ranges of points.  For
  example "2-6, 10, 25" selects eight points from the input set.  The first
  point has an ID of 0.  The point_ option can't be used with the query_ option.
  [Default: no points are selected.]

_`query`
  A specification to retrieve points near a location.  Syntax of the the
  query is X,Y[,Z][/count] where 'X', 'Y' and 'Z' are coordinate
  locations mapping to the X, Y and Z point dimension and 'count' is the
  number of points to return.  If 'count' isn't specified, the 10 points
  nearest to the location are returned.  The query_ option can't be used
  with the point_ option. [Default: no points are selected.]

