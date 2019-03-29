.. _filters.overlay:

filters.overlay
===================

The **overlay filter** allows you to set the values of a selected dimension
based on an OGR-readable polygon or multi-polygon.

.. embed::

OGR SQL support
----------------

You can limit your queries based on OGR's SQL support. If the
filter has both a datasource_ and a query_ option, those will
be used instead of the entire OGR data source. At this time it is
not possible to further filter the OGR query based on a geometry
but that may be added in the future.

.. note::

    The OGR SQL support follows the rules specified in `ExecuteSQL`_
    documentation, and it will pass SQL down to the underlying
    datasource if it can do so.

.. _`ExecuteSQL`: http://www.gdal.org/ogr__api_8h.html#a9892ecb0bf61add295bd9decdb13797a

Example 1
---------

In this scenario, we are altering the attributes of the dimension
``Classification``.  Points from autzen-dd.las that lie within a feature will
have their classification to match the ``CLS`` field associated with that
feature.

.. code-block:: json

  [
      "autzen-dd.las",
      {
          "type":"filters.overlay",
          "dimension":"Classification",
          "datasource":"attributes.shp",
          "layer":"attributes",
          "column":"CLS"
      },
      {
          "filename":"attributed.las",
          "scale_x":0.0000001,
          "scale_y":0.0000001
      }
  ]


Example 2
--------------------------------------------------------------------------------

This example sets the Intensity attribute to ``CLS`` values read from the
`OGR SQL`_ query.

.. _`OGR SQL`: http://www.gdal.org/ogr_sql_sqlite.html

.. code-block:: json

  [
      "autzen-dd.las",
      {
          "type":"filters.overlay",
          "dimension":"Intensity",
          "datasource":"attributes.shp",
          "query":"SELECT CLS FROM attributes where cls!=6",
          "column":"CLS"
      },
      "attributed.las"
  ]

Options
-------

dimension
  Name of the dimension whose value should be altered.  [Required]

_`datasource`
  OGR-readable datasource for Polygon or MultiPolygon data.  [Required]

column
  The OGR datasource column from which to read the attribute.
  [Default: first column]

_`query`
  OGR SQL query to execute on the datasource to fetch geometry and attributes.
  The entire layer is fetched if no query is provided.  [Default: none]

layer
  The data source's layer to use. [Defalt: first layer]

