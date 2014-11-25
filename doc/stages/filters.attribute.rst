.. _filters.attribute:

filters.attribute
===================

The attribute filter allows you to set the values of a
selected dimension. There are three possible scenarios that
are currently supported:

* Set all points in a dimension to single value

* Set points inside an OGR-readable Polygon or MultiPolygon

OGR SQL support
----------------

You can limit your queries based on OGR's SQL support. If the
filter has both a `datasource` and a `query` option, those will
be used instead of the entire OGR data source. At this time it is
not possible to further filter the OGR query based on a geometry
but that may be added in the future.

.. note::

    The OGR SQL support follows the rules specified in `ExecuteSQL`_
    documentation, and it will pass SQL down to the underlying
    datasource if it can do so.

.. _`ExecuteSQL`: http://www.gdal.org/ogr__api_8h.html#a9892ecb0bf61add295bd9decdb13797a


.. warning::

    If no `datasource` is given, it is assumed that the query
    is setting all of the values for the given dimension to the
    `value` option.

Example
-------

In this scenario, we are altering the attributes of three
dimensions -- `Classification`, `PointSourceId`, and `Intensity`. The
`PointSourceId` will simply be set to the single value `26`, while
`Classification` and `Intensity` will have their values set when the
candidate point is inside a polygon with that polygon's attribute.

.. code-block:: xml

    <?xml version="2.0" encoding="utf-8"?>
    <Pipeline version="1.0">
        <Writer type="writers.las">
            <Option name="filename">
                attributed.las
            </Option>
            <Option name="scale_x">
                0.0000001
            </Option>
            <Option name="scale_y">
                0.0000001
            </Option>
            <Filter type="filters.attribute">
                <Option name="dimension">
                    Classification
                    <Options>
                        <Option name="datasource">
                            ./test/data/autzen/attributes.shp
                        </Option>
                        <Option name="layer">
                            attributes
                        </Option>
                        <Option name="column">
                            CLS
                        </Option>
                    </Options>
                </Option>
                <Option name="dimension">
                    Intensity
                    <Options>
                        <Option name="datasource">
                            ./test/data/autzen/attributes.shp
                        </Option>
                        <Option name="query">
                            SELECT CLS FROM attributes where cls!=6
                        </Option>
                        <Option name="column">
                            CLS
                        </Option>
                    </Options>
                </Option>
                <Option name="dimension">
                    PointSourceId
                    <Options>
                        <Option name="value">
                            26
                        </Option>
                    </Options>
                </Option>
                <Reader type="readers.las">
                    <Option name="filename">
                        ../autzen/autzen-dd.las
                    </Option>
                </Reader>
            </Filter>
        </Writer>
    </Pipeline>


Options
-------

dimension
  A dimension Option with an Options block containing at least a "value"
  Option to set all points.

  * value: The value to set all points to for the given dimension

  * datasource: OGR-readable datasource for Polygon or MultiPolygon data

  * column: The column from which to read the attribute. If none is
    specified, the first column is used.

  * query: The OGR SQL query to execute on the datasource to fetch
    geometry and attributes.

  * layer: The data source's layer to use. If none is specified, the
    first one is used.

