.. _readers.tindex:

readers.tindex
================================================================================


A `GDAL tile index`_ is an `OGR`_-readable data source of boundary information.
PDAL provides a similar concept for PDAL-readable point cloud data. You can use
the :ref:`tindex_command` application to generate tile index files in any
format that `OGR`_ supports writing. Once you have the tile index, you can then
use the :ref:`readers.tindex` driver to automatically merge and query the
data described by the tiles.

.. _`GDAL`: http://gdal.org
.. _`OGR`: http://gdal.org/ogr/
.. _`GDAL tile index`: http://www.gdal.org/gdaltindex.html



.. embed::


Basic Example
--------------------------------------------------------------------------------

Given a tile index that was generated with the following scenario:

::

    pdal tindex index.sqlite \
        "/Users/hobu/dev/git/pdal/test/data/las/interesting.las" \
        -f "SQLite" \
        --lyr_name "pdal" \
        --t_srs "EPSG:4326"

Use the following :ref:`pipeline` example to read and automatically
merge the data.


.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.tindex",
          "filter_srs":"+proj=lcc +lat_1=43 +lat_2=45.5 +lat_0=41.75 +lon_0=-120.5 +x_0=399999.9999999999 +y_0=0 +ellps=GRS80 +units=ft +no_defs",
          "filename":"index.sqlite",
          "where":"location LIKE \'%nteresting.las%\'",
          "polygon":"POLYGON ((635629.85000000 848999.70000000, 635629.85000000 853535.43000000, 638982.55000000 853535.43000000, 638982.55000000 848999.70000000, 635629.85000000 848999.70000000))"
        },
        {
          "type":"writers.las",
          "filename":"outputfile.las"
        }
      ]
    }


Options
--------------------------------------------------------------------------------

filename
  OGROpen'able raster file to read [Required]

lyr_name
  The OGR layer name for the data source to use to
  fetch the tile index information.

srs_column
  The column in the layer that provides the SRS
  information for the file. Use this if you wish to
  override or set coordinate system information for
  files.

tindex_name
  The column name that defines the file location for
  the tile index file.
  [Default: **location**]

sql
  `OGR SQL`_ to use to define the tile index layer.

bounds
  A 2D box to pre-filter the tile index. If it is set,
  it will override any ``wkt`` option.

wkt
  A geometry to pre-filter the tile index using
  OGR

t_srs
  Reproject the layer SRS, otherwise default to the
  tile index layer's SRS.

filter_srs
  Transforms any ``wkt`` or ``boundary`` option to this
  coordinate system before filtering or reading data.

where
  `OGR SQL`_ filter clause to use on the layer. It only
  works in combination with tile index layers that are
  defined with ``lyr_name``

dialect
  `OGR SQL`_ dialect to use when querying tile index layer
  [Default: OGRSQL]

count
  Maximum number of points to read [Optional]

.. _`OGR SQL`: http://www.gdal.org/ogr_sql.html



