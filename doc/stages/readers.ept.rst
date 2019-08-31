.. _readers.ept:

readers.ept
===========

`Entwine Point Tile`_ (EPT) is a hierarchical octree-based point cloud format
suitable for real-time rendering and lossless archival.  `Entwine`_ is a
producer of this format.  The EPT Reader supports reading data from the
EPT format, including spatially accelerated queries and file reconstruction
queries.

Sample EPT datasets of hundreds of billions of points in size may be viewed
with `Potree`_ or `Plasio`_.

.. embed::

Example
--------------------------------------------------------------------------------

This example downloads a small area around the the Statue of Liberty from the New York City data set (4.7 billion points) which can be viewed in its entirety in `Potree`_ or `Plasio`_.

.. code-block:: json

   [
      {
         "type": "readers.ept",
         "filename": "http://na.entwine.io/nyc/ept.json",
         "bounds": "([-8242669, -8242529], [4966549, 4966674])"
      },
      "statue-of-liberty.las"
   ]

Additional attributes created by the
:ref:`EPT addon writer <writers.ept_addon>` can be referenced with the ``addon`` option.  Here is an example that overrides the ``Classification`` dimension with an addon dimension derived from the original dataset:

.. code-block:: json

  [
      {
          "type": "readers.ept",
          "filename": "http://na.entwine.io/autzen/ept.json",
          "addons": { "Classification": "~/entwine/addons/autzen/smrf" }
      },
      {
          "type": "writers.las",
          "filename": "autzen-ept-smrf.las"
      }
  ]

For more details about addon dimensions and how to produce them, see :ref:`writers.ept_addon`.

Options
--------------------------------------------------------------------------------

filename
    EPT resource from which to read.  Because EPT resources do not have a file extension, to specify an EPT resource as a string, it must be prefixed with ``ept://``.  For example, ``pdal translate ept://http://na.entwine.io/autzen autzen.laz``. [Required]

spatialreference
    Spatial reference to apply to the data.  Overrides any SRS in the input
    itself.  Can be specified as a WKT, proj.4 or EPSG string. [Default: none]

bounds
    The extents of the resource to select in 2 or 3 dimensions, expressed as a string, e.g.: ``([xmin, xmax], [ymin, ymax], [zmin, zmax])``.  If omitted, the entire dataset will be selected.

resolution
    A point resolution limit to select, expressed as a grid cell edge length.  Units correspond to resource coordinate system units.  For example, for a coordinate system expressed in meters, a ``resolution`` value of ``0.1`` will select points up to a ground resolution of 100 points per square meter.

    The resulting resolution may not be exactly this value: the minimum possible resolution that is at *least* as precise as the requested resolution will be selected.  Therefore the result may be a bit more precise than requested.

addons
    A mapping of assignments of the form ``DimensionName: AddonPath``, which
    assigns dimensions from the specified paths to the named dimensions.
    These addon dimensions are created by the
    :ref:`EPT addon writer <writers.ept_addon>`.  If the dimension names
    already exist in the EPT `Schema`_ for the given resource, then their
    values will be overwritten with those from the appropriate addon.

    Addons may used to override well-known :ref:`dimension <dimensions>`.  For example, an addon assignment of ``"Classification": "~/addons/autzen/MyGroundDimension/"`` will override an existing EPT ``Classification`` dimension with the custom dimension.

origin
    EPT datasets are lossless aggregations of potentially multiple source
    files.  The *origin* options can be used to select all points from a
    single source file.  This option may be specified as a string or an
    integral ID.

    The string form of this option selects a source file by its original
    file path.  This may be a substring instead of the entire path, but
    the string must uniquely select only one source file (via substring
    search).  For example, for an EPT dataset created from source files
    *one.las*, *two.las*, and *two.bpf*, "one" is a sufficient selector,
    but "two" is not.

    The integral form of this option selects a source file by its ``OriginId``
    dimension, which can be determined from  the file's position in EPT
    metadata file ``entwine-files.json``.

polygon
  The clipping polygon, expressed in a well-known text string,
  eg: "POLYGON((0 0, 5000 10000, 10000 0, 0 0))".  This option can be
  specified more than once by placing values in an array.


threads
    Number of worker threads used to download and process EPT data.  A
    minimum of 4 will be used no matter what value is specified.

.. _Entwine Point Tile: https://entwine.io/entwine-point-tile.html
.. _Entwine: https://entwine.io/
.. _Potree: http://potree.entwine.io/data/nyc.html
.. _Plasio: http://speck.ly/?s=http%3A%2F%2Fc%5B0-7%5D.greyhound.io&r=ept%3A%2F%2Fna.entwine.io%2Fnyc&ca=-0&ce=49.06&ct=-8239196%2C4958509.308%2C337&cd=42640.943&cmd=125978.13&ps=2&pa=0.1&ze=1&c0s=remote%3A%2F%2Fimagery%3Furl%3Dhttp%3A%2F%2Fserver.arcgisonline.com%2FArcGIS%2Frest%2Fservices%2FWorld_Imagery%2FMapServer%2Ftile%2F%7B%7Bz%7D%7D%2F%7B%7By%7D%7D%2F%7B%7Bx%7D%7D.jpg
.. _Schema: https://entwine.io/entwine-point-tile.html#schema

headers
    HTTP headers to forward for remote EPT endpoints, structured as a JSON
    object of key/value string pairs.

query
    HTTP query parameters to forward for remote EPT endpoints, structured as a
    JSON object of key/value string pairs.
