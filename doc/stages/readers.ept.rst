.. _readers.ept:

readers.ept
===========

`Entwine Point Tile`_ (EPT) is a hierarchical octree-based point cloud format suitable for real-time rendering and lossless archival.  `Entwine`_ is a producer of this format.  The **EPT Reader** supports reading data from the EPT format, including spatially accelerated queries and file reconstruction queries.

Sample EPT datasets of hundreds of billions of points in size may be viewed at http://potree.entwine.io and http://speck.ly.

.. embed::

Example
--------------------------------------------------------------------------------

This example downloads a small area around the the Statue of Liberty from the New York City data set (4.7 billion points) which can be viewed in its entirety in `Potree`_ or `Plasio`_.

.. code-block:: json

    {
      "pipeline": [
        {
          "type": "readers.ept",
          "filename": "http://na.entwine.io/nyc",
          "bounds": "([-8242669, -8242529], [4966549, 4966674])"
        },
        "statue-of-liberty.las"
      ]
    }


Options
--------------------------------------------------------------------------------

filename
    EPT resource from which to read.  Because EPT resources do not have a file extension, to specify an EPT resource as a string, it must be prefixed with ``ept://``.  For example, ``pdal translate ept://http://na.entwine.io/autzen autzen.laz``. [Required]

.. include:: reader_opts.rst

bounds
    The extents of the resource to select in 2 or 3 dimensions, expressed as a string, e.g.: ``([xmin, xmax], [ymin, ymax], [zmin, zmax])``.  If omitted, the entire dataset will be selected.

origin
    EPT datasets are lossless aggregations of potentially multiple source files.  The *origin* options can be used to select all points from a single source file.  This option may be specified as a string or an integral ID.

    The string form of this option selects a source file by its original file path.  This may be a substring instead of the entire path, but the string must uniquely select only one source file (via substring search).  For example, for an EPT dataset created from source files *one.las*, *two.las*, and *two.bpf*, `"one"` is a sufficient selector, but `"two"` is not.

    The integral form of this option selects a source file by its ``OriginId`` dimension, which can be found via the files position in EPT metadata file ``entwine-files.json``.

threads
    Number of worker threads used to download and process EPT data.  A minimum of 4 will be used no matter what value is specified.

.. _Entwine Point Tile: https://github.com/connormanning/entwine/blob/master/doc/entwine-point-tile.md
.. _Entwine: https://entwine.io/
.. _Potree: http://potree.entwine.io/data/nyc.html
.. _Plasio: http://speck.ly/?s=http%3A%2F%2Fc%5B0-7%5D.greyhound.io&r=ept%3A%2F%2Fna.entwine.io%2Fnyc&ca=-0&ce=49.06&ct=-8239196%2C4958509.308%2C337&cd=42640.943&cmd=125978.13&ps=2&pa=0.1&ze=1&c0s=remote%3A%2F%2Fimagery%3Furl%3Dhttp%3A%2F%2Fserver.arcgisonline.com%2FArcGIS%2Frest%2Fservices%2FWorld_Imagery%2FMapServer%2Ftile%2F%7B%7Bz%7D%7D%2F%7B%7By%7D%7D%2F%7B%7Bx%7D%7D.jpg

