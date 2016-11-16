.. _readers.greyhound:

readers.greyhound
=================

The **Greyhound Reader** allows you to query point data from a `Greyhound`_
server.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.greyhound",
          "url":"data.greyhound.io",
          "resource":"autzen"
        },
        {
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }


Options
-------

_`url`
  Greyhound server URL string. [Required]

_`resource`
  Name of the Greyhound resource to access. [Required]

_`bounds`
  Spatial bounds to query, expressed as a string, e.g.
  *([xmin, xmax], [ymin, ymax])* or
  *([xmin, xmax], [ymin, ymax], [zmin, zmax])*.  By default, the entire resource
  is queried.

_`depth_begin`
  Beginning octree depth to query, inclusive.  Lower depth values have coarser
  resolution, so a depth range of *[0, 8)* could provide a low-resolution
  overview of the entire resource, for example.  [Default: **0**]

_`depth_end`
  Ending octree depth to query, non-inclusive.  A value of **0** will search all
  depths greater-than or equal-to *depth_begin*.  If non-zero, this value should
  be greater than *depth_begin* or the result will always be empty.
  [Default: **0**]

_`tile_path`
  A Greyhound resource may be an aggregation of multiple input files.  If a
  *tile_path* option is present, then only points belonging to that file will
  be queried.  This search is spatially optimized, so no `bounds`_ option needs
  to be present to limit the query bounds.

_`filter`
  Server-side filtering may be requested which may further limit the data
  selected by the query.  The filter is represented as JSON, and performs
  filtering on dimensions present in the resource, or the pseudo-dimension
  *Path*, corresponding to `tile_path`_ values.

  Arbitrary logic combinations may be created using `comparison`_ and
  `logical`_ query operators with syntax matching that of MongoDB.  Some sample
  filters follow.

.. code-block:: json

    {
        "Path":{"$in":["tile-845.laz", "tile-846.laz"]},
        "Classification":{"$ne":18}
    }

.. code-block:: json

    {"$or":[
        {"Red":{"$gt":200}},
        {"Blue":{"$gt":120,"$lt":130}},
        {"Classification":{"$nin":[2,3]}}
    ]}

_`threads`
  Because Greyhound resources are accessed by combination of multiple HTTP
  requests, threaded operation can greatly increase throughput.  This option
  sets the number of concurrent requests allowed.  [Default: **4**]

.. _Greyhound: https://github.com/hobu/greyhound
.. _comparison: https://docs.mongodb.com/manual/reference/operator/query-comparison/
.. _logical: https://docs.mongodb.com/manual/reference/operator/query-logical/

