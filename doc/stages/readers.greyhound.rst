.. _readers.greyhound:

readers.greyhound
=================

The **Greyhound Reader** allows you to query point data from a `Greyhound`_
server.

.. plugin::

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type": "readers.greyhound",
          "url": "data.greyhound.io",
          "resource": "iowa-z",
          "filter": {"$and": [
            {"Path": "02004736.laz"},
            {"Classification": {"$ne": 7}}
          ]}
        },
        "output.laz"
      ]
    }

.. code-block:: json

    {
      "pipeline":[
        "greyhound://data.greyhound.io/resource/iowa-z/read?filter={\"Path\":\"02004736.laz\"}",
        "02004736.laz"
      ]
    }

Options
-------

_`url`
  Greyhound server URL, or a full Greyhound `read`_ query URL.  If specified as a full Greyhound query URL, no other options need to be present.

_`resource`
  Name of the Greyhound resource to access.

_`bounds`
  Spatial bounds to query, expressed as a string, e.g.
  *([xmin, xmax], [ymin, ymax])* or
  *([xmin, xmax], [ymin, ymax], [zmin, zmax])* or as a Greyhound `bounds array`_.  By default, the entire resource is queried.

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

.. _Greyhound: https://github.com/hobu/greyhound
.. _bounds array: https://github.com/hobu/greyhound/blob/master/doc/clientDevelopment.rst#bounds-option
.. _read: https://github.com/hobu/greyhound/blob/master/doc/clientDevelopment.rst#the-read-query
.. _comparison: https://docs.mongodb.com/manual/reference/operator/query-comparison/
.. _logical: https://docs.mongodb.com/manual/reference/operator/query-logical/

