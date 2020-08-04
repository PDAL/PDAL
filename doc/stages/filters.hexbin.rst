.. _filters.hexbin:

filters.hexbin
==============

A common questions for users of point clouds is what the spatial extent of a
point cloud collection is. Files generally provide only rectangular bounds, but
often the points inside the files only fill up a small percentage of the area
within the bounds.

.. figure:: filters.hexbin.img1.jpg
    :scale: 50 %
    :alt: Hexbin derived from input point buffer

    Hexbin output shows boundary of actual points in point buffer, not
    just rectangular extents.

The hexbin filter reads a point stream and writes out a metadata record that
contains a boundary, expressed as a well-known text polygon. The filter counts
the points in each hexagonal area to determine if that area should be included
as part of the boundary.  In
order to write out the metadata record, the *pdal* pipeline command must be
invoked using the "--pipeline-serialization" option:

.. streamable::

Example 1
---------

The following pipeline file and command produces an JSON output file
containing the pipeline's metadata, which includes the result of running
the hexbin filter:

::

  [
      "/Users/me/pdal/test/data/las/autzen_trim.las",
      {
          "type" : "filters.hexbin"
      }
  ]

::

  $ pdal pipeline hexbin-pipeline.json --metadata hexbin-out.json


.. code-block:: none

    {
      "stages":
      {
        "filters.hexbin":
        {
          "area": 746772.7543,
          "avg_pt_per_sq_unit": 22.43269935,
          "avg_pt_spacing": 2.605540869,
          "boundary": "MULTIPOLYGON (((636274.38924399 848834.99817891, 637242.52219686 848834.99817891, 637274.79329529 849226.26445367, 637145.70890157 849338.05481789, 637242.52219686 849505.74036422, 636016.22045656 849505.74036422, 635983.94935813 849114.47408945, 636113.03375184 848890.89336102, 636274.38924399 848834.99817891)))",
          "boundary_json": { "type": "MultiPolygon", "coordinates": [ [ [ [ 636274.38924399, 848834.99817891 ], [ 637242.52219686, 848834.99817891 ], [ 637274.79329529, 849226.26445367 ], [ 637145.70890157, 849338.05481789 ], [ 637242.52219686, 849505.74036422 ], [ 636016.22045656, 849505.74036422 ], [ 635983.94935813, 849114.47408945 ], [ 636113.03375184, 848890.89336102 ], [ 636274.38924399, 848834.99817891 ] ] ] ] },
          "density": 0.1473004999,
          "edge_length": 0,
          "estimated_edge": 111.7903642,
          "hex_offsets": "MULTIPOINT (0 0, -32.2711 55.8952, 0 111.79, 64.5422 111.79, 96.8133 55.8952, 64.5422 0)",
          "sample_size": 5000,
          "threshold": 15
        }
    },
    ...


Example 2
---------

As a convenience, the ``pdal info`` command will produce similar output:

::

    $ pdal info --boundary /Users/me/test/data/las/autzen_trim.las

.. code-block:: json

    {
      "boundary":
      {
        "area": 746772.7543,
        "avg_pt_per_sq_unit": 22.43269935,
        "avg_pt_spacing": 2.605540869,
        "boundary": "MULTIPOLYGON (((636274.38924399 848834.99817891, 637242.52219686 848834.99817891, 637274.79329529 849226.26445367, 637145.70890157 849338.05481789, 637242.52219686 849505.74036422, 636016.22045656 849505.74036422, 635983.94935813 849114.47408945, 636113.03375184 848890.89336102, 636274.38924399 848834.99817891)))",
        "boundary_json": { "type": "MultiPolygon", "coordinates": [ [ [ [ 636274.38924399, 848834.99817891 ], [ 637242.52219686, 848834.99817891 ], [ 637274.79329529, 849226.26445367 ], [ 637145.70890157, 849338.05481789 ], [ 637242.52219686, 849505.74036422 ], [ 636016.22045656, 849505.74036422 ], [ 635983.94935813, 849114.47408945 ], [ 636113.03375184, 848890.89336102 ], [ 636274.38924399, 848834.99817891 ] ] ] ] },
        "density": 0.1473004999,
        "edge_length": 0,
        "estimated_edge": 111.7903642,
        "hex_offsets": "MULTIPOINT (0 0, -32.2711 55.8952, 0 111.79, 64.5422 111.79, 96.8133 55.8952, 64.5422 0)",
        "sample_size": 5000,
        "threshold": 15
      },
      "filename": "\/Users\/acbell\/pdal\/test\/data\/las\/autzen_trim.las",
      "pdal_version": "1.6.0 (git-version: 675afe)"
    }

Options
-------

_`edge_size`
  If not set, the hexbin filter will estimate a hex size based on a sample of
  the data. If set, hexbin will use the provided size in constructing the
  hexbins to test.

sample_size
  How many points to sample when automatically calculating the edge
  size? Only applies if edge_size_ is not explicitly set. [Default: 5000]

threshold
  Number of points that have to fall within a hexagon boundary before it
  is considered "in" the data set. [Default: 15]

precision
  Minimum number of significant digits to use in writing out the
  well-known text of the boundary polygon. [Default: 8]

preserve_topology
  Use GEOS SimplifyPreserveTopology instead of Simplify for polygon simplification with  `smooth` option. [Default: true]

smooth
  Use GEOS simplify operations to smooth boundary to a tolerance [Default: true]

.. include:: filter_opts.rst

