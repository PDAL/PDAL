.. _filters.georeference:

filters.georeference
====================

The **georeference filter** georeferences point cloud expressed in scanner coordinates,
using `GpsTime` Dimension as a synchronisation reference.

.. streamable::

.. note::

  This filter works only with `sbet` trajectories.

Examples
--------

.. code-block:: json

  [
      "input.rxp",
      {
          "type": "filters.georeference",
          "trajectory" : "sbet.out",
          "scan2imu" : "0.547418069126518 0.629350569360448 0.055655 0.282019849017109 0.832472349301833 -0.476921998243165 -0.830192 -0.784992666412172 -0.085575961821537 -0.613566026143419 -0.099056 0.000 0.000 0.000 1.000"

      },
      {
        "type" : "filters.reprojection",
        "in_srs" : "EPSG:4979",
        "out_srs" : "EPSG:2154+5720"
      },
      "georeference.las"
  ]


Options
--------

trajectory
  path to a sbet trajectory file. [Mandatory]

scan2imu
  4x4 transformation matrix from scanner frame to body imu. By default expressed in NED coordinates. [Mandatory]

reverse
   revert georeferencing (go back to scanner frame). [Default: false]

time_offset
  timestamp offset between trajectory and scanner GpsTime. [Default: 0]

coordinate_system
  Two right-handed variants exist for Local tangent plane coordinates: east, north, up (ENU) coordinates and north, east, down (NED). [Default : NED]
  

.. include:: filter_opts.rst
