.. _filters.georeference:

filters.georeference
====================

The **georeference filter** georeferences point cloud expressed in scanner coordinates,
using `GpsTime` Dimension as a synchronisation reference with a given trajectory.

.. streamable::

.. note::

  This filter expects trajectory to :

  * contains `X`, `Y`, `Z`, `Roll`, `Pitch`, `Yaw`, `WanderAngle` and `GpsTime` ;
  * have coordinates expressed in `WGS84` system (EPSG:4979) ;
  * have all its angle values expressed in radians.

Examples
--------

.. code-block:: json

  [
      "input.rxp",
      {
          "type": "filters.georeference",
          "trajectory_file" : "sbet.out",
          "trajectory_options": {
            "type": "readers.sbet",
            "angles_as_degrees": false
        },
          "scan2imu" : "-0.555809 0.545880 0.626970 0.053833 
          0.280774 0.833144 -0.476484 -0.830238 
          -0.782459 -0.088797 -0.616338 -0.099672 
          0.000000 0.000000 0.000000 1.000000"
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

trajectory_file
  path to a sbet trajectory file. [Mandatory]

trajectory_options
  JSON object with keys of reader options and the values to pass through. [Default: {}]

scan2imu
  4x4 transformation matrix from scanner frame to body imu. By default expressed in NED coordinates. [Mandatory]

reverse
   revert georeferencing (go back to scanner frame). [Default: false]

time_offset
  timestamp offset between trajectory and scanner GpsTime. [Default: 0]

coordinate_system
  Two right-handed variants exist for Local tangent plane coordinates: east, north, up (ENU) coordinates and north, east, down (NED). [Default : NED]
  

.. include:: filter_opts.rst
