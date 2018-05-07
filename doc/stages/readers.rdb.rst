.. _readers.rdb:

readers.rdb
===========

The **RDB reader** reads from files in the RDB format, the in-house format
used by `RIEGL Laser Measurement Systems GmbH`_.

.. plugin::


Installation
------------

To build PDAL with rdb support, ``set rdb_DIR`` to the path of your local
rdblib installation. rdblib can be obtained from the `RIEGL download pages`_
with a properly enabled user account. The rdblib files do not need to be
in a system-level directory, though they could be (e.g. they could be in
``/usr/local``, or just in your home directory somewhere). For help building
PDAL with optional libraries, see `the optional library documentation`_.

.. note::
   - Minimum rdblib version required to build the driver and run the tests: 2.1.6
   - This driver was developed and tested on Ubuntu 17.10 using GCC 7.2.0.


Example
-------

This example pipeline reads points from a RDB file and stores them in LAS
format. Only points classified as "ground points" are read since option
``filter`` is set to "riegl.class == 2" (see line 6).

.. code-block:: json
   :emphasize-lines: 6
   :linenos:

    {
      "pipeline":[
        {
          "type": "readers.rdb",
          "filename": "autzen-thin-srs.rdbx",
          "filter": "riegl.class == 2"
        },
        {
          "type": "writers.las",
          "filename": "autzen-thin-srs.rdbx"
        }
      ]
    }


Options
-------

filename
  Name of file to read
  [Required]

count
  Maximum number of points to read
  [Optional]
  [Default: read all points]

filter
  Point filter expression string (see RDB SDK documentation for details)
  [Optional]
  [Default: empty string (= no filter)]

extras
  Read all available dimensions (`true`) or known PDAL dimensions only (`false`)
  [Optional]
  [Default: false]


Dimensions
----------

The reader maps following default RDB point attributes to PDAL dimensions
(if they exist in the RDB file):

+----------------------------+-------------------------+
| RDB attribute              | PDAL dimension(s)       |
+============================+=========================+
| riegl.id                   | Id::PointId             |
+----------------------------+-------------------------+
| riegl.source_cloud_id      | Id::OriginId            |
+----------------------------+-------------------------+
| riegl.timestamp            | Id::InternalTime        |
+----------------------------+-------------------------+
|                            | Id::X,                  |
| riegl.xyz                  | Id::Y,                  |
|                            | Id::Z                   |
+----------------------------+-------------------------+
| riegl.intensity            | Id::Intensity           |
+----------------------------+-------------------------+
| riegl.amplitude            | Id::Amplitude           |
+----------------------------+-------------------------+
| riegl.reflectance          | Id::Reflectance         |
+----------------------------+-------------------------+
| riegl.deviation            | Id::Deviation           |
+----------------------------+-------------------------+
| riegl.pulse_width          | Id::PulseWidth          |
+----------------------------+-------------------------+
| riegl.background_radiation | Id::BackgroundRadiation |
+----------------------------+-------------------------+
| riegl.target_index         | Id::ReturnNumber        |
+----------------------------+-------------------------+
| riegl.target_count         | Id::NumberOfReturns     |
+----------------------------+-------------------------+
| riegl.scan_direction       | Id::ScanDirectionFlag   |
+----------------------------+-------------------------+
| riegl.scan_angle           | Id::ScanAngleRank       |
+----------------------------+-------------------------+
| riegl.class                | Id::Classification      |
+----------------------------+-------------------------+
|                            | Id::Red,                |
| riegl.rgba                 | Id::Green,              |
|                            | Id::Blue                |
+----------------------------+-------------------------+
|                            | Id::NormalX,            |
| riegl.surface_normal       | Id::NormalY,            |
|                            | Id::NormalZ             |
+----------------------------+-------------------------+

All other point attributes that may exist in the RDB file are ignored unless
the option ``extras`` is set to `true`. If so, a custom dimension is defined
for each additional point attribute, whereas the dimension name is equal to
the point attribute name.

.. note::

   Point attributes are read "as-is", no scaling or unit conversion is done
   by the reader. The only exceptions are point coordinates (``riegl.xyz``)
   and surface normals (``riegl.surface_normal``) which are transformed to
   the RDB file's SRS by applying the matrix defined in the (optional) RDB
   file metadata object ``riegl.geo_tag``.


Metadata
--------

The reader adds following objects to the stage's metadata node:


Object "database"
~~~~~~~~~~~~~~~~~

Contains basic information about the RDB file such as the bounding box,
number of points and the file ID.

.. code-block:: json
   :caption: Example:
   :linenos:

    {
      "bounds": {
        "maximum": {
          "X": -2504493.762,
          "Y": -3846841.252,
          "Z":  4413210.394
        },
        "minimum": {
          "X": -2505882.459,
          "Y": -3848231.393,
          "Z":  4412172.548
        }
      },
      "points": 10653,
      "uuid": "637de54d-7e6b-4004-b6ab-b6bc588ec9ea"
    }


List "dimensions"
~~~~~~~~~~~~~~~~~

List of point attribute description objects.

.. code-block:: json
   :caption: Example:
   :linenos:

    [{
      "compression_options": "shuffle",
      "default_value": 0,
      "description": "Cartesian point coordinates wrt. application coordinate system (0: X, 1: Y, 2: Z)",
      "invalid_value": "",
      "length": 3,
      "maximum_value": 535000,
      "minimum_value": -535000,
      "name": "riegl.xyz",
      "resolution": 0.00025,
      "scale_factor": 1,
      "storage_class": "variable",
      "title": "XYZ",
      "unit_symbol": "m"
    },
    {
      "compression_options": "shuffle",
      "default_value": 0,
      "description": "Target surface reflectance",
      "invalid_value": "",
      "length": 1,
      "maximum_value": 327.67,
      "minimum_value": -327.68,
      "name": "riegl.reflectance",
      "resolution": 0.01,
      "scale_factor": 1,
      "storage_class": "variable",
      "title": "Reflectance",
      "unit_symbol": "dB"
    }]

Details about the point attribute properties see RDB SDK documentation.


Object "metadata"
~~~~~~~~~~~~~~~~~

Contains one sub-object for each metadata object stored in the RDB file.

.. code-block:: json
   :caption: Example:
   :linenos:

    {
      "riegl.scan_pattern": {
        "rectangular": {
          "phi_start": 45.0,
          "phi_stop": 270.0,
          "phi_increment": 0.040,
          "theta_start": 30.0,
          "theta_stop": 130.0,
          "theta_increment": 0.040,
          "program": {
            "name": "High Speed"
          }
        }
      },
      "riegl.geo_tag": {
        "crs": {
          "epsg": 4956,
          "wkt": "GEOCCS[\"NAD83(HARN) \/ Geocentric\",DATUM[\"NAD83(HARN)\",SPHEROID[\"GRS 1980\",6378137.000,298.257222101,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"6152\"]],PRIMEM[\"Greenwich\",0.0000000000000000,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"Meter\",1.00000000000000000000,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"X\",OTHER],AXIS[\"Y\",EAST],AXIS[\"Z\",NORTH],AUTHORITY[\"EPSG\",\"4956\"]]"
        },
        "pose": [
           0.837957447, 0.379440385, -0.392240121, -2505819.156,
          -0.545735575, 0.582617132, -0.602270669, -3847595.645,
           0.000000000, 0.718736580,  0.695282481,  4412064.882,
           0.000000000, 0.000000000,  0.000000000,        1.000
        ]
      }
    }

The ``riegl.geo_tag`` object defines the Spatial Reference System (SRS) of the
file. The point coordinates are actually stored in a local coordinate system
(usually horizontally leveled) which is based on the SRS. The transformation
from the local system to the SRS is defined by the 4x4 matrix ``pose`` which
is stored in row-wise order. Point coordinates (``riegl.xyz``) and surface
normals (``riegl.surface_normal``) are automatically transformed to the SRS
by the reader.

Details about the metadata objects see RDB SDK documentation.


List "transactions"
~~~~~~~~~~~~~~~~~~~

List of transaction objects describing the history of the file.

.. code-block:: json
   :caption: Example:
   :linenos:

    [{
      "agent": "RDB Library 2.1.6-1677 (x86_64-windows, Apr  5 2018, 10:58:39)",
      "comments": "",
      "id": 1,
      "rdb": "RDB Library 2.1.6-1677 (x86_64-windows, Apr  5 2018, 10:58:39)",
      "settings": {
        "cache_size": 524288000,
        "chunk_size": 65536,
        "chunk_size_lod": 20,
        "compression_level": 10,
        "primary_attribute": {
          "compression_options": "shuffle",
          "default_value": 0,
          "description": "Cartesian point coordinates wrt. application coordinate system (0: X, 1: Y, 2: Z)",
          "invalid_value": "",
          "length": 3,
          "maximum_value": 535000,
          "minimum_value": -535000,
          "name": "riegl.xyz",
          "resolution": 0.00025,
          "scale_factor": 1,
          "storage_class": "variable",
          "title": "XYZ",
          "unit_symbol": "m"
        }
      },
      "start": "2018-04-06 10:10:39.336",
      "stop": "2018-04-06 10:10:39.336",
      "title": "Database creation"
    },
    {
      "agent": "rdbconvert",
      "comments": "",
      "id": 2,
      "rdb": "RDB Library 2.1.6-1677 (x86_64-windows, Apr  5 2018, 10:58:39)",
      "settings": "",
      "start": "2018-04-06 10:10:39.339",
      "stop": "2018-04-06 10:10:39.380",
      "title": "Import"
    },
    {
      "agent": "RiSCAN PRO 64 bit v2.6.3",
      "comments": "",
      "id": 3,
      "rdb": "RDB Library 2.1.6-1677 (x86_64-windows, Apr  5 2018, 10:58:39)",
      "settings": "",
      "start": "2018-04-06 10:10:41.666",
      "stop": "2018-04-06 10:10:41.666",
      "title": "Meta data saved"
    }]

Details about the transaction objects see RDB SDK documentation.


.. _RIEGL Laser Measurement Systems GmbH: http://www.riegl.com
.. _RIEGL download pages: http://www.riegl.com/members-area/software-downloads/libraries/
.. _the optional library documentation: http://pdal.io/compilation/unix.html#configure-your-optional-libraries
