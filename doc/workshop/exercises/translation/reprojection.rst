.. _reprojection:

Reprojection
================================================================================

.. index:: Reprojection, WGS84, UTM

.. include:: ../../includes/substitutions.rst

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to reproject |ASPRSLAS| data

.. _`LASzip`: http://laszip.org
.. _`ASPRS LAS`: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html

Issue the following command in your |Terminal|:

.. code-block:: console

    $ pdal translate ./exercises/analysis/ground/CSite1_orig-utm.laz \
    ./exercises/translation/csite-dd.laz reprojection \
    --filters.reprojection.out_srs="EPSG:4326"

.. code-block:: doscon

    > pdal translate ./exercises/analysis/ground/CSite1_orig-utm.laz ^
    ./exercises/translation/csite-dd.laz reprojection ^
    --filters.reprojection.out_srs="EPSG:4326"

Unfortunately this doesn't produce the intended results for us. Issue the
following ``pdal info`` command to see why:

.. code-block:: console

    $ pdal info ./exercises/translation/csite-dd.laz --all \
    | jq .stats.bbox.native.bbox
    {
        "maxx": 9.18,
        "maxy": 48.79,
        "maxz": 426.91,
        "minx": 9.16,
        "miny": 48.78,
        "minz": 99.43
    }

.. code-block:: doscon

    > pdal info ./exercises/translation/csite-dd.laz --all ^
    | jq .stats.bbox.native.bbox
    {
        "maxx": 9.18,
        "maxy": 48.79,
        "maxz": 426.91,
        "minx": 9.16,
        "miny": 48.78,
        "minz": 99.43
    }

``--all`` dumps all :ref:`info_command` information about the file, and we can
then use the |jq| command to extract out the "native" (same coordinate system
as the file itself) bounding box.

We can see, the problem is we only have two decimal places of precision on the
bounding box. For geographic coordinate systems, this isn't enough precision.

Printing the first point confirms this problem:

.. code-block:: console

    $ pdal info ./exercises/translation/csite-dd.laz -p 0
    {
    "file_size": 4609784,
    "filename": "./exercises/translation/csite-dd.laz",
    "now": "2022-05-13T13:34:23-0700",
    "pdal_version": "2.4.0 (git-version: Release)",
    "points":
    {
        "point":
        {
        "Blue": 0,
        "Classification": 0,
        "EdgeOfFlightLine": 0,
        "GpsTime": 0,
        "Green": 0,
        "Intensity": 100,
        "NumberOfReturns": 2,
        "PointId": 0,
        "PointSourceId": 0,
        "Red": 0,
        "ReturnNumber": 1,
        "ScanAngleRank": 0,
        "ScanDirectionFlag": 0,
        "UserData": 0,
        "X": 9.167893439,
        "Y": 48.78347733,
        "Z": 316.88
        }
    },
    "reader": "readers.las"
    }


Some formats, like :ref:`writers.las` do not automatically set scaling
information. PDAL cannot really do this for you because there are a number
of ways to trip up. For latitude/longitude data, you will need to set the scale
to smaller values like ``0.0000001``. Additionally, LAS uses an offset value to
move the origin of the value. Use PDAL to set that to ``auto`` so you don't
have to compute it.

.. code-block:: console

    $ pdal translate \
    ./exercises/analysis/ground/CSite1_orig-utm.laz \
    ./exercises/translation/csite-dd.laz reprojection \
    --filters.reprojection.out_srs="EPSG:4326" \
    --writers.las.scale_x=0.0000001 \
    --writers.las.scale_y=0.0000001 \
    --writers.las.offset_x="auto" \
    --writers.las.offset_y="auto"
    (pdal translate writers.las Warning) Auto offset for 'X' requested in stream mode.  Using value of 9.16789.
    (pdal translate writers.las Warning) Auto offset for 'Y' requested in stream mode.  Using value of 48.7835.

.. code-block:: doscon

    > pdal translate ^
    ./exercises/analysis/ground/CSite1_orig-utm.laz ^
    ./exercises/translation/csite-dd.laz reprojection ^
    --filters.reprojection.out_srs="EPSG:4326" ^
    --writers.las.scale_x=0.0000001 ^
    --writers.las.scale_y=0.0000001 ^
    --writers.las.offset_x="auto" ^
    --writers.las.offset_y="auto"
    (pdal translate writers.las Warning) Auto offset for 'X' requested in stream mode.  Using value of 9.16789.
    (pdal translate writers.las Warning) Auto offset for 'Y' requested in stream mode.  Using value of 48.7835.

Run the `pdal info` command again to verify the ``X``, ``Y``, and ``Z``
dimensions:

.. code-block:: console

    $ pdal info ./exercises/translation/csite-dd.laz --all \
    | jq .stats.bbox.native.bbox
    {
        "maxx": 9.179032939,
        "maxy": 48.78976523,
        "maxz": 426.91,
        "minx": 9.164037839,
        "miny": 48.78345443,
        "minz": 99.43
    }

.. code-block:: doscon

    > pdal info ./exercises/translation/csite-dd.laz --all ^
    | jq .stats.bbox.native.bbox
    {
        "maxx": 9.179032939,
        "maxy": 48.78976523,
        "maxz": 426.91,
        "minx": 9.164037839,
        "miny": 48.78345443,
        "minz": 99.43
    }

Notes
--------------------------------------------------------------------------------

1. :ref:`filters.reprojection` will use whatever coordinate system is defined
   by the point cloud file, but you can override it using the ``in_srs``
   option. This is useful in situations where the coordinate system is not
   correct, not completely specified, or your system doesn't have all of the
   required supporting coordinate system dictionaries.

2. PDAL uses |Proj.4| library for reprojection. This library includes the
   capability to do both vertical and horizontal datum transformations.
