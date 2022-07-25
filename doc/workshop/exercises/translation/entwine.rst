.. _workshop-entwine:

Entwine
================================================================================

.. include:: ../../includes/substitutions.rst

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to fetch data from an Entwine index stored in an Amazon
Web Services object store (bucket). Entwine is a point cloud indexing strategy,
which rearranges points into a lossless octree structure known as EPT, for
Entwine Point Tiles. The structure is described here: https://entwine.io/entwine-point-tile.html.

EPT indexes can be used for visualization as well as analysis and data
manipulation at any scale.

Examples of Entwine usage can be found from very fine photogrammetric surveys
to continental scale lidar management.

.. index:: EPT, web services

US Geological Survey (USGS) example data is here: https://usgs.entwine.io/

We will use a sample data set from Dublin, Ireland https://viewer.copc.io/?r=https://na-c.entwine.io/dublin/ept.json


.. index:: Potree


1. View the ``entwine.json`` file in your editor.

    .. literalinclude:: ./entwine.json

    .. note::

        If you use the `Developer Console`_ when visiting
        http://speck.ly or http://potree.entwine.io, you can see the
        browser making requests against the EPT resource at
        http://na-c.entwine.io/dublin/ept.json

2. Issue the following command in your `Conda Shell`.

    .. code-block:: console

        $ pdal pipeline ./exercises/translation/entwine.json -v 7
        (PDAL Debug) Debugging...
        (pdal pipeline readers.ept Debug) Root resolution: 21.3828
        Query resolution:  5
        Actual resolution: 2.67285
        Depth end: 4
        Query bounds: ()
        Threads: 15
        (pdal pipeline Debug) Executing pipeline in stream mode.
        (pdal pipeline writers.las Debug) Wrote 8034506 points to the LAS file

.. _`Developer Console`: https://developers.google.com/web/tools/chrome-devtools/console/

1. Verify that the data look ok:

    .. code-block:: console

        $ pdal info dublin.laz | jq .stats.bbox.native.bbox
        {
            "maxx": -694128.96,
            "maxy": 7049938.84,
            "maxz": 385.37,
            "minx": -699477.88,
            "miny": 7044490.98,
            "minz": -144.24
        }
        $ pdal info dublin.laz -p 0
        {
            "file_size": 56441298,
            "filename": "dublin.laz",
            "now": "2022-05-16T09:57:45-0700",
            "pdal_version": "2.3.0 (git-version: Release)",
            "points":
            {
                "point":
                {
                    "Classification": 4,
                    "EdgeOfFlightLine": 0,
                    "Intensity": 265,
                    "NumberOfReturns": 2,
                    "PointId": 0,
                    "PointSourceId": 16,
                    "ReturnNumber": 1,
                    "ScanAngleRank": 2,
                    "ScanDirectionFlag": 1,
                    "UserData": 0,
                    "X": -695945.82,
                    "Y": 7046284.13,
                    "Z": 122.01
                }
            },
            "reader": "readers.las"
        }

2. Visualize the data in http://plas.io

    .. image:: ../../images/entwine-view.png
        :target: ../../../_images/entwine-view.png


Notes
--------------------------------------------------------------------------------

1. :ref:`readers.ept` contains more detailed documentation about how to
   use PDAL's EPT reader .
