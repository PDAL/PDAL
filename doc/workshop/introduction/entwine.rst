.. _workshop-entwine:

Entwine
================================================================================

.. include:: ../includes/substitutions.rst

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

The |JSON| file defines the pipeline which you were previously creating in |jq|. This simplifies the task and allows for easy repetition
of tasks. This pipeline will collect the sample data set and convert it to a :ref:`COPC<writers.copc>` file.

.. index:: Potree


1. View the ``./exercises/translation/entwine.json`` file in your editor.

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

3. Verify that the COPC data look ok:

    .. code-block:: console

        $ pdal info dublin.copc.laz | jq .stats.bbox.native.bbox
        {
            "maxx": -694128.96,
            "maxy": 7049938.84,
            "maxz": 385.37,
            "minx": -699477.88,
            "miny": 7044490.98,
            "minz": -144.24
        }
        $ pdal info dublin.copc.laz -p 0
        {
            "file_size": 90310030,
            "filename": "dublin.copc.laz",
            "now": "2023-06-02T13:40:36-0500",
            "pdal_version": "2.5.3 (git-version: Release)",
            "points":
            {
              "point":
              {
                "ClassFlags": 0,
                "Classification": 2,
                "EdgeOfFlightLine": 0,
                "GpsTime": 402930.3873,
                "Intensity": 220,
                "NumberOfReturns": 1,
                "OriginId": 0,
                "PointId": 0,
                "PointSourceId": 34,
                "ReturnNumber": 1,
                "ScanAngleRank": 22.99799919,
                "ScanChannel": 0,
                "ScanDirectionFlag": 1,
                "UserData": 0,
                "X": -695085.89,
                "Y": 7048577.02,
                "Z": 7.8
                }
            },
            "reader": "readers.las"
        }

4. Visualize the data in QGIS

    .. image:: ../images/entwine-view.png
        :target: ../../_images/entwine-view.png


Notes
--------------------------------------------------------------------------------

1. :ref:`readers.ept` contains more detailed documentation about how to
   use PDAL's EPT reader.
