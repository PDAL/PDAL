.. _workshop-entwine:

Entwine
================================================================================

.. include:: ../../includes/substitutions.rst

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to fetch data from an Entwine index stored in an Amazon Web Services object store (bucket). Entwine is a point cloud indexing strategy, which rearranges points into a lossless octree structure known as EPT, for Entwine Point Tiles. The structure is described here: https://entwine.io/entwine-point-tile.html.

EPT indexes can be used for visualization as well as analysis and data manipulation at any scale.

Examples of Entwine usage can be found from very fine photogrammetric surveys to continental scale lidar management.

.. index:: EPT, web services

US Geological Survey (USGS) example data is here: https://usgs.entwine.io/

We will use a sample data set from Dublin, Ireland
  http://potree.entwine.io/data/view.html?r=%22http://na-c.entwine.io/dublin/ept.json%22


.. index:: Potree


1. View the ``entwine.json`` file in your editor. If the file does not exist, create
   it and paste the following JSON into it:

    .. literalinclude:: ./entwine.json

    .. note::

        If you use the `Developer Console`_ when visiting
        http://speck.ly or http://potree.entwine.io, you can see the
        browser making requests against the EPT resource at
        http://na-c.entwine.io/dublin/ept.json

2. Issue the following command in your `Conda Shell`.

    .. literalinclude:: ./entwine-command.txt

    .. image:: ../../images/entwine-command.png
        :target: ../../../_images/entwine-command.png

.. _`Developer Console`: https://developers.google.com/web/tools/chrome-devtools/console/

3. Verify that the data look ok:

    .. literalinclude:: ./entwine-info-command.txt

    .. image:: ../../images/entwine-info-verify.png
        :target: ../../../_images/entwine-info-verify.png


4. Visualize the data in http://plas.io

    .. image:: ../../images/entwine-view.png
        :target: ../../../_images/entwine-view.png


Notes
--------------------------------------------------------------------------------

1. :ref:`readers.ept` contains more detailed documentation about how to
   use PDAL's EPT reader .
