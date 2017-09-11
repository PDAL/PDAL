.. _workshop-greyhound:

Greyhound
================================================================================

.. include:: ../../includes/substitutions.rst

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to fetch data from a Greyhound server. Greyhound is a
web server for point cloud data. You can learn more about what it is by
visiting http://lidarnews.com/articles/open-source-point-cloud-web-services-with-greyhound/

.. index:: greyhound, web services


See the Dublin data used in this example in your browser at

http://potree.entwine.io/data/dublin.html

.. index:: Potree


1. View the ``greyhound.json`` file in your editor


    .. literalinclude:: ./greyhound.json

    .. note::

        If you use the `Developer Console`_ when visiting
        http://speck.ly or http://potree.entwine.io, you can see the
        browser making requests against the Greyhound server at
        http://data.greyhound.io

2. Issue the following command in your |Terminal|.


    .. literalinclude:: ./greyhound-command.txt

    .. image:: ../../images/greyhound-command.png
        :target: ../../../_images/greyhound-command.png

.. _`Developer Console`: https://developers.google.com/web/tools/chrome-devtools/console/

3. Verify that the data look ok:

    .. literalinclude:: ./greyhound-info-command.txt

    .. image:: ../../images/greyhound-info-verify.png
        :target: ../../../_images/greyhound-info-verify.png


4. Visualize the data in http://plas.io

    .. image:: ../../images/greyhound-view.png
        :target: ../../../_images/greyhound-view.png


Notes
--------------------------------------------------------------------------------

1. :ref:`readers.greyhound` contains more detailed documentation about how to
   use PDAL's |Greyhound| reader .

2. As ``depth_end`` gets larger, the number of possible points goes up by
   nearly a factor of 4. Use the ``bounds`` option of the reader to split
   up the boxes you are querying to decrease the potential number of points a
   query might return.

