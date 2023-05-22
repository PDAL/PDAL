.. _near:

Searching near a point
================================================================================

.. include:: ../../includes/substitutions.rst

.. index:: nearest, nearby, query, search

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to find points near a given search location. Our
scenario is a simple one -- we want to find the two points nearest the midpoint
of the bounding cube of our ``interesting.las`` data file.

First we need to find the midpoint of the bounding cube. To do that, we need to
print the ``--all`` info for the file and look for the ``bbox`` output:

.. code-block:: console

    $ pdal info ./exercises/info/interesting.las --all | jq .stats.bbox.native.bbox
    {
        "maxx": 638982.55,
        "maxy": 853535.43,
        "maxz": 586.38,
        "minx": 635619.85,
        "miny": 848899.7,
        "minz": 406.59
    }

Find the average the ``X``, ``Y``, and ``Z`` values:

::

    x = 635619.85 + (638982.55 - 635619.85)/2 = 637301.20
    y = 848899.70 + (853535.43 - 848899.70)/2 = 851217.57
    z = 406.59 + (586.38 - 406.59)/2 = 496.49


With our "center point", issue the ``--query`` option to ``pdal info``
and return the three nearest points to it:

.. code-block:: console

    $ pdal info ./exercises/info/interesting.las --query "637301.20, 851217.57, 496.49/3"
    {
    "file_size": 37698,
    "filename": "./exercises/info/interesting.las",
    "now": "2022-05-13T12:37:54-0700",
    "pdal_version": "2.4.0 (git-version: Release)",
    "points":
    {
        "point":
        [
        {
            "Blue": 221,
            "Classification": 1,
            "EdgeOfFlightLine": 0,
            "GpsTime": 247565.2203,
            "Green": 211,
            "Intensity": 169,
            "NumberOfReturns": 1,
            "PointId": 762,
            "PointSourceId": 7330,
            "Red": 228,
            "ReturnNumber": 1,
            "ScanAngleRank": -4,
            "ScanDirectionFlag": 0,
            "UserData": 124,
            "X": 637323.56,
            "Y": 851555.64,
            "Z": 586.38
        },
        {
            "Blue": 243,
        ...

.. note::

    The ``/3`` portion of our query string tells the ``query``
    command to give us the 3 nearest points. Adjust this value to
    return data in closest-distance ordering.

Notes
--------------------------------------------------------------------------------

1. PDAL uses `JSON`_ as the exchange format when printing information from
   :ref:`info_command`.  JSON is a structured, human-readable format that is
   much simpler than its `XML`_ cousin.

2. The ``--query`` option of :ref:`info_command` constructs a `KD-tree`_ of the
   entire set of points in memory. If you have really large data sets, this
   isn't going to work so well, and you will need to come up with a different
   solution.

.. _`KD-tree`: https://en.wikipedia.org/wiki/K-d_tree
.. _`CSV`: https://en.wikipedia.org/wiki/Comma-separated_values
.. _`JSON`: https://en.wikipedia.org/wiki/JSON
.. _`XML`: https://en.wikipedia.org/wiki/XML
