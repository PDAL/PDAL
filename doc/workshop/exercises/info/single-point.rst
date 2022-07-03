.. _workshop-single-point:

Printing the first point
================================================================================

.. include:: ../../includes/substitutions.rst

.. index:: info command, Start Here, installation

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to print information from the first point. Issue the
following command in your `Conda Shell`.

.. code-block:: console

   $ pdal info ./exercises/info/interesting.las -p 0


Here's a summary of what's going on with that command invocation

1. ``pdal``: The ``pdal`` application :)

2. ``info``: We want to run :ref:`info_command` on the data. All commands
   are run by the ``pdal`` application.

3. ``./exercises/info/interesting.las``: The file we are running the command
   on. PDAL will be able to identify this file is an |ASPRSLAS| file from the
   extension, ``.las``, but not every file type is easily identified. You can
   use a :ref:`pipeline <pipeline_command>` to override which
   :ref:`reader <readers>` PDAL will use to open the file.

4. ``-p 0``: ``-p`` corresponds to "print a point", and ``0`` means to print
   the first one (computer people count from 0).

.. code-block:: console

   $ pdal info ./exercises/info/interesting.las -p 0
   {
      "file_size": 37698,
      "filename": "./exercises/info/interesting.las",
      "now": "2022-05-12T12:20:00-0700",
      "pdal_version": "2.4.0 (git-version: Release)",
      "points":
      {
         "point":
         {
            "Blue": 88,
            "Classification": 1,
            "EdgeOfFlightLine": 0,
            "GpsTime": 245380.7825,
            "Green": 77,
            "Intensity": 143,
            "NumberOfReturns": 1,
            "PointId": 0,
            "PointSourceId": 7326,
            "Red": 68,
            "ReturnNumber": 1,
            "ScanAngleRank": -9,
            "ScanDirectionFlag": 1,
            "UserData": 132,
            "X": 637012.24,
            "Y": 849028.31,
            "Z": 431.66
         }
      },
      "reader": "readers.las"
   }


Notes
--------------------------------------------------------------------------------

.. index:: JSON, CSV

1. PDAL uses |JSON| as the exchange format when printing information from
   :ref:`info_command`. JSON is a structured, human-readable format that is
   much simpler than its |XML| cousin.

2. You can use the :ref:`writers.text` writer to output point attributes to
   |CSV| format for other processing.

3. Output help information on the command line by issuing the ``--help`` option

4. A common query with ``pdal info`` is ``--all``, which will print all header,
   metadata, and statistics about a file.
