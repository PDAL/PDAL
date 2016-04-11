.. _georeferencing:

Georeferencing
================================================================================

Purpose:

- Transform airborne or mobile LiDAR points into usable point clouds.


Components
===============

- LiDAR point stream
    - X, Y, Z
    - Timestamp
- GNSS/IMU point stream
    - GPS X, Y, Z
    - IMU roll, pitch, yaw
- Lever arm and boresight matrix
- Combine with the georeferencing equation:

.. figure:: img/georeferencing.png

Caveats
============

- Generic georeferencing is **not yet** implemented in PDAL.
- The :ref:`readers.optech` includes built-in georeferencing, which automatically projects data to WGS84.

Command
============

.. literalinclude:: ../../exercises/georeferencing/georeferencing-command.txt
    :linenos:

- In order to display the data easily in plas.io, we must reproject to UTM.
- There are some spurious points close to the sensor that skew the intensity distribution.
  By filtering these points, we will improve the default display in plas.io.

Verify
===========

.. figure:: ../../images/georeferencing-run-command.png

Visualize
===============

.. figure:: ../../images/georeference-plasio.png
