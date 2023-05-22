.. _filters.trajectory:

filters.trajectory
==================

The **trajectory filter** computes an estimate the the sensor location based
on the position of multiple returns and the sensor scan angle. It is primarily
useful for LAS input as it requires scan angle and return counts in order to
work.

The method is described in detail `here`_. It extends the method of :cite:`Gatziolis2019`.

.. note::

  This filter creates a new dataset describing the trajectory of the sensor,
  replacing the input dataset.

Examples
--------

.. code-block:: json

  [
      "input.las",
      {
          "type": "filters.trajectory"
      },
      "trajectory.las"
  ]


Options
--------

dtr
  Multi-return sampling interval in seconds. [Default: .001]

dst
  Single-return sampling interval in seconds. [Default: .001]

minsep
   Minimum separation of returns considered in meters. [Default: .01]

tblock
  Block size for cublic spline in seconds. [Default: 1.0]

tout
  Output data interval in seconds. [Default: .01]

.. include:: filter_opts.rst

.. _`here`: ../papers/lidar-traj.pdf
