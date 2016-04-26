.. _pcl_command:

********************************************************************************
pcl
********************************************************************************

The ``pcl`` command is used to invoke a PCL JSON pipeline. See
:ref:`pcl_block_tutorial` for more information.

.. note::

    The ``pcl`` command is only available when PDAL is linked with PCL.

::

    $ pdal pcl <input> <output> <pcl>

::

    --input [-i] arg   Non-positional argument to specify input file name.
    --output [-o] arg  Non-positional argument to specify output file name.
    --pcl [-p] arg     Non-positional argument to specify pcl file name.
    --compress [-z]    Compress output data (if supported by output format)
    --metadata [-m]    Forward metadata from previous stages.


