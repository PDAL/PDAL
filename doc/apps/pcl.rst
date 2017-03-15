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

    --input, -i        Input filename
    --output, -o       Output filename
    --pcl, -p          PCL filename
    --compress, -z     Compress output data (if supported by output format)
    --metadata, -m     Forward metadata (VLRs, header entries, etc) from previous


