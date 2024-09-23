.. _sort_command:

********************************************************************************
sort
********************************************************************************

The ``sort`` command uses :ref:`filters.mortonorder` to sort data by XY values.

::

    $ pdal sort <input> <output>

::

    --input, -i        Input filename
    --output, -o       Output filename
    --compress, -z     Compress output data (if supported by output format)
    --metadata, -m     Forward metadata (VLRs, header entries, etc) from previous stages


