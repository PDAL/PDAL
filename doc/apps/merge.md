.. _merge_command:

********************************************************************************
merge
********************************************************************************

The ``merge`` command will combine input files into a single output file.

::

    $ pdal merge <input> ... <output>

::

    --files, -f    List of filenames.  The last file listed is taken to be
        the output file.

This command provides simple merging of files.  It provides no facility for
filtering, reprojection, etc.  The file type of the input files may be
different from one another and different from that of the output file.


