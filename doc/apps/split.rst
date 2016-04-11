.. _split_command:

********************************************************************************
split
********************************************************************************

The ``split`` command will create multiple output files from a single input
file.  The command takes an input file name and an output filename (used as a
template) or output directory specification.

::

    $ pdal split <input> <output>

::

    --input [-i] arg   Non-positional option for specifying input file name
    --output [-o] arg  Non-positional option for specifying output file/directory name
    --length arg       Edge length for splitter cells.  See :ref:`filters.splitter`.
    --capacity arg     Point capacity for chipper cells.  See :ref:`filters.chipper`.

If neither the ``--length`` nor ``--capacity`` arguments are specified, an
implcit argument of capacity with a value of 100000 is added.

The output argument is a template.  If the output argument is, for example,
``file.ext``, the output files created are ``file_#.ext`` where # is a number
starting at one and incrementing for each file created.

If the output argument ends in a path separator, it is assumed to be a
directory and the input argument is appended to create the output template.
The ``split`` command never creates directories.  Directories must pre-exist.

Example 1:
--------------------------------------------------------------------------------

::

    $ pdal split --capacity 100000 infile.laz outfile.bpf

This command takes the points from the input file ``infile.laz`` and creates
output files ``outfile_1.bpf``, ``outfile_2.bpf``, ... where each output file
contains no more than 100000 points.


