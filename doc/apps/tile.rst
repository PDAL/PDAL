.. _tile_command:

********************************************************************************
tile
********************************************************************************

The ``tile`` command will create multiple output files from input files
by generating square tiles of points.  The command takes an input
file name and an output filename template.

This command is similar to the :ref:`split <split_command>` command, but
differs in several ways.  The ``tile`` command:

- Uses streaming mode to limit the amount of memory consumed by point data.
- Uses a placeholder for filename output.
- Provides for reprojection of data to create consistent output.
- Always creates square tiles that contain all points "covered" by each tile.

::

    $ pdal tile <input> <output>

::

    --input, -i     Input filename
    --output, -o    Output filename
    --length        Edge length for cells [Default: 1000]
    --origin_x      Origin in X axis for cells [Default: None]
    --origin_y      Origin in Y axis for cells [Default: None]
    --buffer        Size of buffer (overlap) to include around each tile.
                    [Default: 0]
    --out_srs       Spatial reference system to which all input points
                    will be reprojected. [Default: None]

The input filename can contain a `glob pattern`_ to allow multiple files
as input.

The output filename must contain a placeholder character ``#``.  The
placeholder character is replaced with an X/Y index of the tile as a part
of a cartesian system.  For example, if the output filename is specified as
``out#.las``, the tile containing the origin will be named ``out0_0.las``.
The tile to its right will be named ``out1_0.las``.  The tile above it
will be named ``out0_1.las``.  The command does not create directories -- 
create any desired directories before running.

If an origin is not supplied with as argument, the first point read is
used as the origin.

Example 1:
--------------------------------------------------------------------------------

::

    $ pdal tile infile.laz "outfile_#.bpf"

This command takes the points from the input file ``infile.laz`` and creates
output files ``outfile_0_0.bpf``, ``outfile_0_1.bpf``, ... where each output
file contains points in the 1000x1000 square units represented by the tile.
The X/Y location of the first point is used as the origin of the tile grid.

Example 2:
--------------------------------------------------------------------------------

::

    $ pdal tile "/home/me/files/*" "out_#.txt" --out_srs="EPSG:4326"

Reads all files in the directory /home/me/files as input and reprojects
points to geographic coordinates if necessary.  The output is written to
a set of text files in the current directory.

.. _glob pattern: https://en.wikipedia.org/wiki/Glob_%28programming%29
