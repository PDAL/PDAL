.. _delta_command:

******************************************************************************
delta
******************************************************************************

The ``delta`` command is used to select a nearest point from a candidate file
for each point in the source file. If the ``--2d`` option is used, the
query only happens in XY coordinate space.

::

    $ pdal delta <source> <candidate> [output]

Standard out is used if no output file is specified.

::

    --source arg     Non-positional option for specifying source filename
    --candidate arg  Non-positional option for specifying candidate filename
    --output arg     Non-positional option for specifying output filename [/dev/stdout]
    --2d             only 2D comparisons/indexing

Example 1:
--------------------------------------------------------------------------------

::

    $ pdal delta ../../test/data/las/1.2-with-color.las \
        ../../test/data/las/1.2-with-color.las
    --------------------------------------------------------------------------------
    Delta summary for
         source: '../../test/data/las/1.2-with-color.las'
         candidate: '../../test/data/las/1.2-with-color.las'
    --------------------------------------------------------------------------------

    ----------- --------------- --------------- --------------
     Dimension       X             Y                  Z
    ----------- --------------- --------------- --------------
     Min        0.0000            0.0000            0.0000
     Max        0.0000            0.0000            0.0000
     Mean       0.0000            0.0000            0.0000
    ----------- --------------- --------------- --------------

Example 2:
--------------------------------------------------------------------------------

::

    $ pdal delta test/data/1.2-with-color.las \
        test/data/1.2-with-color.las --detail
    "ID","DeltaX","DeltaY","DeltaZ"
    0,0.00,0.00,0.00
    1,0.00,0.00,0.00
    2,0.00,0.00,0.00
    3,0.00,0.00,0.00
    4,0.00,0.00,0.00
    5,0.00,0.00,0.00
