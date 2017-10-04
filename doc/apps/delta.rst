.. _delta_command:

******************************************************************************
delta
******************************************************************************

The ``delta`` command is used to select a nearest point from a candidate file
for each point in the source file.

::

    $ pdal delta <source> <candidate>

::

    --source           source file name
    --candidate        candidate file name
    --detail           Output deltas per-point
    --alldims          Compute diffs for all dimensions (not just X,Y,Z)

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
