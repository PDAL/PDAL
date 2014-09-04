.. _apps:

******************************************************************************
PDAL Applications
******************************************************************************

:Author: Howard Butler
:Contact: hobu.inc at gmail dot com
:Date: 9/1/2014

PDAL contains a single `git`_-style application, called *pdal*. The `pdal` 
application currently contains six commands:

* :ref:`delta <delta_command>`
* :ref:`diff <diff_command>`
* :ref:`info <info_command>`
* :ref:`pcl <pcl_command>`
* :ref:`pipeline <pipeline_command>`
* :ref:`translate <translate_command>`

Applications are run by invoking the *pdal* application along with the 
command name:

::

    $ pdal info myfile.las
    $ pdal translate input.las output.las
    $ pdal pipeline --stdin < myxml.xml

.. note::
    
    The :ref:`pcl` command is only available if PCL is linked.

Help about each command can be retrieved via the ``--help`` switch. 
The ``--drivers`` and ``--options`` switches can tell you more about 
particular drivers and their options:

::

    $ pdal info --help
    $ pdal translate --drivers
    $ pdal pipeline --options drivers.las.writer

Additional, driver-specific options may be specified by using a 
namespace-prefixed option name. For example, it is possible to 
set the LAS day of year at translation time with the following 
option:

::

    $ pdal translate \
    --drivers.las.writer.creation_doy="42" \
    input.las
    output.las

.. note::

        Driver specific options can be identified using the 
        ``pdal info --options`` invocation.
        
.. _`git`: http://git-scm.com/


.. _delta_command:

``delta`` command
------------------------------------------------------------------------------

The *delta* command is used to select a nearest point from a candidate file
for each point in the source file. If the ``--2d`` option is used, the
query only happens in XY coordinate space.

::

    $ pdal delta test/data/1.2-with-color.las test/data/1.2-with-color.las > deltas

A simple CSV-style text is output with delta information:

::

    [hobu@pyro pdal (master)]$ ./bin/pdal delta test/data/1.2-with-color.las test/data/1.2-with-color.las
    ------------------------------------------------------------------------------------------
     Delta summary for source 'test/data/1.2-with-color.las' and candidate 'test/data/1.2-with-color.las'
    ------------------------------------------------------------------------------------------

    ----------- --------------- --------------- --------------
     Dimension       X             Y                  Z
    ----------- --------------- --------------- --------------
     Min        0.0000            0.0000            0.0000
     Min        0.0000            0.0000            0.0000
     Mean       0.0000            0.0000            0.0000
    ----------- --------------- --------------- --------------

::

        [hobu@pyro pdal (master)]$ ./bin/pdal delta test/data/1.2-with-color.las test/data/1.2-with-color.las --detail

::

    "ID","DeltaX","DeltaY","DeltaZ"
    0,0.00,0.00,0.00
    1,0.00,0.00,0.00
    2,0.00,0.00,0.00
    3,0.00,0.00,0.00
    4,0.00,0.00,0.00
    5,0.00,0.00,0.00

::

      --source arg          source file name
      --candidate arg       candidate file name
      --output arg          output file name
      --2d                  only 2D comparisons/indexing


.. _diff_command:

``diff`` command 
------------------------------------------------------------------------------

The *diff* command is used for executing a simple contextual difference 
between two sources. 

::

    $ pdal diff test/data/1.2-with-color.las test/data/1.2-with-color-clipped.las
    
It will output JSON if there are any differences. It will output nothing 
and return 0 if there are no differences. At this time it supports 
checking the following:

* Different schema
* Expected count
* Metadata
* Actual point count
* Byte-by-byte point data


.. _info_command:

``info`` command
------------------------------------------------------------------------------

Dumps information about a point cloud file, such as:

* basic properties (extents, number of points, point format)

* coordinate reference system

* additional metadata

* summary statistics about the points

* the plain text format should be reStructured text if possible to allow 
  a user to retransform the output into whatever they want with ease

::

    $ pdal info  test/data/1.2-with-color.las --count 3 --query "636601.87, 849018.59, 425.10"

Print the first 10 points of the file as `reStructuredText`_

::
    $ pdal info test/data/1.2-with-color.las -p 0-10

Print three selected points of the file as `reStructuredText`_

::
    $ pdal info test/data/1.2-with-color.las -p 4, 16, 28


.. _`reStructuredText`: http://docutils.sourceforge.net/docs/user/rst/quickref.html

:: 

    -p [ --point ] [=arg(=0)] point to dump
    --query arg               A 2d or 3d point query point
    --distance arg            A query distance
    -a [ --stats ]            dump stats on all points (reads entire dataset)
    --count arg (=0)          How many points should we write?
    --dimensions arg          dump stats on all points (reads entire dataset)
    -s [ --schema ]           dump the schema
    -m [ --metadata ]         dump the metadata
    --sdo_pc                  dump the SDO_PC Oracle Metadata
    -r [ --stage ]            dump the stage info
    --xml                     dump XML instead of JSON
    --seed arg (=0)           Seed value for random sample
    --sample_size arg (=1000) Sample size for random sample


.. _pcl_command:

``pcl`` command
------------------------------------------------------------------------------

The *pcl* command is used to invoke a PCL JSON pipeline. See
:ref:`pcl_block_tutorial` for more information.

::

    -i [ --input ] arg      input file name
    -o [ --output ] arg     output file name
    -p [ --pcl ] arg        pcl file name
    -z [ --compress ]       Compress output data (if supported by output format)

The *pcl* command is only available when PDAL is build with PCL support.


.. _pipeline_command:

``pipeline`` command
------------------------------------------------------------------------------

The *pipeline* command is used to execute :ref:`pipeline` XML. See :ref:`reading` 
or :ref:`pipeline` for more information.

::

    -i [ --input ] arg           input file name
    --pipeline-serialization arg
    --validate                   Validate the pipeline (including serialization),
                               but do not execute writing of points
    --count arg (=0)             How many points should we write?
    --skip arg (=0)              How many points should we skip?


.. _translate_command:

``translate`` command 
------------------------------------------------------------------------------

The *translate* command is used for simple conversion of files based on their 
file extensions. Use the :ref:`pipeline_command` for more significant 
translation operations.

::

    -i [ --input ] arg           input file name
    -o [ --output ] arg          output file name
    --a_srs arg                  Assign input coordinate system (if supported by
                               output format)
    --t_srs arg                  Transform to output coordinate system (if
                               supported by output format)
    -z [ --compress ]            Compress output data (if supported by output
                               format)
    --count arg (=0)             How many points should we write?
    --skip arg (=0)              How many points should we skip?
    --bounds arg                 Extent (in XYZ to clip output to)
    --polygon arg                POLYGON WKT to use for precise crop of data (2d
                               or 3d)
    --scale arg                  A comma-separated or quoted, space-separated
                               list of scales to set on the output file:
                               --scale 0.1,0.1,0.00001
                               --scale "0.1 0.1 0.00001"
    --offset arg                 A comma-separated or quoted, space-separated
                               list of offsets to set on the output file:
                               --offset 0,0,0
                               --offset "1234 5678 91011"
    -m [ --metadata ] [=arg(=1)] Forward metadata (VLRs, header entries, etc)
                               from previous stages

The translate command can be augmented by specifying full-path options at the 
command line invocation. For example, the following invocation will translate 
`1.2-with-color.las` to `output.laz` while doing the following:

* Setting the creation day of year to 42
* Setting the creation year to 2014
* Setting the LAS point format to 1
* Cropping the file with the given polygon

```
./bin/pdal translate \
    --drivers.las.writer.creation_doy="42" \
    --drivers.las.writer.creation_year="2014" \
    --drivers.las.writer.format="1" \
    --filters.crop.polygon="POLYGON ((636889.412951239268295 851528.512293258565478 422.7001953125,636899.14233423944097 851475.000686757150106 422.4697265625,636899.14233423944097 851475.000686757150106 422.4697265625,636928.33048324030824 851494.459452757611871 422.5400390625,636928.33048324030824 851494.459452757611871 422.5400390625,636928.33048324030824 851494.459452757611871 422.5400390625,636976.977398241520859 851513.918218758190051 424.150390625,636976.977398241520859 851513.918218758190051 424.150390625,637069.406536744092591 851475.000686757150106 438.7099609375,637132.647526245797053 851445.812537756282836 425.9501953125,637132.647526245797053 851445.812537756282836 425.9501953125,637336.964569251285866 851411.759697255445644 425.8203125,637336.964569251285866 851411.759697255445644 425.8203125,637473.175931254867464 851158.795739248627797 435.6298828125,637589.928527257987298 850711.244121236610226 420.509765625,637244.535430748714134 850511.791769731207751 420.7998046875,636758.066280735656619 850667.461897735483944 434.609375,636539.155163229792379 851056.63721774588339 422.6396484375,636889.412951239268295 851528.512293258565478 422.7001953125))" \
    ./test/data/1.2-with-color.las \
    output.laz
```
