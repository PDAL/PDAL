.. _translate_command:

********************************************************************************
translate
********************************************************************************

The ``translate`` command can be used for simple conversion of files based on
their file extensions. It can also be used for constructing pipelines directly
from the command-line.

::

    $ pdal translate <input> <output>

::

    -i [ --input ] arg    input file name
    -o [ --output ] arg   output file name
    -p [ --pipeline ] arg pipeline output
    -r [ --reader ] arg   reader type
    -f [ --filter ] arg   filter type
    -w [ --writer ] arg   writer type

The ``--input`` and ``--output`` file names are required options.

The ``--pipeline`` file name is optional. If given, the pipeline constructed
from the command-line arguments will be written to disk for reuse in the
:ref:`pipeline_command`.

The ``--filter`` flag is optional. It is used to specify the driver used to
filter the data. ``--filter`` accepts multiple arguments if provided, thus
constructing a multi-stage filtering operation.

If no ``--reader`` or ``--writer`` type are given, PDAL will attempt to infer
the correct drivers from the input and output file name extensions respectively.

Example 1:
--------------------------------------------------------------------------------

The ``translate`` command can be augmented by specifying full-path options at
the command-line invocation. For example, the following invocation will
translate ``1.2-with-color.las`` to ``output.laz`` while doing the following:

* Setting the creation day of year to 42
* Setting the creation year to 2014
* Setting the LAS point format to 1
* Cropping the file with the given polygon

::

    $ pdal translate \
        --writers.las.creation_doy="42" \
        --writers.las.creation_year="2014" \
        --writers.las.format="1" \
        --filters.crop.polygon="POLYGON ((636889.412951239268295 851528.512293258565478 422.7001953125,636899.14233423944097 851475.000686757150106 422.4697265625,636899.14233423944097 851475.000686757150106 422.4697265625,636928.33048324030824 851494.459452757611871 422.5400390625,636928.33048324030824 851494.459452757611871 422.5400390625,636928.33048324030824 851494.459452757611871 422.5400390625,636976.977398241520859 851513.918218758190051 424.150390625,636976.977398241520859 851513.918218758190051 424.150390625,637069.406536744092591 851475.000686757150106 438.7099609375,637132.647526245797053 851445.812537756282836 425.9501953125,637132.647526245797053 851445.812537756282836 425.9501953125,637336.964569251285866 851411.759697255445644 425.8203125,637336.964569251285866 851411.759697255445644 425.8203125,637473.175931254867464 851158.795739248627797 435.6298828125,637589.928527257987298 850711.244121236610226 420.509765625,637244.535430748714134 850511.791769731207751 420.7998046875,636758.066280735656619 850667.461897735483944 434.609375,636539.155163229792379 851056.63721774588339 422.6396484375,636889.412951239268295 851528.512293258565478 422.7001953125))" \
        ./test/data/1.2-with-color.las \
        output.laz

Example 2:
--------------------------------------------------------------------------------

Given these tools, we can now construct a custom pipeline on-the-fly. The
example below uses a simple LAS reader and writer, but stages a PCL-based
voxel grid filter, followed by the PCL-based ground filter. We can even set
stage-specific parameters as shown.

::

    $ pdal translate input.las output.las \
        --filter filters.pclblock filters.ground \
        --filters.pclblock.json="{\"pipeline\":{\"filters\":[{\"name\":\"VoxelGrid\"}]}}" \
        --filters.ground.approximate=true --filters.ground.extract=true



