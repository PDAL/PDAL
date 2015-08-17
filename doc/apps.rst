.. _apps:

******************************************************************************
PDAL Applications
******************************************************************************

:Author: Howard Butler
:Contact: howard at hobu.co
:Date: 6/1/2015

PDAL contains consists of a single application, called ``pdal``. The ```pdal``
application currently has the following subcommands:

* :ref:`delta <delta_command>`
* :ref:`diff <diff_command>`
* :ref:`ground <ground_command>`
* :ref:`info <info_command>`
* :ref:`pcl <pcl_command>`
* :ref:`pipeline <pipeline_command>`
* :ref:`random <random_command>`
* :ref:`split <split_command>`
* :ref:`tindex <tindex_command>`
* :ref:`translate <translate_command>`
* :ref:`view <view_command>`

Applications are run by invoking the ``pdal`` application along with the
command name:

::

    $ pdal info myfile.las
    $ pdal translate input.las output.las
    $ pdal pipeline --stdin < myxml.xml

Help for each command can be retrieved via the ``--help`` switch. The
``--drivers`` and ``--options`` switches can tell you more about particular
drivers and their options:

::

    $ pdal info --help
    $ pdal translate --drivers
    $ pdal pipeline --options writers.las

Additional driver-specific options may be specified by using a
namespace-prefixed option name. For example, it is possible to set the LAS day
of year at translation time with the following option:

::

    $ pdal translate \
        --writers.las.creation_doy="42" \
        input.las \
        output.las

.. note::

    Driver specific options can be identified using the ``pdal info --options``
    invocation.


.. _delta_command:

delta command
------------------------------------------------------------------------------

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
^^^^^^^^^^^^^

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
^^^^^^^^^^

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
    ....


.. _diff_command:

diff command
------------------------------------------------------------------------------

The ``diff`` command is used for executing a simple contextual difference
between two sources.

::

    $ pdal diff <source> <candidate>

::

    --source arg     Non-positional option for specifying filename of source file.
    --candidate arg  Non-positional option for specifying filename to test against source.

The command returns 0 and produces no output if the files describe the same
point data in the same format, otherwise 1 is returned and a JSON-formatted
description of the differences is produced.

The command checks for the equivalence of the following items:

* Different schema
* Expected count
* Metadata
* Actual point count
* Byte-by-byte point data


.. _ground_command:

ground command
------------------------------------------------------------------------------

The ``ground`` command is used to segment the input point cloud into ground
versus non-ground returns. The output is a point cloud containing only ground
returns. The ``ground`` command invokes `Point Cloud Library
<http://pointclouds.org/>`_'s `ProgressiveMorphologicalFilter`_.

.. note::

    The ``ground`` command is only available when PDAL is linked with PCL.

::

    $ pdal ground <input> <output>

::

    --input [-i] arg       Non-positional option for specifying input filename
    --output [-o] arg      Non-positional option for specifying output filename
    --maxWindowSize arg    max window size [33]
    --slope arg            slope [1]
    --maxDistance arg      max distance [2.5]
    --initialDistance arg  initial distance [0.15]
    --cellSize arg         cell size [1]
    --classify             apply classification labels? [true]
    --extract              extract ground returns? [false]
    --approximate [-a]     Use significantly faster approximate algorithm? [false]


.. _info_command:

info command
------------------------------------------------------------------------------

Dumps information about a point cloud file, such as:

* basic properties (extents, number of points, point format)
* coordinate reference system
* additional metadata
* summary statistics about the points
* the plain text format should be reStructured text if possible to allow a user
  to retransform the output into whatever they want with ease

::

    $ pdal info <input>

::

    --input arg       Non-positional argument to specify input filename.
    --point [-p] arg  Display points for particular points.  Points can be specified in
                      a range or list: 4-10, 15, 255-300.
    --query arg       Add a listing of points based on the distance from the provided
                      location.  The number of points returned can be limited by
                      providing an optional count.
                      --query "25.34,35.123/3" or --query "11532.23 -10e23 1.234/10"
    --stats           Display the minimum, maximum, average and count of each
                      dimension.
    --boundary        Compute a hexagonal boundary that contains all points.
    --dimensions arg  Use with --stats to limit the dimensions on which statistics
                      should be computed.
                      --dimensions "X, Y,Red"
    --schema          Dump the schema of the internal point storage.
    --pipeline-serialization
                      Create a JSON representation of the pipeline used to generate
                      the output.
    --summary         Dump the point count, spatial reference, extrema and dimension
                      names.
    --metadata        Dump the metadata associated with the input file.

If no options are provided, ``--stats`` is assumed.

Example 1:
^^^^^^^^^^^^

::

    $ pdal info  test/data/las/1.2-with-color.las \
        --query="636601.87, 849018.59, 425.10"
    {
      "0":
      {
        "Blue": 134,
        "Classssification": 1,
        "EdgeOfFlightLine": 0,
        "GpsTime": 245383.38808001476,
        "Green": 104,
        "Intensity": 124,
        "NumberOfReturns": 1,
        "PointSourceId": 7326,
        "Red": 134,
        "ReturnNumber": 1,
        "ScanAngleRank": -4,
        "ScanDirectionFlag": 1,
        "UserData": 126,
        "X": 636601.87,
        "Y": 849018.59999999998,
        "Z": 425.10000000000002
      },
      "1":
      {
        "Blue": 134,
        "Classification": 2,
        "EdgeOfFlightLine": 0,
        "GpsTime": 246099.17323102333,
        "Green": 106,
        "Intensity": 153,
        "NumberOfReturns": 1,
        "PointSourceId": 7327,
        "Red": 143,
        "ReturnNumber": 1,
        "ScanAngleRank": -10,
        "ScanDirectionFlag": 1,
        "UserData": 126,
        "X": 636606.76000000001,
        "Y": 849053.94000000006,
        "Z": 425.88999999999999
      },
      ...

Example 2:
^^^^^^^^^^

::

    $ pdal info test/data/1.2-with-color.las -p 0-10
    {
      "filename": "../../test/data/las/1.2-with-color.las",
      "pdal_version": "PDAL 1.0.0.b1 (116d7d) with GeoTIFF 1.4.1 GDAL 1.11.2 LASzip 2.2.0",
      "points":
      {
        "point":
        [
          {
            "Blue": 88,
            "Classification": 1,
            "EdgeOfFlightLine": 0,
            "GpsTime": 245380.78254962614,
            "Green": 77,
            "Intensity": 143,
            "NumberOfReturns": 1,
            "PointId": 0,
            "PointSourceId": 7326,
            "Red": 68,
            "ReturnNumber": 1,
            "ScanAngleRank": -9,
            "ScanDirectionFlag": 1,
            "UserData": 132,
            "X": 637012.23999999999,
            "Y": 849028.31000000006,
            "Z": 431.66000000000003
          },
          {
            "Blue": 68,
            "Classification": 1,
            "EdgeOfFlightLine": 0,
            "GpsTime": 245381.45279923646,
            "Green": 66,
            "Intensity": 18,
            "NumberOfReturns": 2,
            "PointId": 1,
            "PointSourceId": 7326,
            "Red": 54,
            "ReturnNumber": 1,
            "ScanAngleRank": -11,
            "ScanDirectionFlag": 1,
            "UserData": 128,
            "X": 636896.32999999996,
            "Y": 849087.70000000007,
            "Z": 446.38999999999999
          },
          ...


.. _pcl_command:

pcl command
------------------------------------------------------------------------------

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


.. _pipeline_command:

pipeline command
------------------------------------------------------------------------------

The ``pipeline`` command is used to execute :ref:`pipeline` XML. See
:ref:`reading` or :ref:`pipeline` for more information.

::

    $ pdal pipeline <input>

::

    --input [-i] arg  Non-positional argument to specify input file name.
    --pipeline-serialization arg
                      Write input pipeline along with all metadata and created by the
                      pipeline to the specified file.
    --validate        Validate the pipeline (including serialization), but do not execute
                      writing of points

.. note::

    The ``pipeline`` command can accept option substitutions, but they
    do not replace existing options that are specified in the input XML
    pipeline.  For example, to set the output and input LAS files for a
    pipeline that does a translation, construct XML that does not contain
    ``filename`` for reader and writer and issue the command with the
    following arguments:

    ::

        $ pdal pipeline -i translate.xml --writers.las.filename=output.laz \
            --readers.las.filename=input.las


.. _random_command:

random command
------------------------------------------------------------------------------

The ``random`` command is used to create a random point cloud. It uses
:ref:`readers.faux` to create a point cloud containing ``count`` points
drawn randomly from either a uniform or normal distribution. For the uniform
distribution, the bounds can be specified (they default to a unit cube). For
the normal distribution, the mean and standard deviation can both be set for
each of the x, y, and z dimensions.

::

    $ pdal random <output>

::

    --output [-o] arg   Non-positional argument to specify output file name.
    --compress [-z]     Compress output data (if supported by output format)
    --count arg         Number of points in created point cloud [0].
    --bounds arg        Extent (in XYZ to clip output to):
                        --bounds "([xmin,xmax],[ymin,ymax],[zmin,zmax])"
    --mean arg          List of means (for --distribution normal)
                        --mean 0.0,0.0,0.0
                        --mean "0.0 0.0 0.0"
    --stdev arg         List of standard deviations (for --distribution normal)
                        --stdev 0.0,0.0,0.0
                        --stdev "0.0 0.0 0.0"
    --distribution arg  Distribution type (uniform or normal) [uniform]


.. _split_command:

split command
------------------------------------------------------------------------------

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
^^^^^^^^^^^

::

    $ pdal split --capacity 100000 infile.laz outfile.bpf

This command takes the points from the input file ``infile.laz`` and creates
output files ``outfile_1.bpf``, ``outfile_2.bpf``, ... where each output file
contains no more than 100000 points.


.. _tindex_command:

tindex command
------------------------------------------------------------------------------

The ``tindex`` command is used to create a `GDAL`_-style tile index for
PDAL-readable point cloud types (see `gdaltindex`_).

.. note::

    The ``tindex`` command is only available when PDAL is linked with `GDAL`_.

The ``tindex`` command has two modes.  The first mode creates a spatial index
file for a set of point cloud files.  The second mode creates a point cloud
file that is the result of merging the points from files referred to in a
spatial index file that meet some criteria (usually a geographic region filter).

tindex Creation Mode
^^^^^^^^^^^^^^^^^^^^^^^^

::

    $ pdal tindex <tindex> <filespec>

This command will index the files referred to by ``filespec`` and place the
result in ``tindex``.  The ``tindex`` is a vector file or database that can be
handled by `OGR <http://www.gdal.org/ogr_formats.html>`_. The type of the index
file can be specified by specifying the OGR code for the format using the
``--driver`` option.  If no driver is specified, the format defaults to "ESRI
Shapefile".

In vector file-speak, each file specified by ``filespec`` is stored as a feature
in a layer in the index file. The ``filespec`` is a `glob pattern
<http://man7.org/linux/man-pages/man7/glob.7.html>'_.  and normally needs to be
quoted to prevent shell expansion of wildcard characters.

::

    --tindex                   Non-positional option for specifying the index file name.
    --filespec                 Non-positional option for specifying pattern of files to
                               be indexed.
    --lyr_name                 Name of layer in which to store the features. Defaults to
                               the base name of the first file indexed.
    --tindex_name              Name of the field in the feature in which to store the
                               indexed file name. ["location"]
    --driver                   OGR driver name. ["ESRI Shapefile"]
    --t_srs                    Spatial reference system in which to store index vector
                               data. ["EPSG:4326"]
    --a_srs                    Spatial reference assumed to be the reference for the
                               source data.  If the source data includes spatial reference
                               information, this value is IGNORED. ["EPSG:4326"]
    --write_absolute_path arg  Write absolute rather than relative file paths [false]

tindex Merge Mode
^^^^^^^^^^^^^^^^^^^^^

::

    $ pdal tindex --merge <tindex> <filespec>

This command will read the index file ``tindex`` and merge the points in the
files listed index file that pass any filter that might be specified, writing
the output to the point cloud file specified in ``filespec``.  The type of the
output file is determined automatically from the filename extension.

::

    --tindex    Non-positional option for specifying the index filename.
    --filespec  Non-positional option for specifying the merge output filename.
    --polygon   Well-known text representation of geometric filter.  Only
                points inside the object will be written to the output file.
    --bounds    Bounding box for clipping points.  Only points inside the box
                will be written to the output file.
                --bounds "([xmin,xmax],[ymin,ymax],[zmin,zmax])"
    --t_srs     Spatial reference system in which the output data should be
                represented. ["EPSG:4326"]

Example 1:
^^^^^^^^^^^

Find all LAS files via ``find``, send that file list via STDIN to
``pdal tindex``, and write a SQLite tile index file with a layer named ``pdal``:

::

    $ find las/ -iname "*.las" | pdal tindex index.sqlite -f "SQLite" \
        --stdin --lyr_name pdal

Example 2:
^^^^^^^^^^^

Glob a list of LAS files, output the SRS for the index entries to EPSG:4326, and
write out an `SQLite`_ file.

::

    $ pdal tindex index.sqlite "*.las" -f "SQLite" --lyr_name "pdal" \
        --t_srs "EPSG:4326"


.. _translate_command:

translate command
------------------------------------------------------------------------------

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
^^^^^^^^^^^

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
^^^^^^^^^^^

Given these tools, we can now construct a custom pipeline on-the-fly. The
example below uses a simple LAS reader and writer, but stages a PCL-based
voxel grid filter, followed by the PCL-based ground filter. We can even set
stage-specific parameters as shown.

::

    $ pdal translate input.las output.las \
        --filter filters.pclblock filters.ground \
        --filters.pclblock.json="{\"pipeline\":{\"filters\":[{\"name\":\"VoxelGrid\"}]}}" \
        --filters.ground.approximate=true --filters.ground.extract=true


.. _view_command:

view command
------------------------------------------------------------------------------

The ``view`` command can be used to visualize a point cloud using the
PCLVisualizer. The command takes a single argument, the input file name.

.. note::

    The ``view`` command is only available when PDAL is linked with PCL.

::

    $ pdal view <input>

Once the data has been loaded into the viewer, press h or H to display the
help.

::

    | Help:
    -------
              p, P   : switch to a point-based representation
              w, W   : switch to a wireframe-based representation (where available)
              s, S   : switch to a surface-based representation (where available)

              j, J   : take a .PNG snapshot of the current window view
              c, C   : display current camera/window parameters
              f, F   : fly to point mode

              e, E   : exit the interactor
              q, Q   : stop and call VTK's TerminateApp

               +/-   : increment/decrement overall point size
         +/- [+ ALT] : zoom in/out

              g, G   : display scale grid (on/off)
              u, U   : display lookup table (on/off)

        o, O         : switch between perspective/parallel projection (default = perspective)
        r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]
        CTRL + s, S  : save camera parameters
        CTRL + r, R  : restore camera parameters

        ALT + s, S   : turn stereo mode on/off
        ALT + f, F   : switch between maximized window mode and original size

              l, L           : list all available geometric and color handlers for the current actor map
        ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)
              0..9 [+ CTRL]  : switch between different color handlers (where available)

        SHIFT + left click   : select a point (start with -use_point_picking)

              x, X   : toggle rubber band selection mode for left mouse button


.. _`SQLite`: http://www.sqlite.org
.. _`gdaltindex`: http://www.gdal.org/gdaltindex.html
.. _`GDAL`: http://www.gdal.org
.. _`ProgressiveMorphologicalFilter`: http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php#progressive-morphological-filtering.
