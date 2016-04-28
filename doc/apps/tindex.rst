.. _tindex_command:

********************************************************************************
tindex
********************************************************************************

The ``tindex`` command is used to create a `GDAL`_-style tile index for
PDAL-readable point cloud types (see `gdaltindex`_).

The ``tindex`` command has two modes.  The first mode creates a spatial index
file for a set of point cloud files.  The second mode creates a point cloud
file that is the result of merging the points from files referred to in a
spatial index file that meet some criteria (usually a geographic region filter).

tindex Creation Mode
--------------------------------------------------------------------------------

::

    $ pdal tindex <tindex> <filespec>

This command will index the files referred to by ``filespec`` and place the
result in ``tindex``.  The ``tindex`` is a vector file or database that
will be created by ``pdal`` as necessary to store the file index.
The type of the index
file can be specified by specifying the OGR code for the format using the
``--driver`` option.  If no driver is specified, the format defaults to "ESRI
Shapefile".  Any filetype that can be handled by
`OGR <http://www.gdal.org/ogr_formats.html>`_ is acceptable.

In vector file-speak, each file specified by ``filespec`` is stored as a
feature in a layer in the index file. The ``filespec`` is a `glob pattern
<http://man7.org/linux/man-pages/man7/glob.7.html>`_.  and normally needs to be
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
--------------------------------------------------------------------------------

::

    $ pdal tindex --merge <tindex> <filespec>

This command will read the existing index file ``tindex`` and merge the
points in the indexed files that pass any filter that might be specified,
writing the output to the point cloud file specified in ``filespec``.
The type of the output file is determined automatically from the filename
extension.

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
--------------------------------------------------------------------------------

Find all LAS files via ``find``, send that file list via STDIN to
``pdal tindex``, and write a SQLite tile index file with a layer named ``pdal``:

::

    $ find las/ -iname "*.las" | pdal tindex index.sqlite -f "SQLite" \
        --stdin --lyr_name pdal

Example 2:
--------------------------------------------------------------------------------

Glob a list of LAS files, output the SRS for the index entries to EPSG:4326, and
write out an `SQLite`_ file.

::

    $ pdal tindex index.sqlite "*.las" -f "SQLite" --lyr_name "pdal" \
        --t_srs "EPSG:4326"


.. _`SQLite`: http://www.sqlite.org
.. _`gdaltindex`: http://www.gdal.org/gdaltindex.html
.. _`GDAL`: http://www.gdal.org

