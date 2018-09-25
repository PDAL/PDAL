.. _readers.slpk:

readers.slpk
===========

`Scene Layer Packages (SLPK)`_ is a specification created by Esri as a format
for their 3D Scene Layer and scene services. SLPK is a format that allows you
to package all the necessary I3S files together and store them locally rather
than find information through REST.

Example
--------------------------------------------------------------------------------
This example will unarchive the slpk file, store it in a temp directory, and
traverse it. The data will be reprojected to another spatial reference, and
output to a las file. This is done through PDAL's command line interface or
through the pipeline.

.. code-block:: json
    {
        "pipeline":[
            {
                "type": "readers.slpk",
                "filename": "PDAL/test/data/i3s/SMALL_AUTZEN_LAS_All.slpk"
                "bounds": "([-123.075542,-123.06196],[44.049719,44.06278])"
            }
        ]
    }

``pdal traslate  PDAL/test/data/i3s/SMALL_AUTZEN_LAS_All.slpk \
        autzen.las \
        --reprojection filters.reprojection.out_srs=EPSG:3857 \
        --readers.i3s.threads=64 \
        --readers.i3s.bounds="([-123.075542,-123.06196],[44.049719,44.06278])"``

Options
--------------------------------------------------------------------------------
filename
    SLPK file stored locally and must have a file extension of .slpk
    Example: ``pdal info /PDAL/test/data/i3s/SMALL_AUTZEN_LAS_ALL.slpk``

threads
    This specifies the number of threads that you would like to use while reading. The default number of threads to buse used is 8. This affects the speed at which files are fetched and added to the PDAL view.
    Example: ``--readers.slpk.threads=numThreads``

bounds
    The bounds refers to the extents of the resource in X, Y, Z coordinates with the Z dimension being optional. This must be input as a string.
    Example:``readers.slpk.bounds="([xmin,xmax],[ymin,ymax],[zmin,zmax])"``

.. _Scene Layer Packages (SLPK): https://github.com/Esri/i3s-spec/blob/master/format/Indexed%203d%20Scene%20Layer%20Format%20Specification.md#_8_1
