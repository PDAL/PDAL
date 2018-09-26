.. _readers.slpk:

readers.slpk
===========

`Scene Layer Packages (SLPK)`_ is a specification created by Esri as a format
for their 3D Scene Layer and scene services. SLPK is a format that allows you
to package all the necessary `I3S`_ files together and store them locally rather
than find information through REST.

Example
--------------------------------------------------------------------------------
This example will unarchive the slpk file, store it in a temp directory, and traverse it. The data will be output to a las file. This is done through PDAL's command line interface or through the pipeline.

.. code-block:: json
    {
        "pipeline":[
            {
                "type": "readers.slpk",
                "filename": "PDAL/test/data/i3s/SMALL_AUTZEN_LAS_All.slpk",
                "bounds": "([-123.075542,-123.06196],[44.049719,44.06278])"
            }
        ]
    }

``pdal traslate  PDAL/test/data/i3s/SMALL_AUTZEN_LAS_All.slpk \
      autzen.las \
      --readers.slpk.bounds="([-123.075542,-123.06196],[44.049719,44.06278])"``

Options
--------------------------------------------------------------------------------
filename
    SLPK file must have a file extension of .slpk.
    Example: ``pdal translate /PDAL/test/data/i3s/SMALL_AUTZEN_LAS_ALL.slpk output.las``

bounds
    The bounds refers to the extents of the resource in X, Y, Z coordinates with the Z dimension being optional. This must be input as a string.
    Example:``readers.slpk.bounds="([xmin,xmax],[ymin,ymax],[zmin,zmax])"``

dimensions
    These are the dimensions that the user would like to use for this read of the SLPK archive. Only these dimensions will be added to the layout. Here is a list of supported dimensions and their corresponding PDAL dimensions:
        Esri            Pdal
        =============== ===============
        INTENSITY       Intensity
        CLASS_CODE      ClassFlags
        FLAGS           Flag
        RETURNS         NumberOfReturns
        USER_DATA       UserData
        POINT_SRC_ID    PointSourceId
        GPS_TIME        GpsTime
        SCAN_ANGLE      ScanAngleRank
        RGB             Red

    Example: ``--readers.slpk.dimensions="rgb, intensity"``
lod
    This is the density of the points in the nodes that will be selected during the read. A density of 0 will select any nodes with a density calculated to be between 0 and 0.5. A higher density means the same extents will be looked at, but more points within thos bounds will be viewed. This number may change between files.
    Example: ``--readers.slpk.lod=2``

.. _Scene Layer Packages (SLPK): https://github.com/Esri/i3s-spec/blob/master/format/Indexed%203d%20Scene%20Layer%20Format%20Specification.md#_8_1
.. _I3S: https://pdal.io/readers.i3s.html
