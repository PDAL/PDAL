.. _readers.i3s:

readers.i3s
===========

`Indexed 3d Scene Layer (I3S)`_ is a specification created by Esri as a format for their 3D Scene Layer and scene services. The I3S reader handles RESTful webservices in an I3S file structure/format.

Example
--------------------------------------------------------------------------------
This example will download the Autzen dataset from the arcgis scene server and output it to a las file. This is done through PDAL's command line interface or through the pipeline.

.. code-block:: json
    {
        "pipeline":[
            {
                "type": "readers.i3s",
                "filename": "https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer",
                "bounds": "([-123.075542,-123.06196],[44.049719,44.06278])"
            }
        ]
    }

``pdal traslate i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer \
        autzen.las \
        --readers.i3s.threads=64 \
        --readers.i3s.bounds="([-123.075542,-123.06196],[44.049719,44.06278])"``

Options
--------------------------------------------------------------------------------
filename
    I3S file stored remotely. These must be prefaced with an "i3s://".
    Exmaple remote file: ``pdal translate i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer autzen.las``

threads
    This specifies the number of threads that you would like to use while reading. The default number of threads to be used is 8. This affects the speed at which files are fetched and added to the PDAL view.
    Example: ``--readers.i3s.threads=64``

bounds
    The bounds refers to the extents of the resource in X, Y, Z coordinates with the Z dimension being optional. This must be input as a string.
    Example:``readers.i3s.bounds="([xmin,xmax],[ymin,ymax],[zmin,zmax])"``

dimensions
    These are the dimensions that the user would like to use for this read of the I3S files. Only these dimensions will be added to the layout. Here is a list of supported I3S dimensions and their corresponding PDAL dimensions:
        Esri            Pdal
        =============   ===============
        INTENSITY       Intensity
        CLASS_CODE      ClassFlags
        FLAGS           Flag
        RETURNS         NumberOfReturns
        USER_DATA       UserData
        POINT_SRC_ID    PointSourceId
        GPS_TIME        GpsTime
        SCAN_ANGLE      ScanAngleRank
        RGB             Red

    Example: ``--readers.i3s.dimensions="returns, rgb"``

depth
    The depth refers to the depth of the node tree that the user would like to see data from. This essentially alters the viewed resolution. This will select the highest resolution closest to the depth selected. 0 represents the 0th depth of the tree, and the minimum resolution of the data. Selecting nothing here will output the highest resolution in every node subtree.

    Example: ``--readers.i3s.depth=2``

.. _Indexed 3d Scene Layer (I3S): https://github.com/Esri/i3s-spec/blob/master/format/Indexed%203d%20Scene%20Layer%20Format%20Specification.md
