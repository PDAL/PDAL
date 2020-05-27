.. _readers.i3s:

readers.i3s
===========

`Indexed 3d Scene Layer (I3S)`_ is a specification created by Esri as a format for their 3D Scene Layer and scene services. The I3S reader handles RESTful webservices in an I3S file structure/format.

Example
--------------------------------------------------------------------------------
This example will download the Autzen dataset from the arcgis scene server and output it to a las file. This is done through PDAL's command line interface or through the pipeline.

.. code-block:: json

  [
      {
          "type": "readers.i3s",
          "filename": "https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer",
          "bounds": "([-123.075542,-123.06196],[44.049719,44.06278])"
      }
  ]

.. code::

    pdal translate i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer \
        autzen.las \
        --readers.i3s.threads=64 \
        --readers.i3s.bounds="([-123.075542,-123.06196],[44.049719,44.06278])"

Options
--------------------------------------------------------------------------------

.. include:: reader_opts.rst

filename
    I3S file stored remotely. These must be prefaced with an "i3s://".

    Example remote file: ``pdal translate i3s://https://tiles.arcgis.com/tiles/arcgis/rest/services/AUTZEN_LiDAR/SceneServer autzen.las``

threads
    This specifies the number of threads that you would like to use while
    reading. The default number of threads to be used is 8. This affects
    the speed at which files are fetched and added to the PDAL view.

    Example: ``--readers.i3s.threads=64``

bounds
    The bounds refers to the extents of the resource in X, Y, Z coordinates with the Z dimension being optional. This must be input as a string.

    Example:``readers.i3s.bounds="([xmin,xmax],[ymin,ymax],[zmin,zmax])"``

dimensions
    Comma-separated list of dimensions that should be read.  Specify the
    Esri name, rather than the PDAL dimension name.

        =============   ===============
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
        =============   ===============

    Example: ``--readers.i3s.dimensions="returns, rgb"``

min_density and max_density
    This is the range of density of the points in the nodes that will be selected during the read. The density of a node is calculated by the vertex count divided by the effective area of the node. Nodes do not have a uniform density acrossdepths in the tree, so some sections may be more or less dense than others. The default values for these parameters will pull all the leaf nodes (the highest resolution).

    Example: ``--readers.i3s.min_density=2 --readers.i3s.max_density=2.5``

.. _Indexed 3d Scene Layer (I3S): https://github.com/Esri/i3s-spec/blob/master/format/Indexed%203d%20Scene%20Layer%20Format%20Specification.md
