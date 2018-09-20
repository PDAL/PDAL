.. _readers.i3s:

readers.i3s
===========

`Indexed 3d Scene Layer (I3S)`_ and `Scene Layer Packages (SLPK)`_ are
specifications created by Esri as a format for their 3D Scene Layer and scene services. The I3S reader both handles the reading of SLPK archived files locally as well as data stored in the I3S format remotely.

Example
-------
This example will download the Autzen dataset from the arcgis scene server, reproject it to another spatial reference, and output it to a las file. This is done through PDAL's command line interface.

``pdal traslate i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer autzen.las --reprojection filters.reprojection.out_srs=EPSG:3857 --readers.i3s.threads=64``

Options
-------
filename
    I3S file stored remotely, or a SLPK file stored locally. Remote files must be prefaced with an "i3s://", and local files must have the ".slpk" file extension.
    Exmaple remote file: ``pdal translate i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer autzen.las``
    Example local file: ``pdal info /PDAL/test/data/i3s/SMALL_AUTZEN_LAS_ALL.slpk``

threads
    This specifies the number of threads that you would like to use while reading. The default number of threads to buse used is 8. This affects the speed at which files are fetched and added to the PDAL view.
    Example: ``--readers.i3s.threads=64``

bounds
    The bounds refers to the extents of the resource in X, Y, Z coordinates with the Z dimension being optional. This must be input as a string.
    Example:``readers.i3s.bounds="([xmin,xmax],[ymin,ymax],[zmin,zmax])"``

.. _Indexed 3d Scene Layer (I3S): https://github.com/Esri/i3s-spec/blob/master/format/Indexed%203d%20Scene%20Layer%20Format%20Specification.md
.. _Scene Layer Packages (SLPK): https://github.com/Esri/i3s-spec/blob/master/format/Indexed%203d%20Scene%20Layer%20Format%20Specification.md#_8_1
