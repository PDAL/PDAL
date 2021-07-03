.. _readers.slpk:

readers.slpk
============

`Scene Layer Packages (SLPK)`_ is a specification created by Esri as a format
for their 3D Scene Layer and scene services. SLPK is a format that allows you
to package all the necessary :ref:`I3S <readers.i3s>` files together and store them locally rather
than find information through REST.

Example
--------------------------------------------------------------------------------
This example will unarchive the slpk file, store it in a temp directory,
and traverse it. The data will be output to a las file. This is done
through PDAL's command line interface or through the pipeline.

.. code-block:: json

  [
      {
          "type": "readers.slpk",
          "filename": "PDAL/test/data/i3s/SMALL_AUTZEN_LAS_All.slpk",
          "obb": {
              "center": [
                  636590,
                  849216,
                  460
              ],
              "halfSize": [
                  590,
                  281,
                  60
              ],
              "quaternion":
              [
                  0,
                  0,
                  0,
                  1
              ]
          }
      }
  ]

::

    pdal traslate  PDAL/test/data/i3s/SMALL_AUTZEN_LAS_All.slpk autzen.las

Options
--------------------------------------------------------------------------------
filename
    SLPK file must have a file extension of .slpk.
    Example: ``pdal translate /PDAL/test/data/i3s/SMALL_AUTZEN_LAS_ALL.slpk output.las``

.. include:: reader_opts.rst

obb
    An oriented bounding box used to filter the data being retrieved.  The obb
    is specified as JSON exactly as described by the `I3S specification`_.

dimensions
    Comma-separated list of dimensions that should be read.  Specify the
    Esri name, rather than the PDAL dimension name.

        =============== ===============
        Esri            PDAL
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
        =============== ===============

    Example: ``--readers.slpk.dimensions="rgb, intensity"``

min_density and max_density
    This is the range of density of the points in the nodes that will
    be selected during the read. The density of a node is calculated by
    the vertex count divided by the effective area of the node. Nodes do
    not have a uniform density across depths in the tree, so some sections
    may be more or less dense than others. Default values for these
    parameters will select all leaf nodes (the highest resolution).

    Example: ``--readers.slpk.min_density=2 --readers.slpk.max_density=2.5``

.. _Scene Layer Packages (SLPK): https://github.com/Esri/i3s-spec/blob/master/format/Indexed%203d%20Scene%20Layer%20Format%20Specification.md#_8_1
.. _I3S specification: https://github.com/Esri/i3s-spec/blob/master/docs/2.0/obb.cmn.md
