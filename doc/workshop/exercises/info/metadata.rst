.. _workshop-metadata:

Printing file metadata
================================================================================

.. include:: ../../includes/substitutions.rst

.. index:: metadata, coordinate system, spatial reference system

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to print metadata information. Issue the
following command in your `Conda Shell`.

.. code-block:: console

    $ pdal info ./exercises/info/interesting.las --metadata
    {
    "file_size": 37698,
    "filename": "./exercises/info/interesting.las",
    "metadata":
    {
        "comp_spatialreference": "PROJCS[\"NAD_1983_Oregon_Statewide_Lambert_Feet_Intl\",GEOGCS[\"GCS_North_American_1983\",DATUM[\"North_American_Datum_1983\",SPHEROID[\"GRS_1980\",6378137,298.257222101]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]]],PROJECTION[\"Lambert_Conformal_Conic_2SP\"],PARAMETER[\"latitude_of_origin\",41.75],PARAMETER[\"central_meridian\",-120.5],PARAMETER[\"standard_parallel_1\",43],PARAMETER[\"standard_parallel_2\",45.5],PARAMETER[\"false_easting\",400000],PARAMETER[\"false_northing\",0],UNIT[\"foot\",0.3048,AUTHORITY[\"EPSG\",\"9002\"]],AXIS[\"Easting\",EAST],AXIS[\"Northing\",NORTH]]",
        "compressed": false,
        "copc": false,
        "count": 1065,
        "creation_doy": 145,
        "creation_year": 2012,
        "dataformat_id": 3,
        "dataoffset": 1488,
        "filesource_id": 0,
        "global_encoding": 0,
        "global_encoding_base64": "AAA=",
        "gtiff": "Geotiff_Information:\n   Version: 1\n   Key_Revision: 1.0\n   Tagged_Information:\n      End_Of_Tags.\n   Keyed_Information:\n      GTModelTypeGeoKey (Short,1): ModelTypeProjected\n      GTRasterTypeGeoKey (Short,1): RasterPixelIsArea\n      GTCitationGeoKey (Ascii,44): \"NAD_1983_Oregon_Statewide_Lambert_Feet_Intl\"\n      GeographicTypeGeoKey (Short,1): User-Defined\n      GeogCitationGeoKey (Ascii,106): \"GCS Name = GCS_North_American_1983|Datum = D_North_American_1983|Ellipsoid = GRS_1980|Primem = Greenwich|\"\n      GeogGeodeticDatumGeoKey (Short,1): User-Defined\n      GeogAngularUnitsGeoKey (Short,1): Angular_Degree\n      GeogEllipsoidGeoKey (Short,1): User-Defined\n      GeogSemiMajorAxisGeoKey (Double,1): 6378137          \n      GeogInvFlatteningGeoKey (Double,1): 298.257222101    \n      GeogPrimeMeridianLongGeoKey (Double,1): 0                \n      ProjectedCSTypeGeoKey (Short,1): User-Defined\n      ProjectionGeoKey (Short,1): User-Defined\n      ProjCoordTransGeoKey (Short,1): CT_LambertConfConic_2SP\n      ProjLinearUnitsGeoKey (Short,1): Linear_Foot\n      ProjStdParallel1GeoKey (Double,1): 43               \n      ProjStdParallel2GeoKey (Double,1): 45.5             \n      ProjFalseOriginLongGeoKey (Double,1): -120.5           \n      ProjFalseOriginLatGeoKey (Double,1): 41.75            \n      ProjFalseOriginEastingGeoKey (Double,1): 400000           \n      ProjFalseOriginNorthingGeoKey (Double,1): 0                \n      End_Of_Keys.\n   End_Of_Geotiff.\n",
        "header_size": 227,
        "major_version": 1,
        "maxx": 638982.55,
        "maxy": 853535.43,
        "maxz": 586.38,
        "minor_version": 2,
        "minx": 635619.85,
        "miny": 848899.7,
        "minz": 406.59,
        "offset_x": 0,
        "offset_y": 0,
        "offset_z": 0,
        "point_length": 34,
        "project_id": "00000000-0000-0000-0000-000000000000",
        "scale_x": 0.01,
        "scale_y": 0.01,
        "scale_z": 0.01,
        "software_id": "HOBU-GENERATING",
        ...

.. note::

    PDAL :ref:`metadata <metadata>` is returned a in a tree
    structure corresponding to processing pipeline that produced
    it.

.. seealso::

    Use the |JSON| processing capabilities of your favorite processing
    software to selectively access and manipulate values.

    * `Python JSON library`_
    * `jsawk`_ (like ``awk`` but for JSON data)
    * `jq`_ (command line processor for JSON)
    * `Ruby JSON library`_

.. _`Python JSON library`: https://docs.python.org/3/library/json.html
.. _`jsawk`: https://github.com/micha/jsawk
.. _`jq`: https://stedolan.github.io/jq/
.. _`Ruby JSON library`: https://ruby-doc.org/stdlib-3.0.2/libdoc/json/rdoc/JSON.html


Structured Metadata Output
................................................................................

Many command-line utilities output their data in a human-readable custom
format. The downsides to this approach are significant. PDAL was designed to be
used in the context of other software tools driving it. For example, it is
quite common for PDAL to be used in data validation scenarios. Other programs
might need to inspect information in PDAL's output and then act based on the
values. A human-readable format would mean that downstream program would need
to write a parser to consume PDAL's special format.

|JSON| provides a nice balance between human- and machine- readable, but
even then it can be quite hard to find what you're looking for, especially
if the output is long. ``pdal`` command output used in conjunction with a
JSON parsing tool like ``jq`` provide a powerful inspection combination.

For example, we might only care about the ``system_id`` and ``compressed``
flag for this particular file. Our simple ``pdal info --metadata`` command
gives us that, but it also gives us a bunch of other stuff we don't need
at the moment. Let's focus on extracting what we want using the
``jq`` command.

.. code-block:: console

    $ pdal info ./exercises/info/interesting.las --metadata \
    | jq  ".metadata.compressed, .metadata.system_id"
    false
    "HOBU-SYSTEMID"


.. code-block:: doscon

    > pdal info ./exercises/info/interesting.las --metadata ^
    | jq  ".metadata.compressed, .metadata.system_id"
    false
    "HOBU-SYSTEMID"


.. note::

    PDAL's JSON output is very powerfully combined with the processing
    capabilities of other programming languages such as JavaScript or Python.
    Both of these languages have excellent built-in tools for consuming
    JSON, along with plenty of other features to allow you to do something
    with the data inside the data structures. As we will see later
    in the workshop, this PDAL feature is one that makes construction
    of custom data processing workflows with PDAL very convenient.



Notes
--------------------------------------------------------------------------------

1. PDAL uses |JSON| as the exchange format when printing information from
   :ref:`info_command`.  JSON provides human and machine-readable text data.

2. The PDAL :ref:`metadata document <metadata>` contains background and
   information about specific metadata entries and what they mean.

3. Metadata available for a given file depends on the stage that produces the
   data. :ref:`Readers <readers>` produce same-named values where possible, but
   it is common that variables are different. :ref:`Filters <filters>` and even
   :ref:`writers <writers>` can also produce metadata entries.

4. Spatial reference system or coordinate system information is a kind of
   special metadata.  Spatial references are come directly from source data
   or are provided via options in PDAL.
