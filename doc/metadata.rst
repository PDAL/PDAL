.. _metadata:

******************************************************************************
Metadata
******************************************************************************

:Author: Howard Butler
:Contact: hobu.inc at gmail dot com
:Date: 5/30/2012

Metadata is an important component of any data processing story. PDAL attempts
to allow users to operate with metadata in a relatively free-form way, with
its main Metadata utility, :cpp:class:`pdal::Metadata`, being a lightly
wrapped `boost::property_tree`_.

The basic structure of a :cpp:class:`pdal::Metadata` instance is the following 
tree structure:

.. code-block:: javascript

    {
        "type": "blank",
        "value": "",
        "name": "name",
        "metadata": {}
    }

.. note::
    
    Metadata instances can contain other Metadata instances by adding them 
    with the :cpp:func:`pdal::Metadata::addMetadata` method.  They will be 
    added to the `metadata` sub-tree of the internal property_tree.

Metadata Types
------------------------------------------------------------------------------

:cpp:class:`pdal::Metadata` instances are essentially a specially-organized 
boost::property_tree, and as such, require that any classes that are added 
to them be copy-constructable and have an ostream<< operator for them.  While 
these constraints mean they are all serializable to strings, 
:cpp:class:`pdal::Metadata` also keeps an explicit type variable, `type` for 
each instance. This property allows us to say something extra about the 
Metadata entry, and allows them to go in and out of :cpp:class:`pdal::Stage` and 
:cpp:class:`pdal::PointBuffer` with type fidelity. 

The metadata `type` variable roughly maps to the `XSD type names`_.  The following 
types are valid for current PDAL versions, though more may be added.

.. csv-table:: PDAL :cpp:class:`pdal::Metadata` types

    double, float, integer
    nonNegativeInteger, boolean, string
    base64Binary, uuid, bounds
    spatialreference, blank

.. _`XSD type names`:  http://infohost.nmt.edu/tcc/help/pubs/rnc/xsd.html

.. warning::

    Explicitly-sized types are not supported. Assume that `integer` or
    `nonNegativeInteger` map to the typical 4-byte signed and unsigned types.
    You might be required to adjust the value based on an explicit
    interpretation and cast it into these larger types.
   
`JSON`_ representation
------------------------------------------------------------------------------
 
A more interesting metadata tree might come from the
:cpp:class:`pdal::drivers::las::Reader`.  Interesting things to note include 
 


.. _`JSON`: http://www.json.org/

.. code-block:: javascript

    {
        "name": "drivers.las.reader",
        "type": "blank",
        "value": "",
        "metadata":
        {
            "compressed":
            {
                "name": "compressed",
                "description": "true if this LAS file is compressed",
                "type": "boolean",
                "value": "false"
            },
            "dataformatid":
            {
                "name": "dataformatid",
                "description": "The Point Format ID as specified in the LAS specification",
                "type": "nonNegativeInteger",
                "value": "3"
            },
            ...
            "project_id":
            {
                "name": "project_id",
                "description": "Project ID (GUID data): The four fields that comprise a complete Globally Unique Identifier (GUID) are now reserved for use as a Project Identifier (Project ID). The field remains optional. The time of assignment of the Project ID is at the discretion of processing software. The Project ID should be the same for all files that are associated with a unique project. By assigning a Project ID and using a File Source ID (defined above) every file within a project and every point within a file can be uniquely identified, globally.",
                "type": "uuid",
                "value": "00000000-0000-0000-0000-000000000000"
            },
            "system_id":
            {
                "name": "system_id",
                "description": "",
                "type": "string",
                "value": "HOBU-SYSTEMID"
            },
            ...
            "vlr_0":
            {
                "name": "vlr_0",
                "description": "A Polygon WKT entry",
                "type": "base64Binary",
                "value": "UE9MWUdPTigoNiAxNSwgMTAgMTAsIDIwIDEwLCAyNSAxNSwgMjUgMzUsIDE5IDQwLCAxMSA0MCwgNiAyNSwgNiAxNSkpCg==",
                "metadata":
                {
                    "reserved":
                    {
                        "name": "reserved",
                        "description": "Two bytes of padded, unused space. Some softwares expect the values of these bytes to be 0xAABB as specified in the 1.0 version of the LAS specification",
                        "type": "nonNegativeInteger",
                        "value": "43707"
                    },
                    "user_id":
                    {
                        "name": "user_id",
                        "description": "The User ID field is ASCII character data that identifies the user which created the variable length record. It is possible to have many Variable Length Records from different sources with different User IDs. If the character data is less than 16 characters, the remaining data must be null. The User ID must be registered with the LAS specification managing body. The management of these User IDs ensures that no two individuals accidentally use the same User ID. The specification will initially use two IDs: one for globally specified records (LASF_Spec), and another for projection types (LASF_Projection). Keys may be requested at http:\/\/www.asprs.org\/lasform\/keyform.html.",
                        "type": "string",
                        "value": "hobu"
                    },
                    "record_id":
                    {
                        "name": "record_id",
                        "description": "The Record ID is dependent upon the User ID. There can be 0 to 65535 Record IDs for every User ID. The LAS specification manages its own Record IDs (User IDs owned by the specification), otherwise Record IDs will be managed by the owner of the given User ID. Thus each User ID is allowed to assign 0 to 65535 Record IDs in any manner they desire. Publicizing the meaning of a given Record ID is left to the owner of the given User ID. Unknown User ID\/Record ID combinations should be ignored.",
                        "type": "nonNegativeInteger",
                        "value": "1234"
                    },
                    "description":
                    {
                        "name": "description",
                        "description": "",
                        "type": "string",
                        "value": "A Polygon WKT entry"
                    }
                }
            },
            ...
        }
    }
    
.. _metadataxml:

:ref:`Pipeline` XML representation
------------------------------------------------------------------------------

The :ref:`Pipeline` representation of the :cpp:class:`pdal::Metadata` is a 
little bit flatter... 


::

    <?xml version="1.0" encoding="utf-8"?>
    <Reader type="drivers.las.reader">
      <Option name="debug">false</Option>
      <Option name="filename">test/data/interesting.las</Option>
      <Option name="verbose">0</Option>
      <Metadata name="drivers.las.reader" type="blank">
        <Metadata name="compressed" type="boolean">false</Metadata>
        <Metadata name="dataformatid" type="nonNegativeInteger">3</Metadata>
        <Metadata name="version_major" type="nonNegativeInteger">1</Metadata>
        <Metadata name="version_minor" type="nonNegativeInteger">2</Metadata>
        <Metadata name="filesource_id" type="nonNegativeInteger">0</Metadata>
        <Metadata name="reserved" type="nonNegativeInteger">0</Metadata>
        <Metadata name="project_id" type="uuid">00000000-0000-0000-0000-000000000000</Metadata>
        <Metadata name="system_id" type="string">HOBU-SYSTEMID</Metadata>
        <Metadata name="software_id" type="string">HOBU-GENERATING</Metadata>
        <Metadata name="creation_doy" type="nonNegativeInteger">145</Metadata>
        <Metadata name="creation_year" type="nonNegativeInteger">2012</Metadata>
        <Metadata name="header_size" type="nonNegativeInteger">227</Metadata>
        <Metadata name="dataoffset" type="nonNegativeInteger">1488</Metadata>
        <Metadata name="scale_x" type="double">0.01</Metadata>
        <Metadata name="scale_y" type="double">0.01</Metadata>
        <Metadata name="scale_z" type="double">0.01</Metadata>
        <Metadata name="offset_x" type="double">-0</Metadata>
        <Metadata name="offset_y" type="double">-0</Metadata>
        <Metadata name="offset_z" type="double">-0</Metadata>
        <Metadata name="minx" type="double">635619.85</Metadata>
        <Metadata name="miny" type="double">848899.7000000001</Metadata>
        <Metadata name="minz" type="double">406.59</Metadata>
        <Metadata name="maxx" type="double">638982.55</Metadata>
        <Metadata name="maxy" type="double">853535.4300000001</Metadata>
        <Metadata name="maxz" type="double">586.38</Metadata>
        <Metadata name="count" type="nonNegativeInteger">1065</Metadata>
        <Metadata name="vlr_0" type="base64Binary">UE9MWUdPTigoNiAxNSwgMTAgMTAsIDIwIDEwLCAyNSAxNSwgMjUgMzUsIDE5IDQwLCAxMSA0MCwgNiAyNSwgNiAxNSkpCg==
            <Metadata name="reserved" type="nonNegativeInteger">43707</Metadata>
            <Metadata name="user_id" type="string">hobu</Metadata>
            <Metadata name="record_id" type="nonNegativeInteger">1234</Metadata>
            <Metadata name="description" type="string">A Polygon WKT entry</Metadata>
        </Metadata>
        <Metadata name="vlr_1" type="base64Binary">AQABAAAAFQAABAAAAQABAAEEAAABAAEAAgSxhywAAAAACAAAAQD/fwEIsYdqACwAAggAAAEA/38GCAAAAQCOIwgIAAABAP9/CQiwhwEABgALCLCHAQAHAA0IsIcBAAgAAAwAAAEA/38CDAAAAQD/fwMMAAABAAgABAwAAAEAKiMGDLCHAQACAAcMsIcBAAMADAywhwEAAQANDLCHAQAAAA4MsIcBAAQADwywhwEABQAAAAAAAAAAAA==
            <Metadata name="reserved" type="nonNegativeInteger">43707</Metadata>
            <Metadata name="user_id" type="string">LASF_Projection</Metadata>
            <Metadata name="record_id" type="nonNegativeInteger">34735</Metadata>
            <Metadata name="description" type="string">GeoTIFF GeoKeyDirectoryTag</Metadata>
        </Metadata>
        <Metadata name="vlr_2" type="base64Binary">AAAAAADgREAAAAAAACBewAAAAAAAgEVAAAAAAADARkD//////2kYQQAAAAAAAAAAAAAAQKZUWEGo+euUHaRyQAAAAAAAAAAA
            <Metadata name="reserved" type="nonNegativeInteger">43707</Metadata>
            <Metadata name="user_id" type="string">LASF_Projection</Metadata>
            <Metadata name="record_id" type="nonNegativeInteger">34736</Metadata>
            <Metadata name="description" type="string">GeoTIFF GeoDoubleParamsTag</Metadata>
        </Metadata>
        <Metadata name="vlr_3" type="base64Binary">TkFEXzE5ODNfT3JlZ29uX1N0YXRld2lkZV9MYW1iZXJ0X0ZlZXRfSW50bHxHQ1MgTmFtZSA9IEdDU19Ob3J0aF9BbWVyaWNhbl8xOTgzfERhdHVtID0gRF9Ob3J0aF9BbWVyaWNhbl8xOTgzfEVsbGlwc29pZCA9IEdSU18xOTgwfFByaW1lbSA9IEdyZWVud2ljaHx8AA==
            <Metadata name="reserved" type="nonNegativeInteger">43707</Metadata>
            <Metadata name="user_id" type="string">LASF_Projection</Metadata>
            <Metadata name="record_id" type="nonNegativeInteger">34737</Metadata>
            <Metadata name="description" type="string">GeoTIFF GeoAsciiParamsTag</Metadata>
        </Metadata>
        <Metadata name="vlr_4" type="base64Binary">UFJPSkNTWyJOQURfMTk4M19PcmVnb25fU3RhdGV3aWRlX0xhbWJlcnRfRmVldF9JbnRsIixHRU9HQ1NbIkdDU19Ob3J0aF9BbWVyaWNhbl8xOTgzIixEQVRVTVsiRF9Ob3J0aF9BbWVyaWNhbl8xOTgzIixTUEhFUk9JRFsiR1JTXzE5ODAiLDYzNzgxMzcuMCwyOTguMjU3MjIyMTAxXV0sUFJJTUVNWyJHcmVlbndpY2giLDAuMF0sVU5JVFsiRGVncmVlIiwwLjAxNzQ1MzI5MjUxOTk0MzI5NV1dLFBST0pFQ1RJT05bIkxhbWJlcnRfQ29uZm9ybWFsX0NvbmljXzJTUCJdLFBBUkFNRVRFUlsiRmFsc2VfRWFzdGluZyIsMTMxMjMzNS45NTgwMDUyNDldLFBBUkFNRVRFUlsiRmFsc2VfTm9ydGhpbmciLDAuMF0sUEFSQU1FVEVSWyJDZW50cmFsX01lcmlkaWFuIiwtMTIwLjVdLFBBUkFNRVRFUlsiU3RhbmRhcmRfUGFyYWxsZWxfMSIsNDMuMF0sUEFSQU1FVEVSWyJTdGFuZGFyZF9QYXJhbGxlbF8yIiw0NS41XSxQQVJBTUVURVJbIkxhdGl0dWRlX09mX09yaWdpbiIsNDEuNzVdLFVOSVRbIkZvb3QiLDAuMzA0OF1dAA==
            <Metadata name="reserved" type="nonNegativeInteger">43707</Metadata>
            <Metadata name="user_id" type="string">liblas</Metadata>
            <Metadata name="record_id" type="nonNegativeInteger">2112</Metadata>
            <Metadata name="description" type="string">OGR variant of OpenGIS WKT SRS</Metadata>
        </Metadata>
      </Metadata>
    </Reader>

.. _`boost::property_tree`: http://www.boost.org/doc/libs/release/libs/property_tree/
