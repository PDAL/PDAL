.. _metadata:

******************************************************************************
Metadata
******************************************************************************

In addition to point data, PDAL stores metadata during the processing of
a pipeline.  Metadata is stored internally as strings, though the API
accepts a variety of types that are automatically converted as necessary.
Each item of metadata consists of a name, a description (optional), a value
and a type.  In addition, each item of metadata can have a list of child
metadata values.

Metadata is made available to users of PDAL through a JSON tree.  Commands
such as :ref:`pdal pipeline <pipeline_command>` and
:ref:`pdal translate <translate_command>` provide options to allow
the JSON-formatted metadata created by PDAL to be written to a file.

Metadata Nodes
------------------------------------------------------------------------------

Each item of metadata is stored in an object known as a MetadataNode.
Metadata nodes are reference types that can be copied cheaply.  Metadata nodes
are annotated with the original data type to allow better interpretation of
the data.
For example, when binary data is stored in a base 64-encoded
format, knowing that the data doesn't ulitmately represent a string can allow
algorithms to convert it back to its binary representation when desired.
Similarly, knowing that data is numeric allows it
to be written as a JSON numeric type rather than as a string.

The name of a metadata node is immutable.  If you wish to add a copy of
metadata (and subchildren) to some node using a different name, you need
to call the provided function "clone()".

A metadata node is added as a child to another node using add().  Usually
the type of the data assigned to the metadata node is determined through
overloading, but there are instances where this is impossible and the
programmer must call a specific function to set the type of the metadata node.
Binary data that has been converted to a string by base 64 encoding can
be tagged as a such by calling addEncoded().  Programmers can specify the
type of a node explictly by calling addWithType().  Currently supported
types are: "boolean", "string", "float", "double", "bounds",
"nonNegativeInteger", "integer", "uuid" and "base64Binary".

Metadata nodes can be presented as lists when transformed to JSON.  If
multiple nodes with the same name are added to a parent node, those
subnodes will automatically be tagged as list nodes and will be enclosed in
square brackets.  Single nodes can be forced to be treated as JSON lists
by calling addList() instead of add() on a parent node.


Metadata and Stages
------------------------------------------------------------------------------

Stages in PDAL each have a base metadata node.  You can retrieve a stage's
metadata node by calling getMetadata().  When a PDAL pipeline is run, its
metadata is organized as a list of stage nodes to which subnodes have been
added.  From within the implementation of a stage, metadata is typically
added similarly to the following:

.. code-block:: c++

    MetadataNode root = getMetadata();
    root.add("nodename", "Some string data");
    root.add("intlist", 45);
    root.add("intlist", 55);
    Uuid nullUuid;
    MetadataNode pnode("parent");
    root.add(pnode);
    pnode.add("nulluuidnode", nullUuid);
    pnode.addList("num_in_list", 66);

If the above code was part of a stage "writers.test", a transformation to JSON
would produce the following output:

.. code-block:: json

    {
      "writers.test":
      {
        "intlist":
        [
          45,
          55
        ],
        "nodename": "Some string data",
        "parent":
        {
          "nulluuidnode": "00000000-0000-0000-0000-000000000000",
          "num_in_list":
          [
            66
          ]
        }
      }
    }

