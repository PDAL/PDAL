.. _readers.obj:

readers.obj
===============

The **OBJ reader** reads data from files in the OBJ format.
This reader constructs a mesh from the faces specified in the OBJ file, ignoring
vertices that are not associated with any face. Faces, vertices, vertex normals and vertex
textures are read, while all other obj elements (such as lines and curves) are ignored.

.. plugin::

Example
-------
This pipeline reads from an example OBJ file outputs
the vertices as a point to a LAS file.

.. code-block:: json

    [
        {
            "type": "readers.obj",
            "filename": "test/data/obj/1.2-with-color.obj"
        },
        {
            "type" : "writers.las",
            "filename": "output.las",
            "scale_x": 1.0e-5,
            "scale_y": 1.0e-5,
            "scale_z": 1.0e-5,
            "offset_x": "auto",
            "offset_y": "auto",
            "offset_z": "auto"
        }
    ]


Options
-------

.. include:: reader_opts.rst

filename
  File to read. [Required]