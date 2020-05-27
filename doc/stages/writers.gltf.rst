.. _writers.gltf:

writers.gltf
============

GLTF is a file format `specification`_ for 3D graphics data.
If a mesh has been generated
for a PDAL point view, the **GLTF Writer** will produce simple output in
the GLTF format.  PDAL does not currently support many of the attributes
that can be found in a GLTF file.  This writer creates a *binary* GLTF.

.. _specification: https://www.khronos.org/gltf/

.. embed::

Example
-------

.. code-block:: json

  [
      "infile.las",
      {
          "type": "filters.poisson",
          "depth": 12
      },
      {
          "type":"writers.gltf",
          "filename":"output.glb",
          "red": 0.8,
          "metallic": 0.5
      }
  ]

Options
-------

filename
    Name of the GLTF (.glb) file to be written. [Required]

metallic
    The metallic factor of the faces. [Default: 0]
    
roughness
    The roughness factor of the faces. [Default: 0]
    
red
    The red component of the color applied to the faces. [Default: 0]
    
green
    The green component of the color applied to the faces. [Default: 0]
    
blue
    The blue component of the color applied to the faces. [Default: 0]
    
alpha
    The alpha component to be applied to the faces. [Default: 1.0]

double_sided
    Whether the faces are colored on both sides, or just the side
    visible from the initial observation point (positive normal vector).
    [Default: false]

