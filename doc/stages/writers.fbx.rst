.. _writers.fbx:

writers.fbx
===========

Output to the Autodesk FBX format. The pipeline requires the mesh 
to be built, as only mesh output is created. You must us 
:ref:`filters.poisson` or `filters.greedyprojection` to compute 
a mesh for the point cloud before outputting the FBX.

.. plugin::

Compilation
-------------

	  
You must download and install the Autodesk SDK 
and then compile the PDAL FBX plugin against it. Visit 
https://www.autodesk.com/developer-network/platform-technologies/fbx-sdk-2019-0 
to obtain a current copy of the SDK.  

Example Windows CMake configuration

::
  	  -DFBX_ROOT_DIR:FILEPATH="C:\fbx\2019.0" ^
	  -DBUILD_PLUGIN_FBX=ON ^





Example
-------

.. code-block:: json

  [
      {
          "type":"readers.las",
          "filename":"inputfile.las"
      },
      {
          "type":"filters.poisson"
      },
      {
          "type":"writers.fbox",
          "filename":"outputfile.fbx"
      }
  ]

..code-block:: shell

	pdal translate autzen.las autzen.fbx -f poisson

Options
-------

filename
    FBX filename to write
    [Required]

ascii
    Write ASCII FBX format.  [Default: false]
