README for PDAL C# Support
===========================

 *** UNDER DEVELOPMENT ***  (proceed with caution)

This directory contains the code needed to create C# bindings and the pcview
app for PDAL.

If questions, contact mpg@flaxen.com.

The C# work does not use CMake, as CMake doesn't yet support C# builds.

* For swig-2.0.1 (and possibly later), you need to modify one of SWIG's internal library files:
    in ...\swigwin-2.0.1\Lib\std\std+_basic_string.i, comment out line 48 ("%fragment...")
  I'm not sure why this is needed, but unless you do swig won't work.  Note this is not a PDAL
  problem per se, it's something about swig itself.
  
* You need to set these environment variables:
  - set PDAL_SWIG_BOOST_HOME=C:\Utils\boost_1_46_1

* (PCView, the SlimDX-based DirectX viewer, is not currently being supported.)

* The solution file pdal_swig.sln has three projects:
  - pdal_swig_cpp
    . runs swig on the .i file
      - creates the .cpp wrapper file
      - creates the .cs wrapper classes
    . compiles the .cpp wrapper file
    . creates pdal_swig_cpp.dll
  - pdal_swig_cs
    . compiled the .cs wrapper classes
    . creates pdal_swig_cs.dll
  - pdal_swig_test
    . compiles a simple test app to verify the bindings work
    . creates pdal_swig_test.exe


* The two DLLs and the EXE are put into the main pdal bin directory, for ease of use.


* Only Visual Studio 2010 is supported.  Four build configurations are possible:
  - x86/Debug
  - x86/Release
  - x64/Debug
  - x64/Release
Of these four, only x86/Debug has actually been tested :-)


* The test app should be run from the csharp/pdal_swig_test directory.
