README for PDAL C# Support
===========================

 *** UNDER DEVELOPMENT ***  (proceed with caution)

This directory contains the code needed to create C# bindings and the pcview
app for PDAL.

If questions, contact mpg@flaxen.com.

The C# work does not use CMake, as CMake doesn't yet support C# builds.

* You need to set these environment variables:
  - set PDAL_SWIG_BOOST_HOME=C:\Utils\boost_1_45_0-win32

* To build PCVeiw, you will need the DLLs for SlimDX and SlimDXControl.

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
	. creates swig_test.exe


* The two DLLs and the EXE are put into the main pdal bin directory, for ease of use.


* Only Visual Studio 2010 is supported.  Four build configurations are possible:
  - x86/Debug
  - x86/Release
  - x64/Debug
  - x64/Release
Of these four, only x64/Debug has actually been tested :-)

There seems to be an issue with running swig-generated code in x86 mode on a
64-bit OS, and the examples that ship with swig bear this out.  I'm not sure
what's up with that.  See this thread:
  http://old.nabble.com/SWIG-examples-will-not-run-on-my-windows-7---64-bits-td29085404.html


* The test app should be run from the csharp/pdal-swig-test directory.
