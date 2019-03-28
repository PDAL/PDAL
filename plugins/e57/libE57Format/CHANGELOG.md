libE57Format
==
- v2.0.1 (15 Jan 2019)
  - writing files was broken and would produce the following error:
    Error: bad API function argument provided by user (E57_ERROR_BAD_API_ARGUMENT) (ImageFileImpl.cpp line 109)
    
- v2.0 (06 Jan 2019)
  - forked from E57RefImpl
  - removed all but the main sources for reading and writing E57 files
  - now requires C++11
  - now requires cmake 3.1+
  - no longer uses BOOST
  - multiple fixes for compilation on macOS
  - fix a couple of fallthrough bugs which would result in undefined behaviour
  - turn off `E57_MAX_DEBUG` by default
  - improve file read times
  - add a checksum policy (see _ReadChecksumPolicy_ in *E57Format.h*) so the library user can decide how frequently to check them
  - remove "big endian" byte swap code (not sure it was working and no way to test)
  - lots and lots of code cleanups
    - refactored the code into multiple files
    - removed unused macros and code
    - remove non-useful comments
    - add proper initialization of class and struct members
    - modernize using c++11
  - [Windows] add cmake option ()`USING_STATIC_XERCES`) to tell the build if you are using a static Xerces lib

E57RefImpl
==

- 2013-04-03 roland_schwarz
  - e57unpack now also can handle 2d images
- 2013-01-30 roland_schwarz
  - added missing library identifier string to cmake build
- 2011-10-04 roland_schwarz
  - update to use boost version 1.46.1
  - streamlined cmake files for better static linking
- 2011-03-14 roland_schwarz
  - E57RefImplConfig.cmake added
  - e57validate tool added to cmake
- 2011-03-10: roland_schwarz
  - Added E57RefImplConfig.cmake for user project configuration.
  - The build instructions can be found inside the CMakeLists.txt file.
- 2010-10-16 roland_schwarz
  - e57unpack tool added
  - riegl_1 example files added
