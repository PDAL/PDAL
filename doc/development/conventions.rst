.. _development_conventions:

==================
Coding Conventions
==================

To the extent possible and reasonable, we value consistency of source code
formatting, class and variable naming, and so forth.

This Note lists some such conventions that we would like to follow, where
it makes sense to do so.


Source Formatting
=================

We use astyle (http://astyle.sourceforge.net) as a tool to reformat C++
source code files in a consistent fashion.  The file astylerc, at the top
of the github repo, contains the default settings we use.

Our conventions are:

* LF endings (unix style), not CRLF (windows style)

* spaces, not tabs

* indent to four (4) spaces ("Four shalt be the number thou shalt count, 
  and the number of the counting shall be four. Three shalt thou not count, 
  neither count thou five...")

* braces shall be on their own lines, like this::

    if (p)
    {
       foo();
    }

* copyright header on every file 

* two spaces between major units, e.g. function bodies


Naming Conventions
==================

* classes should be names using UpperCamelCase

* functions should be in lowerCamelCase

* member variables should be prefixed with "m\_", followed by the name in
  lowerCamelCase -- for example, "m_numberOfPoints"

* there should be only one class per file, and the name of the file should 
  match the class name -- that is, class PointData should live in files
  PointData.hpp and PointData.cpp


Other Conventions
=================

* the use of getter and setter methods is preferred to exposing member
  variables
  
* Surround all code with "namespace pdal {...}"; where justifiable, you 
  may introduce a nested namespace.
  
* Use exceptions for exceptional events that are not going to be handled 
  directly within the context of where the event occurs.  Avoid status
  codes.  See exceptions.hpp for a set of pdal-specific exception types 
  you may throw.  (See Note on error handling)

* Describe use of "debug" and "verbose" settings.

* Don't put member function bodies in the class declaration in the
  header file, unless clearly justified for performance reasons.
  Use the "inline" keyword in these cases(?).
  
* Use const.

* Don't put "using" declarations in headers.

* Document all public (and protected) member functions using
  doxygen markup.

Layout/Organization of Source Tree
==================================

* public headers in ./include
    
* private headers alongside source files in src/

* ...


#include Conventions
====================

* For public headers from the ./include/pdal directory, use angle brackets:
  #include <pdal/Stage.h>

* For private headers (from somehwere in ./src), use quotes:
  #include "support.hpp"

* #include lines should be grouped and arranged in this order: C++/std headers,
  3rd-party headers (e.g. gdal or boost), pdal headers, local headers.  The
  pdal headers may be further grouped by subdirectory, e.g. drivers/liblas,
  filters, etc.

* Exception to the above: source files (.cpp) should #include their
  corresponding .hpp file first.  This assures that the header is including
  all the files it needs to.
  
* Don't #include a file where a simple forward declaration will do.
  (Note: this only applies to pdal files; don't forward declare from system
  or 3rd party headers.)

* Don't include a file unless it actually is required to compile the source unit.

* Include guards should spell out the full path, like this:
  #ifndef INCLUDED_DRIVERS_FAUX_READER_ITERATOR_HPP
 
