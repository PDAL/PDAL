.. _development_conventions:

================================================================================
Coding Conventions
================================================================================

To the extent possible and reasonable, we value consistency of source code
formatting, class and variable naming, and so forth.  Please follow existing
code, rather than introducing your own (of course, better) formatting or
change existing code unless you're changing behavior.

This note lists some such conventions that we would like to follow, where
it makes sense to do so.

Source Formatting
=================

We use astyle (http://astyle.sourceforge.net) as a tool to reformat C++
source code files in a consistent fashion.  The file astylerc, at the top
of the github repo, contains the default settings we use.

Our conventions are:

* Lines should be kept to 80 characters where reasonable.

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

* copyright header, license, and author(s) on every file

* two spaces between major units, e.g. function bodies


Naming Conventions
==================

* classes should be names using UpperCamelCase

* functions should be in lowerCamelCase

* member variables should be prefixed with "m\_", followed by the name in
  lowerCamelCase -- for example, "m_numberOfPoints"

* there should be one class per file, and the name of the file should
  match the class name -- that is, class PointData should live in files
  PointData.hpp and PointData.cpp.


Other Conventions
=================

* Surround all code with "namespace pdal {...}"; where justifiable, you
  may introduce a nested namespace.

* All exceptions that are not caught internally should be of type pdal_error.
  Exceptions used as local error handling should always be caught.

* Don't put member function bodies in the class declaration in the
  header file, unless clearly justified for performance reasons.
  Use the "inline" keyword in these cases(?).

* Use const.

* Don't put "using" declarations in headers.

* Document all public (and protected) member functions using
  doxygen markup.

#include Conventions
====================

* For public headers from the ./include/pdal directory, use angle brackets:
  #include <pdal/Stage.h>

* For private headers (from somehwere in ./src), use quotes:
  #include "Support.hpp"

* Don't #include a file where a simple forward declaration will do.
  (Note: this only applies to pdal files; don't forward declare from system
  or 3rd party headers.)

* Don't include a file unless it actually is required to compile the source unit.

* Don't use manual include guards. All reasonable compilers support the once pragma::

  #pragma once

