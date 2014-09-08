.. _development_errorhandling:

=========================
Errors and Error Handling
=========================

Exceptions
==========

* throw exceptions, don't use status codes

* use the pdal_error hiearchy (list)


PDAL Position on (Non)conformance
==================================

PDAL proudly and unabashedly supports formal standards/specifications for
file formats.  We recognize, however, that in some cases files will not
follow a given standard precisely, due to an unclear spec or simply out of
carelessness.

When reading files that are not formatted correctly:

* PDAL may try to compensate for the error.  This is typically done when as 
  a practical matter the market needs support for well-known or pervasive,
  but nonetheless "broken", upstream implementations.

* PDAL may explicitly reject such files.  This is typically done where we do 
  not wish to continue to promote or support mistakes that should be fixed 
  upstream.

PDAL will always write correctly formatted files.  In some cases, however,
PDAL may choose to offer as an option the ability to break the standard if,
as a practical matter, doing so would significantly aid the market.  Such an
option would never be the default behavior, however.

For files that are conformant but which lie, such as the extents in the
header being wrong, we will generally offer both the ability to propagate 
the "wrong" information and the ability to helpfully correct it on the fly;
the latter is generally our default position.
