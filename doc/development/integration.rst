.. _integration:

================================================================================
Continuous Integration
================================================================================

PDAL :ref:`regression tests <pdal_test>` are run on a per-commit basis by at
least two continuous integration platforms.

Status
--------------------------------------------------------------------------------

|travisstatus|
|appveyorstatus|

.. |travisstatus| image:: https://travis-ci.org/PDAL/PDAL.png?branch=master
   :target: https://travis-ci.org/PDAL/PDAL

.. |appveyorstatus| image:: https://ci.appveyor.com/api/projects/status/6dehrm0v22cw58d3/branch/master?svg=true
   :target: https://ci.appveyor.com/project/hobu/pdal

.. _travis:

Travis
--------------------------------------------------------------------------------

The Travis continuous integration platform runs the PDAL test suite on Alpine
Linux. The build status and other supporting information can be found at
https://travis-ci.org/PDAL/PDAL Its configuration can be found at
https://github.com/PDAL/PDAL/blob/master/.travis.yml All administrators of the
GitHub `PDAL` group have rights to modify the Travis configuration.

It uses the ``alpine:edge`` Docker image found at
https://hub.docker.com/_/alpine/ as a base platform. If you want to add new
functionality based on a dependency, you will need to ensure that the dependency
is available in https://pkgs.alpinelinux.org/packages and update the Travis
configuration YAML accordingly.

.. _appveyor:

AppVeyor
--------------------------------------------------------------------------------

PDAL uses the AppVeyor continuous integration platform to run the PDAL
compilation and test suite on Windows. The build status and other supporting
information can be found at https://ci.appveyor.com/project/hobu/pdal Its
configuration can be found at
https://github.com/PDAL/PDAL/blob/master/appveyor.yml All administrators of the
GitHub `PDAL` group have rights to modify the AppVeyor configuration.

`Howard Butler`_ currently pays the bill to run in the AppVeyor upper
performance processing tier. The AppVeyor configuration depends on `OSGeo4W64`_
for dependencies. If you want to add new test functionality based on a
dependency, you will need to update OSGeo4W64 with a new package to do so.

.. _`OSGeo4W64`: http://trac.osgeo.org/osgeo4w/
.. _`Howard Butler`: http://github.com/hobu
