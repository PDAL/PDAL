.. _integration:

================================================================================
Continuous Integration
================================================================================

PDAL :ref:`regression tests <pdal_test>` are run on a per-commit basis using
`GitHub Actions`_

.. _`GitHub Actions`: https://github.com/features/actions

Status
--------------------------------------------------------------------------------

|linuxstatus|
|osxstatus|
|windowstatus|
|docsstatus|



.. |linuxstatus| image:: https://github.com/PDAL/PDAL/workflows/Linux/badge.svg
   :target: https://github.com/PDAL/PDAL/actions?query=workflow%3ALinux

.. |osxstatus| image:: https://github.com/PDAL/PDAL/workflows/OSX/badge.svg
   :target: https://github.com/PDAL/PDAL/actions?query=workflow%3AOSX

.. |windowstatus| image:: https://github.com/PDAL/PDAL/workflows/Windows/badge.svg
   :target: https://github.com/PDAL/PDAL/actions?query=workflow%3AWindows

.. |docsstatus| image:: https://github.com/PDAL/PDAL/workflows/Docs/badge.svg
   :target: https://github.com/PDAL/PDAL/actions?query=workflow%3ADocs

Configuration
--------------------------------------------------------------------------------

Continuous integration configuration is modified by manipulating configuration
files in to locations:

* ``./github/workflows``
* ``./scripts/ci``

Linux, OSX, and Windows builds are all configured separately with scripts in the
``./scripts/ci`` directory.

Dependencies
--------------------------------------------------------------------------------

All of the tests use Conda Forge for dependencies.

The Linux builder has a "fixed"
configuration that pins GDAL to a specific version to prevent the rest of the
dependency tree from floating according to Conda Forge's package dependency rules.

Docs
--------------------------------------------------------------------------------

Docs are always built and doc artifacts are attached to the build:

* HTML
* PDF
* Misspelled words

Push to pdal.io
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Docs are pushed to pdal.io under the following conditions:

* Doc building succeeds
* The push branch denoted in ``./github/workflows/docs.yaml`` matches the current
  ``*-maintenance`` branch.

