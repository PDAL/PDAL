.. _apps:

******************************************************************************
Applications
******************************************************************************

PDAL contains consists of a single application, called ``pdal``.  Applications
are run by invoking the ``pdal`` application along with the command name:

::

    $ pdal info myfile.las
    $ pdal translate input.las output.las
    $ pdal pipeline --stdin < myxml.xml

Help for each command can be retrieved via the ``--help`` switch. The
``--drivers`` and ``--options`` switches can tell you more about particular
drivers and their options:

::

    $ pdal info --help
    $ pdal translate --drivers
    $ pdal pipeline --options writers.las

Additional driver-specific options may be specified by using a
namespace-prefixed option name. For example, it is possible to set the LAS day
of year at translation time with the following option:

::

    $ pdal translate \
        --writers.las.creation_doy="42" \
        input.las \
        output.las

.. note::

    Driver specific options can be identified using the ``pdal info --options``
    invocation.

.. toctree::
   :maxdepth: 2
   :glob:

   *
