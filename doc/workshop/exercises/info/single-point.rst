.. _workshop-single-point:

Printing the first point
================================================================================

.. include:: ../../includes/substitutions.rst

.. index:: info command, Start Here, docker run

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to print information from the first point. Issue the
following command in your `Docker Quickstart Terminal`.

.. literalinclude:: ./single-point-command.txt
    :linenos:

Here's a summary of what's going on with that command invocation

1. ``docker``: We are running PDAL within the context of docker, so all of our
   commands will start with the ``docker`` command.

2. ``run``: Tells docker we're going to run an image

3. ``-v /c/Users/Howard/PDAL:/data``: Maps our workshop directory to a directory called
   ``/data`` inside the container.

   .. seealso::

       The `Docker Volume <https://docs.docker.com/engine/userguide/dockervolumes/>`__
       document describes mounting volumes in more detail.

4. ``pdal/pdal``: This is the Docker image we are going to run. We fetched it in the
   :ref:`docker` portion of the workshop.

5. ``pdal``: We're finally going to run the ``pdal`` application :)

6. ``info``: We want to run :ref:`info_command` on the data. All commands
   are run by the ``pdal`` application.

7. ``/data/exercises/info/interesting.las``: The ``pdal`` command is now
   running in the context of our container, which we mounted a ``/data``
   directory in with the volume mount operation in Step #3. Our
   ``interesting.las`` file resides there.

8. ``-p 0``: ``-p`` corresponds to "print a point", and ``0`` means to print
   the first one (computer people count from 0).


.. image:: ../../images/info-interesting-single-point.png

Notes
--------------------------------------------------------------------------------

.. index:: JSON

1. PDAL uses `JSON`_ as the exchange format when printing information from :ref:`info_command`.
   JSON is a structured, human-readable format that is much simpler than its `XML`_ cousin.

.. index:: CSV

2. You can use the :ref:`writers.text` writer to output point attributes to `CSV`_ format for
   other processing.

3. Output help information on the command line by issuing the ``--help`` option

4. A common query with ``pdal info`` is ``--all``, which will print all header,
   metadata, and statistics about a file.

5. In the command, we add the ``\`` character for line continuation. All items on the
   ``docker run`` command must be on the same line. You will see this convention
   throughout the workshop to make the command easier to read, but remember
   that everything needs to be on one line.

.. _`CSV`: https://en.wikipedia.org/wiki/Comma-separated_values
.. _`JSON`: https://en.wikipedia.org/wiki/JSON
.. _`XML`: https://en.wikipedia.org/wiki/XML
