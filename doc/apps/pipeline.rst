.. _pipeline_command:

********************************************************************************
pipeline
********************************************************************************

The ``pipeline`` command is used to execute :ref:`pipeline` JSON. See
:ref:`reading` or :ref:`pipeline` for more information.

::

    $ pdal pipeline <input>

::

    --input [-i] arg  Non-positional argument to specify input file name.
    --pipeline-serialization arg
                      Write input pipeline along with all metadata and created by the
                      pipeline to the specified file.
    --validate        Validate the pipeline (including serialization), but do not execute
                      writing of points

.. note::

    The ``pipeline`` command can accept option substitutions, but they
    do not replace existing options that are specified in the input JSON
    pipeline.  For example, to set the output and input LAS files for a
    pipeline that does a translation, construct JSON that does not contain
    ``filename`` for reader and writer and issue the command with the
    following arguments:

    ::

        $ pdal pipeline translate.json --writers.las.filename=output.laz \
            --readers.las.filename=input.las


