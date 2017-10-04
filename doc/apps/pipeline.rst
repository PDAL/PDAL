.. _pipeline_command:

********************************************************************************
pipeline
********************************************************************************

The ``pipeline`` command is used to execute :ref:`pipeline` JSON. See
:ref:`reading` or :ref:`pipeline` for more information.

::

    $ pdal pipeline <input>

::

  --input, -i               Input filename
  --pipeline-serialization  Output file for pipeline serialization
  --validate                Validate the pipeline (including serialization),
      but do not write points
  --progress                Name of file or FIFO to which stages should write
      progress information. The file/FIFO must exist. PDAL will not create the
      progress file.
  --stdin, -s               Read pipeline from standard input
  --stream                  Attempt to run pipeline in streaming mode.
  --metadata                Metadata filename


Substitutions
................................................................................

The ``pipeline`` command can accept command-line option substitutions and
they replace
existing options that are specified in the input JSON pipeline.  If
multiple stages of the same name exist in the pipeline, `all` stages would
be overridden. For example, to set the output and input LAS files for a
pipeline that does a translation, the ``filename`` for the reader and the
writer can be overridden:

::

    $ pdal pipeline translate.json --writers.las.filename=output.laz \
        --readers.las.filename=input.las

Option substitution can also refer to the tag of an individual stage.
This can be done by using the syntax --stage.<tagname>.<option>.  This
allows options to be set on individual stages, even if there are multiple
stages of the same type.  For example, if a pipeline contained two LAS
readers with tags ``las1`` and ``las2`` respectively, the following
command would allow assignment of different filenames to each stage:

::

    {
        "pipeline" : [
            {
                "tag" : "las1",
                "type" : "readers.las"
            },
            {
                "tag" : "las2",
                "type" : "readers.las"
            },
            "placeholder.laz"
        ]
    }

    $ pdal pipeline translate.json --writers.las.filename=output.laz \
        --stage.las1.filename=file1.las --stage.las2.filename=file2.las

Options specified by tag names override options specified by stage types.
