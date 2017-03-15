.. _diff_command:

********************************************************************************
diff
********************************************************************************

The ``diff`` command is used for executing a simple contextual difference
between two sources.

::

    $ pdal diff <source> <candidate>

::

  --source       source file name
  --candidate    candidate file name
  --output       output file name
  --2d           only 2D comparisons/indexing
  --detail       Output deltas per-point
  --alldims      Compute diffs for all dimensions (not just X,Y,Z)

The command returns 0 and produces no output if the files describe the same
point data in the same format, otherwise 1 is returned and a JSON-formatted
description of the differences is produced.

The command checks for the equivalence of the following items:

* Different schema
* Expected count
* Metadata
* Actual point count
* Byte-by-byte point data


