.. _filters.julia:

filters.julia
==============

The **Julia Filter** allows `Julia`_ software to be embedded in a
:ref:`pipeline` that allows modification of PDAL points through a `TypedTables`_
datatype.

The supplied julia function must take a `TypedTables`_ FlexTable as an argument
and return the same object (with modifications).

.. warning::

    The returned Table contains all the :ref:`dimensions` of the incoming ``ins`` Table

.. plugin::

.. code-block:: julia

  module MyModule
    using TypedTables

    function multiply_z(ins)
      for n in 1:length(ins)
        ins[n] = merge(ins[n], (; :Z => row.Z * 10.0)
      end
      return ins
    end
  end


 If you want write a dimension that might not be available, you can specify
 it with the add_dimension_ option:

   ::

       "add_dimension": "NewDimensionOne"

 To create more than one dimension, this option also accepts an array:

   ::

       "add_dimension": [ "NewDimensionOne", "NewDimensionTwo", "NewDimensionThree" ]


 You can also specify the :ref:`type <types>` of the dimension using an ``=``.
   ::

       "add_dimension": "NewDimensionOne=uint8"


Filter Example
--------------------------------------------------------------------------------

.. code-block:: json

  [
      "file-input.las",
      {
          "type":"filters.smrf"
      },
      {
          "type":"filters.julia",
          "script":"filter_z.jl",
          "function":"filter_z",
          "module":"MyModule"
      },
      {
          "type":"writers.las",
          "filename":"file-filtered.las"
      }
  ]

The JSON pipeline file referenced the external `filter_z.jl` `Julia`_ script,
which removes points with the ``Z`` coordinate by less than 420.

.. code-block:: julia

  module MyModule
    using TypedTables

    function filter_z(ins)
      return filter(p -> p.Z > 420, ins)
    end
  end

Modification Example
--------------------------------------------------------------------------------

.. code-block:: json

  [
      "file-input.las",
      {
          "type":"filters.smrf"
      },
      {
          "type":"filters.julia",
          "script":"multiply_z.jl",
          "function":"multiply_z",
          "module":"MyModule"
      },
      {
          "type":"writers.las",
          "filename":"file-modified.las"
      }
  ]

The JSON pipeline file referenced the external `multiply_z.jl` `Julia`_ script,
which scales the ``Z`` coordinate by a factor of 10.

.. code-block:: julia

  module MyModule
    using TypedTables

    function multiply_z(ins)
      for n in 1:length(ins)
        ins[n] = merge(ins[n], (; :Z => row.Z * 10.0)
      end
      return ins
    end
  end

Options
--------------------------------------------------------------------------------

script
  When reading a function from a separate `Julia`_ file, the file name to read
  from.

source
  The literal `Julia`_ code to execute, when the script option is
  not being used.

module
  The Julia module that is holding the function to run. [Required]

function
  The function to call. [Required]

_`add_dimension`
  A dimension name or an array of dimension names to add to the pipeline that do not already exist.

.. include:: filter_opts.rst

.. _Julia: https://julialang.org/
.. _TypedTables: https://github.com/JuliaData/TypedTables.jl
