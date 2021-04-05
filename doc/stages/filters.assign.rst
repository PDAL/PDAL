.. _filters.assign:

filters.assign
===================

The assign filter allows you set the value of a dimension for all points
to a provided value that pass a range filter.

.. embed::

.. streamable::

.. note::
    The `assignment` and `condition` options are deprecated and may be removed in a
    future release.

Options
-------

assignment
  A :ref:`range <ranges>` followed by an assignment of a value (see example).
  Can be specified multiple times.  The assignments are applied sequentially
  to the dimension value as set when the filter began processing. [Required]

condition
  A list of :ref:`ranges <ranges>` that a point's values must pass in order
  for the assignment to be performed. [Default: none]

value
  A list of :ref:`assignment expressions <Assignment Expressions>` to be applied to points.
  The list of values is evaluated in order. [Default: none]

.. include:: filter_opts.rst

.. _assignment expressions:

Assignment Expressions
======================

The assignment expression syntax is an expansion on the :ref:`PDAL expression` syntax
that provides for assignment of values to points. The generic expression is:

.. code-block::

    "value" : "Dimension = ValueExpression [WHERE ConditionalExpression)]"

``Dimension`` is the name of a PDAL dimension.

A ``ValueExpression`` consists of constants, dimension names and mathematical operators
that evaluates to a numeric value.  The supported mathematical operations are addition(`+`),
subtraction(`-`), multiplication(`*`) and division(`\\`).

A :ref:`ConditionalExpression <PDAL expression>` is an optional boolean value that must
evaluate to `true` for the ``ValueExpression`` to be applied.

Example 1
=========

.. code-block::

    "value" : "Red = Red / 256"

This scales the ``Red`` value by 1/256. If the input values are in the range 0 - 65535, the output
value will be in the range 0 - 255.


Example 2
=========

.. code-block::

    "value" :
    [
        "Classification = 2 WHERE HeightAboveGround < 5",
        "Classification = 1 WHERE HeightAboveGround >= 5"
    ]

This sets the classification of points to either ``Ground`` or ``Unassigned`` depending on the
value of the ``HeightAboveGround`` dimension.

Example 3
=========

.. code-block::

    "value" :
    [
        "X = 1",
        "X = 2 WHERE X > 10"
    ]

This sets the value of ``X`` for all points to 1. The second statement is essentially ignored
since the first statement sets the ``X`` value of all points to 1 and therefore no points
the ``ConditionalExpression`` of the second statement.
