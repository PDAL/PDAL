:orphan:

.. _PDAL expression:

Expression Syntax
=================

The PDAL expression syntax is a subset of that found in a great many programming languages.
Specifically, it uses a limited set of operators from the C language. Dimension names
can be used where a variable or constant would be used. Double-precision constants are
supported.

All mathematical operations are done with double-precision floating point. There is no
automatic conversion of numeric values to logical values.  For example, the following is
not permitted:

::

  ((Intensity > 0) && Classification)

Instead, you must write:

::

  ((Intensity > 0) && (Classification != 0))

Mathematical Operators
----------------------

.. list-table::
    :widths: 10 30 30
    :header-rows: 1

    * - Operator
      - Function
      - Example
    * - `*`
      - Multiplication
      - Intensity * 64.0
    * - /
      - Division
      - Green / 255
    * - `+`
      - Addition
      - Classification + 255
    * - `-`
      - Subtraction
      - X \- 64215.2

Logical Operators
-----------------

.. list-table::
    :widths: 10 30 30
    :header-rows: 1

    * - Operator
      - Function
      - Example
    * - ! 
      - Not
      - !(X < 25)
    * - `>`
      - Greater
      - X > 52.523
    * - >=
      - Greater Than or Equal
      - X >= 52.523
    * - `<`
      - Less
      - Y < -28.456
    * - <= 
      - Less Than or Equal
      - X <= 0
    * - == 
      - Equal
      - Classification == 7
    * - != 
      - Not Equal
      - Classification != 7
    * - &&
      - And
      - Classification == 7 && Intensity > 64
    * - ||
      - Or
      - Classification == 7 || Classification == 8

The order or operations is as listed, which matches that of the C language. Parentheses are
supported to alter the order of operations.

Examples
--------

::

    ((Classification == 7 || Classification == 8) && NumberOfReturns == 1)

Selects points with a classification of 7 or 8 and number of returns equal to 1.  Note
that in this case the parentheses are necessary.

::

    X > 2500 && X < 4700 && Y > 0

Selects points with an X between 2500 and 4700 and a positive Y value.

::

    (NumberOfReturns > 1 && ReturnNumber == 1)

Selects "first" returns from a laser pulse.

::

    !(NumberOfReturns == 1)

Selects only those points where the laser pulse generated multiple returns.
