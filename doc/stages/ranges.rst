.. _ranges:

Ranges
======

A range specification is a dimension name, followed by an optional negation
character ('!'), and a starting and ending value separated by a colon, 
surrounded by parentheses or square brackets.  Either the starting or ending
values can be omitted.  Parentheses indicate an open endpoint that doesn't
include the adjacent value.  Square brackets indicate a closed endpoint
that includes the adjacent value.

Example 1:
----------

::

  Z[10:]

Selects all points with a Z value greater than or equal to 10.
  
Example 2:
----------

::

  Classification[2:2]

Selects all points with a classification of 2.

Example 3:
----------

::

  Red!(20:40]

Selects all points with values less than or equal to twenty and those with
values greater than 40

Example 4:
----------

::

  Blue[:255)
  
Selects all points with a blue value less than 255.

Example 5:
----------

::

  Intesity![25:25]

Selects all points with an intensity not equal to 25.
