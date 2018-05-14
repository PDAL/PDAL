.. _using:

===============================================================================
Contributing to PDAL
===============================================================================

PDAL is a small project with few developers.  We would appreciate having
help building on our existing software.  In particular, we are interested
in new drivers and filters that fit into the existing infrastructure.

When contributing, please follow along with existing practice.  If you
have ideas for improving what we're doing, a discussion prior to a
PR would be appreciated.  Here are some guidelines that may be helpful:

- Don't make changes to existing code that don't need to be made.  If the
  code is working, there is no reason to "improve" it unless it's in the
  process of adding functionality or fixing problems.

- When you're modifying code in an existing file, follow along with what
  exists, updating where appropriate.  Don't modify an entire file because
  you're making a small change.

- Don't optimize at the expense of clarity unless you have a really good
  reason.

- Try to keep lines no more than 80 characters long.

- We have only moderately good test coverage and we don't do formal reviews,
  so it's on you to review your own code carefully and write the tests you
  need to be convinced that it's correct.  Someone else may look things over,
  but probably without as much knowledge as you have about the functionality
  you're introducting.

- Sometimes it makes sense to do something against the guidelines, but
  you should have a decent reason for doing so.
