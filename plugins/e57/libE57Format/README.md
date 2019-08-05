libE57Format
==
[![Build Status](https://travis-ci.org/asmaloney/libE57Format.svg?branch=master)](https://travis-ci.org/asmaloney/libE57Format)

A library to provide read & write support for the E57 file format.

This is a fork of [E57RefImpl](https://sourceforge.net/projects/e57-3d-imgfmt/) v1.1.332. It is primarily for use in the [CloudCompare](https://github.com/CloudCompare/CloudCompare) project, but it should work for anyone else who wants to use it.

The original source is from [E57RefImpl 1.1.332](https://sourceforge.net/projects/e57-3d-imgfmt/files/E57Refimpl-src/) and then everything was stripped out except the main implementation for reading and writing E57.

This version also removes the dependency on [Boost](http://www.boost.org/) and requires C++11.

Many, many other changes were made prior to the first release of this fork. See the [CHANGELOG](CHANGELOG.md) and git history for details.


Why Fork?
--

The E57RefImpl code hasn't been touched in years and I wanted to make changes to compile this library with macOS. Forking it gives me a bit more freedom to update the code and make changes as required.

I changed the name of the project so that it is not confused with the **E57RefImpl** project.

I have also changed the main include file's name from `E57Foundation.h` to `E57Format.h` to make sure there is no inclusion confusion.

Versions of **libE57Format** start at 2.0.

Tools
--

Ryan Baumann has updated the `e57unpack` and `e57validate` tools to work with **libE57Format**. You can find them in the [e57tools](https://github.com/ryanfb/e57tools) repo.

License
--
[Boost Software License (BSL1.0)](https://opensource.org/licenses/BSL-1.0).
