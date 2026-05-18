---
author: |
 Howard Butler
contact: |
  howard@hobu.co
date: 11/20/2024
title: Steps for Making a PDAL Release
---

# Release Process

This document describes the process for releasing a new version of PDAL.

1)  If it is a major release, make a release branch. Any changes you make to facilitate the
    release should be made in the release branch.

    >     git checkout -b 2.9-maintenance
    >     git push -u origin 2.9-maintenance

    -  Update the `STABLE_BRANCH` pointer in the GitHub variables at <https://github.com/PDAL/PDAL/settings/variables/actions>

    Do not create a new maintenance branch for a minor release. Simply
    tag a new version off of the `-maintenance` branch in that case.

2)  Set version numbers. The only place you need to set the PDAL version
    number is in the project() function of CMakeLists.txt.

    - CMakeLists.txt

      >     project(PDAL VERSION 2.9.0 LANGUAGES CXX C)

    - Update library version in CMakeLists.txt.
      Note that if there is no breaking ABI change (unlikely),
      you can just update the minor.

      >     set(PDAL_SOLIB_MAJOR 15)
      >     set(PDAL_SOLIB_MINOR 0)
      >     set(PDAL_SOLIB_BUILD 0)

    - Update `_config.yml` `release_date` to today's date
    - Update `_config.yml` `version` to full version number (ie, 2.9.0)
    - Update `doc/download.rst` past release pointer with last release date/version
    - Update the `STABLE_BRANCH` pointer in the GitHub variables at <https://github.com/PDAL/PDAL/settings/variables/actions>
    - Update CITATION.cff with release date and version information

3)  Ensure CI is ✅

4)  If you have time, download or build the latest released Clang and
    GCC and fix any bugs or warnings that show up. Run the tests. If
    you\'re really brave, do the same thing with MSVC. Fix all
    warnings/problems.

5)  Tag the release and push the tag to GitHub. If the tag name is
    `x.x.x`, a draft release will be pushed to <https://github.com/PDAL/PDAL/releases>
    >     git tag 2.9.0
    >     git push origin 2.9.0

6)  Write and update the [draft release notes.](https://github.com/PDAL/PDAL/releases).
    The safest way to do this is to go though all the commits since the last release and reference
    any changes worthy of the release notes. See previous release notes for a template
    and a starting point. If you find issues after making the release branch,
    add them to the release notes.

7) Update Conda package

   - For PDAL releases that bump minor version number, but do not change
     dependencies or build configurations, the
     [regro-cf-autotick-bot]{.title-ref} should automatically create a
     pull request at <https://github.com/conda-forge/libpdal-feedstock>.
     Once the builds succeed, the PR can be merged and the updated
     package will soon be available in the [conda-forge]{.title-ref}
     channel.

   - For major releases, once the Conda Forge machinery has built and
     pushed the package, a pinning migration like
     <https://github.com/conda-forge/conda-forge-pinning-feedstock/pull/6396>
     must be created.

   - Updates that alter the build configuration but do not bump the
     version number should be submitted as PRs from a fork of the
     <https://github.com/conda-forge/libpdal-feedstock> repository. In
     these cases, the build number should be incremented.

8) \[Optional\] Update Alpine package

   - The PDAL Alpine package lives at
     <https://github.com/alpinelinux/aports/blob/master/testing/pdal/APKBUILD>.
     Pull requests can be made against the alpinelinux/aports
     repository. If the build configuration alone is changing, with no
     version increase, simply increment the build number
     [pkgrel]{.title-ref}. If the [pkgver]{.title-ref} is changing,
     then reset [pkgrel]{.title-ref} to 0.
     Pull requests should have a commit message of the following form
     [testing/pdal: \<description\>]{.title-ref}.

