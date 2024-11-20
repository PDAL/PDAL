(development-alpine)=

# Alpine

This page is intended to provide information about Alpine that may be useful
for PDAL developers, especially when it comes to adding new PDAL dependencies.

## Packages

When adding a dependency to PDAL, you will need to update our Travis
configuration for continuous integration and testing, and Dockerfiles for
automated builds. Begin by checking for your package in
<https://pkgs.alpinelinux.org/packages>. Packages containing binaries can
typically be found by searching for the library/package name alone. Development
files are typically grouped in a separate subpackage with `-dev` appended to
the package name. Libraries are sometimes grouped in yet another subpackage
with `-libs` appended. It may take a little inspection of the package
contents to determine exactly what you are getting with a particular package.

If a package does not yet exist, you'll need to consult
<https://wiki.alpinelinux.org/wiki/Creating_an_Alpine_package> or phone a friend.
Alpine developers can frequently be found on the IRC channel #alpine-devel.

## Travis

We currently run our Travis CI builds by first pulling `alpine:3.6` and then
running a script within the Alpine container. Any new dependencies that are
required for PDAL to be built and tested will need to be added to
<https://github.com/PDAL/PDAL/tree/master/scripts/ci/alpine>.

## Docker

Our Docker automated builds are built from the Dockerfiles located in
<https://github.com/PDAL/PDAL/tree/master/scripts/docker>. There are folders for
each supported release as well as master, and there are variants for Alpine and
Ubuntu based images. In the Alpine Dockerfiles, any development dependencies
should be added in the `apk add` step that uses the `--virtual` switch, as
these will be deleted after compilation. Any runtime dependencies should be
added to the regular `apk add` step.
