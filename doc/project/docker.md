(development_docker)=

# Building Docker Containers for PDAL

PDAL's {ref}`repository <source>` is linked to [DockerHub] for automatic
building of [Docker] containers. PDAL keeps these Docker containers current.

- `pdal/pdal:latest` -- PDAL master
- `pdal/pdal:sha-<commit>` -- a master commit, by the first 8 characters of its SHA
- {{'`pdal/pdal:{ver}` -- PDAL current release'.format(ver=version) }}
- `pdal/pdal:<major>.<minor>` -- the newest release of that series, e.g. `pdal/pdal:2.11`

Each is also published as `ghcr.io/pdal/pdal`, for both `linux/amd64` and
`linux/arm64`.

## Dockerfile

The PDAL Docker container definition is used by both the latest and release
branch Docker containers but it is built using the definition of the file for
the given branch. It is built using the Dockerfile at
<https://github.com/PDAL/PDAL/blob/master/scripts/docker/ubuntu/Dockerfile>

[docker]: https://www.docker.com/
[dockerhub]: https://hub.docker.com/r/pdal/pdal/
[ubuntu bionic]: http://releases.ubuntu.com/18.04/
