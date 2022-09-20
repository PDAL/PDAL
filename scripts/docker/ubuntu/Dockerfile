FROM condaforge/mambaforge:latest as build
MAINTAINER Howard Butler <howard@hobu.co>

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

RUN conda create -n pdal -y
ARG CI
ARG GITHUB_SHA
ARG GITHUB_REPOSITORY="PDAL/PDAL"
ARG GITHUB_SERVER_URL="https://github.com"

SHELL ["conda", "run", "-n", "pdal", "/bin/bash", "-c"]

RUN mamba install -c conda-forge git compilers conda-pack cmake make ninja sysroot_linux-64=2.17 && \
    mamba install --yes -c conda-forge pdal --only-deps

#RUN if [ -z "${CI+x}" ]; then git clone "${GITHUB_SERVER_URL}/${GITHUB_REPOSITORY}" pdal; else echo "CI is set, we are not going to check it out '$CI'" && env; ls $HOME; find /; fi

RUN git clone "${GITHUB_SERVER_URL}/${GITHUB_REPOSITORY}" pdal && \
    cd pdal && \
    git checkout ${GITHUB_SHA}


RUN ls && pwd && mkdir -p pdal/build && \
    cd pdal/build  && \
    CXXFLAGS="-Werror=strict-aliasing" LDFLAGS="-Wl,-rpath-link,$CONDA_PREFIX/lib" cmake -G Ninja  \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_LIBRARY_PATH:FILEPATH="$CONDA_PREFIX/lib" \
        -DCMAKE_INCLUDE_PATH:FILEPATH="$CONDA_PREFIX/include" \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
        -DBUILD_PLUGIN_CPD=OFF \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_HDF=ON \
        -DBUILD_PLUGIN_TILEDB=ON \
        -DBUILD_PLUGIN_E57=ON \
        -DBUILD_PGPOINTCLOUD_TESTS=OFF \
        -DWITH_ZSTD=ON \
        ..

RUN cd pdal/build  && \
    ninja

RUN cd pdal/build  && \
    ctest -V

RUN cd pdal/build  && \
    ninja install

RUN conda-pack -n pdal --dest-prefix=/opt/conda/envs/pdal -o  /tmp/env.tar && \
     mkdir /venv && cd /venv && tar xf /tmp/env.tar  && \
     rm /tmp/env.tar

FROM condaforge/miniforge3

ENV CONDAENV "/opt/conda/envs/pdal"
COPY --from=build /venv "/opt/conda/envs/pdal"

ENV PROJ_NETWORK=TRUE
ENV PATH $PATH:${CONDAENV}/bin
ENV GTIFF_REPORT_COMPD_CS=TRUE
ENV REPORT_COMPD_CS=TRUE
ENV OAMS_TRADITIONAL_GIS_ORDER=TRUE


SHELL ["conda", "run", "--no-capture-output", "-n", "pdal", "/bin/sh", "-c"]

