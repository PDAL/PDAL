# Need this one to be global so multiple stages can use it
ARG PYTHON_VERSION="3.13"

FROM condaforge/miniforge3:latest AS build
LABEL org.opencontainers.image.authors="howard@hobu.co"

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN conda create -n pdal -y
ARG GITHUB_SHA
ARG GITHUB_REPOSITORY="PDAL/PDAL"
ARG GITHUB_SERVER_URL="https://github.com"

SHELL ["conda", "run", "-n", "pdal", "/bin/bash", "-c"]

RUN mamba install -y \
    git compilers cmake ninja make \
    conda-pack \
    python=${PYTHON_VERSION} pip numpy scikit-build-core pybind11 \
    && \
    mamba install -y libpdal --only-deps

RUN git clone "${GITHUB_SERVER_URL}/${GITHUB_REPOSITORY}" pdal && \
    cd pdal ; \
    git checkout ${GITHUB_SHA}

RUN mkdir -p pdal/build && \
    cd pdal/build  && \
    CXXFLAGS="-Werror=strict-aliasing" LDFLAGS="-Wl,-rpath-link,$CONDA_PREFIX/lib" cmake -G Ninja  \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH:FILEPATH="$CONDA_PREFIX/lib" \
    -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
    -DBUILD_PLUGIN_CPD=ON \
    -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
    -DBUILD_PLUGIN_NITF=ON \
    -DBUILD_PLUGIN_ICEBRIDGE=ON \
    -DBUILD_PLUGIN_HDF=ON \
    -DBUILD_PLUGIN_ARROW=ON \
    -DBUILD_PLUGIN_DRACO=ON \
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

RUN git clone https://github.com/PDAL/python-plugins.git pdal-python-plugins && \
    cd pdal-python-plugins && git checkout 1.6.6  && \
    python -m pip install \
    -Cbuild-dir=build \
    . \
    --config-settings=cmake.build-type="Release" \
    -vv \
    --no-deps \
    --no-build-isolation

RUN git clone https://github.com/PDAL/python.git pdal-python && \
    cd pdal-python && git checkout 3.5.3  && \
    python -m pip install \
    -Cbuild-dir=build \
    . \
    --config-settings=cmake.build-type="Release" \
    -vv \
    --no-deps \
    --no-build-isolation

RUN conda-pack -n pdal --dest-prefix=/opt/conda/envs/pdal -o  /tmp/env.tar && \
    mkdir /venv && cd /venv && tar xf /tmp/env.tar  && \
    rm /tmp/env.tar

FROM condaforge/miniforge3

ARG PYTHON_VERSION

ENV CONDAENV="/opt/conda/envs/pdal"

COPY --from=build /venv "${CONDAENV}"

ENV PROJ_NETWORK=TRUE
ENV PROJ_DATA="${CONDAENV}/share/proj"
ENV GDAL_DATA="${CONDAENV}/share/gdal"
ENV GEOTIFF_CSV="${CONDAENV}/share/epsg_csv"

ENV PDAL_DRIVER_PATH="${CONDAENV}/lib:${CONDAENV}/lib/python${PYTHON_VERSION}/site-packages/pdal/"
ENV GDAL_DRIVER_PATH="${CONDAENV}/lib/gdalplugins"

ENV PATH="$PATH:${CONDAENV}/bin"

ENV GTIFF_REPORT_COMPD_CS=TRUE
ENV REPORT_COMPD_CS=TRUE

SHELL ["conda", "run", "--no-capture-output", "-n", "pdal", "/bin/sh", "-c"]
