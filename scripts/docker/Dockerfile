FROM pdal/dependencies:latest
MAINTAINER Howard Butler <howard@hobu.co>
#ARG branch=master

ENV CC clang
ENV CXX clang++

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
        cython \
        python-pip \
        libhpdf-dev \
    && rm -rf /var/lib/apt/lists/*


#RUN pip install packaging \
#    && pip install PDAL

