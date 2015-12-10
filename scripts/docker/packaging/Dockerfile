FROM pdal/dependencies
MAINTAINER Howard Butler <howard@hobu.co>

# Get howard@hobu.co key
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys BFE1B014
COPY howard-hobu-co-gpg-private.key /
RUN gpg --allow-secret-key-import --import /howard-hobu-co-gpg-private.key

RUN git config --global user.email "howard@hobu.co"
RUN git config --global user.name "Howard Butler"

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
	packaging-dev \
	git-buildpackage \
	fakeroot \
	pdebuild \
	cowbuilder \
	&& rm -rf /var/lib/apt/lists/*


ENV DEBFULLNAME="Howard Butler"
ENV DEBEMAIL=howard@hobu.co

