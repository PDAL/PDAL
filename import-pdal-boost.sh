#
# file: import-pdal-boost.sh
#
# How to update the embedded copy of boost
# ========================================
#
# Prerequisites:
#    1. current build of boost, including tools/bcp
#
# Instructions:
#    1. Remove the current embedded copy of boost.
#        $ cd $PDAL_ROOT
#        $ rm -r boost
#    2. Find current included set of boost headers.
#        $ export BOOST_HEADERS=\
#             `find src include test apps \
#                  \( -name '*.[cChH]' -o \
#                     -name '*.[cChH][pPxX][pPxX]' -o \
#                     -name '*.[cChH][cChH]' \) \
#                  -exec grep 'include.*boost' {} \; \
#              | grep '^#' \
#              | sed -e 's/.*boost/boost/' -e 's/>.*//' \
#              | sort -u`
#    3. Run bcp.
#        $ cd $BOOST_ROOT
#        $ ./bcp --namespace=pdalboost --namespace-alias \
#              $BOOST_HEADERS $PDAL_ROOT/boost
#    4. Stage.
#        $ cd $PDAL_ROOT
#        $ git add boost
#        $ git status         # verify this looks sane
#    5. Fix files (bcp misses a couple of namespace issues)
#        $ vim boost/boost/property_tree/detail/xml_parser_read_rapidxml.hpp \
#            # "namespace rapidxml" -> "namespace pdalboostrapidxml"
#    6. Test it.
#        $ cmake -DPDAL_EMBED_BOOST=ON .
#    7. Check it in.
#        $ git commit
#
echo "see comments for instructions"
exit 1

export BCPDIR=/Users/kmckelvey/Source/boost_1_48_0
export TARGET=/Users/kmckelvey/Source/boost-header-diff-test
export BOOST_HEADERS=`find src include test apps \( -name '*.[cChH]' -o -name '*.[cChH][pPxX][pPxX]' -o -name '*.[cChH][cChH]' \) -exec grep 'include.*boost' {} \; | grep '^#' | sed -e 's/.*boost/boost/' -e 's/>.*//' | sort -u`

export BOOST_HEADERS="$BOOST_HEADERS boost/parameter/aux_/overloads.hpp"
mkdir -p $TARGET
cd $BCPDIR
./bcp --namespace=pdalboost --namespace-alias $BOOST_HEADERS $TARGET
