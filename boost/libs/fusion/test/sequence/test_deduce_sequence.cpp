
#include <boost/mpl/vector.hpp>
#include <boost/fusion/support.hpp>

typedef pdalboost::fusion::traits::deduce_sequence < 

pdalboost::mpl::vector<int, char> 

>::type seq1_t;


typedef pdalboost::fusion::traits::deduce_sequence < 

pdalboost::fusion::vector<int, char> 

>::type seq2_t;
