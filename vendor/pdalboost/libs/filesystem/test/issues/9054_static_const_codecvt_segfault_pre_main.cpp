#include "boost/filesystem.hpp"

static const pdalboost::filesystem::path::codecvt_type &dummy =
  pdalboost::filesystem::path::codecvt();

int main()
{
  return 0;
}

