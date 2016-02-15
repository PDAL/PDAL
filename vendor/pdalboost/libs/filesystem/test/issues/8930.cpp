// Before running this test: export LANG=foo

#include <boost/filesystem.hpp>
int main() {
  pdalboost::filesystem::path("/abc").root_directory();
}

