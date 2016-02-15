#include <boost/filesystem.hpp>

int main(void)
{
    pdalboost::filesystem::copy_file("a", "b");
    return 0;
}

