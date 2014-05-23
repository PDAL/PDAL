#include <cxxabi.h>

int main(int ac, char *av[])
{
    int status;
    char *p = abi::__cxa_demangle("i", 0, 0, &status);
}

