#define BOOST_TEST_DYN_LINK

#include "support.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>

#include <libpc/Utils.hpp>


bool compare_files(const std::string& file1, const std::string& file2)
{
    uintmax_t len1 = libpc::Utils::fileSize(file1);
    uintmax_t len2 = libpc::Utils::fileSize(file1);

    if (len1 != len2)
    {
        return false;
    }

    std::istream* str1 = libpc::Utils::openFile(file1);
    std::istream* str2 = libpc::Utils::openFile(file2);
    BOOST_CHECK(str1);
    BOOST_CHECK(str2);

    char* buf1 = new char[len1];
    char* buf2 = new char[len2];

    str1->read(buf1,len1);
    str2->read (buf2,len2);

    libpc::Utils::closeFile(str1);
    libpc::Utils::closeFile(str2);

    char* p = buf1;
    char* q = buf2;

    for (uintmax_t i=0; i<len1; i++)
    {
        if (*p != *q) 
        {
            delete[] buf1;
            delete[] buf2;
            return false;
        }
        ++p;
        ++q;
    }

    delete[] buf1;
    delete[] buf2;

    return true;
}
