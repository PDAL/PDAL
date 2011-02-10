//
// (C) Copyright 2010 Michael P. Gerlek (mpg@flaxen.com)
// Distributed under the BSD License
// (See accompanying file LICENSE.txt or copy at
// http://www.opensource.org/licenses/bsd-license.php)
//

#include <tut/tut.hpp>
#include <iostream>
#include <string>
#include "libpc_test.hpp"

namespace tut
{ 
    struct zipreader_data
    {
        std::string file_las;
        std::string file_laz;

        zipreader_data() :
            file_las(g_test_data_path + "//1.2-with-color.las"),
            file_laz(g_test_data_path + "//1.2-with-color.laz")
        {}
    };

    typedef test_group<zipreader_data> tg;
    typedef tg::object to;

    tg test_group_zipreader("liblas::ZipReader");

    // Test the factory does the right thing and the header is marked as compressed
    template<>
    template<>
    void to::test<1>()
    {
      std::cout << "yow\n";
    }
}
