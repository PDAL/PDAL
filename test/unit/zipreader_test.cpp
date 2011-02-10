//
// (C) Copyright 2010 Michael P. Gerlek (mpg@flaxen.com)
// Distributed under the BSD License
// (See accompanying file LICENSE.txt or copy at
// http://www.opensource.org/licenses/bsd-license.php)
//

#ifdef HAVE_LASZIP

#include <liblas/liblas.hpp>
#include <liblas/variablerecord.hpp>
#include <tut/tut.hpp>
#include <fstream>
#include <string>
#include "liblas_test.hpp"
#include "common.hpp"

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
        std::ifstream ifs_las;
        ifs_las.open(file_las.c_str(), std::ios::in | std::ios::binary);
        std::ifstream ifs_laz;
        ifs_laz.open(file_laz.c_str(), std::ios::in | std::ios::binary);

        liblas::ReaderFactory factory;
        liblas::Reader reader_las = factory.CreateWithStream(ifs_las);
        liblas::Reader reader_laz = factory.CreateWithStream(ifs_laz);

        liblas::Header const& header_las = reader_las.GetHeader();
        liblas::Header const& header_laz = reader_laz.GetHeader();

        ensure_equals(header_las.Compressed(), false);
        ensure_equals(header_laz.Compressed(), true);
    }

    // Test reading 3 points (via ReadNext)
    template<>
    template<>
    void to::test<2>()
    {
        std::ifstream ifs_las;
        ifs_las.open(file_las.c_str(), std::ios::in | std::ios::binary);
        std::ifstream ifs_laz;
        ifs_laz.open(file_laz.c_str(), std::ios::in | std::ios::binary);

        liblas::ReaderFactory factory;
        liblas::Reader reader_las = factory.CreateWithStream(ifs_las);
        liblas::Reader reader_laz = factory.CreateWithStream(ifs_laz);

        reader_las.ReadNextPoint();
        liblas::Point p0_las = reader_las.GetPoint();
        reader_las.ReadNextPoint();
        liblas::Point p1_las = reader_las.GetPoint();
        reader_las.ReadNextPoint();
        liblas::Point p2_las = reader_las.GetPoint();

        reader_laz.ReadNextPoint();
        liblas::Point p0_laz = reader_laz.GetPoint();
        reader_laz.ReadNextPoint();
        liblas::Point p1_laz = reader_laz.GetPoint();
        reader_laz.ReadNextPoint();
        liblas::Point p2_laz = reader_laz.GetPoint();

        ensure_equals(p0_las, p0_laz);
        ensure_equals(p1_las, p1_laz);
        ensure_equals(p2_las, p2_laz);

        test_file_12Color_point0(p0_las);
        test_file_12Color_point1(p1_las);
        test_file_12Color_point2(p2_las);
        test_file_12Color_point0(p0_laz);
        test_file_12Color_point1(p1_laz);
        test_file_12Color_point2(p2_laz);

        return;
    }

    // Test reading 3 points (via ReadAt)
    template<>
    template<>
    void to::test<3>()
    {
        std::ifstream ifs_las;
        ifs_las.open(file_las.c_str(), std::ios::in | std::ios::binary);
        std::ifstream ifs_laz;
        ifs_laz.open(file_laz.c_str(), std::ios::in | std::ios::binary);

        liblas::ReaderFactory factory;
        liblas::Reader reader_las = factory.CreateWithStream(ifs_las);
        liblas::Reader reader_laz = factory.CreateWithStream(ifs_laz);

        // test ReadPointAt()
        {
            reader_las.ReadPointAt(2);
            liblas::Point p2_las = reader_las.GetPoint();
            reader_las.ReadPointAt(1);
            liblas::Point p1_las = reader_las.GetPoint();
            reader_las.ReadPointAt(0);
            liblas::Point p0_las = reader_las.GetPoint();

            test_file_12Color_point0(p0_las);
            test_file_12Color_point1(p1_las);
            test_file_12Color_point2(p2_las);

            reader_laz.ReadPointAt(2);
            liblas::Point p2_laz = reader_laz.GetPoint();
            reader_laz.ReadPointAt(1);
            liblas::Point p1_laz = reader_laz.GetPoint();
            reader_laz.ReadPointAt(0);
            liblas::Point p0_laz = reader_laz.GetPoint();
    
            ensure_equals(p0_las, p0_laz);
            ensure_equals(p1_las, p1_laz);
            ensure_equals(p2_las, p2_laz);

            test_file_12Color_point0(p0_laz);
            test_file_12Color_point1(p1_laz);
            test_file_12Color_point2(p2_laz);
        }

        // test Seek()
        {
            reader_las.Seek(1);
            reader_las.ReadNextPoint();
            liblas::Point p1_las = reader_las.GetPoint();
            reader_las.Seek(0);
            reader_las.ReadNextPoint();
            liblas::Point p0_las = reader_las.GetPoint();
            reader_las.Seek(2);
            reader_las.ReadNextPoint();
            liblas::Point p2_las = reader_las.GetPoint();

            test_file_12Color_point0(p0_las);
            test_file_12Color_point1(p1_las);
            test_file_12Color_point2(p2_las);

            reader_laz.Seek(1);
            reader_laz.ReadNextPoint();
            liblas::Point p1_laz = reader_laz.GetPoint();
            reader_laz.Seek(0);
            reader_laz.ReadNextPoint();
            liblas::Point p0_laz = reader_laz.GetPoint();
            reader_laz.Seek(2);
            reader_laz.ReadNextPoint();
            liblas::Point p2_laz = reader_laz.GetPoint();
    
            ensure_equals(p0_las, p0_laz);
            ensure_equals(p1_las, p1_laz);
            ensure_equals(p2_las, p2_laz);

            test_file_12Color_point0(p0_laz);
            test_file_12Color_point1(p1_laz);
            test_file_12Color_point2(p2_laz);
        }
    }

    // Test the VLR
    template<>
    template<>
    void to::test<4>()
    {
        std::ifstream ifs;
        ifs.open(file_laz.c_str(), std::ios::in | std::ios::binary);

        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);

        liblas::Header const& header = reader.GetHeader();
        test_laszip_vlr(header);

        return;
    }
}

#endif // HAVE_LASZIP
