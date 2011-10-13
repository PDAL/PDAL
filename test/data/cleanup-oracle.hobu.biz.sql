select table_name from user_tables;

drop table PDAL_TEST_BASE;
delete from PDAL_TEST_BLOCKS;
drop table PDAL_TEST_BLOCKS;
delete from user_sdo_geom_metadata where table_name = 'PDAL_TEST_BLOCKS';


exit;
