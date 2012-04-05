select table_name from user_tables;
drop table SIMPLE_CLOUD;
delete from SIMPLE_BLOCKS;
drop table SIMPLE_BLOCKS;
CREATE TABLE SIMPLE_CLOUD (id number, CLOUD SDO_PC, BOUNDARY SDO_GEOMETRY);

delete from user_sdo_geom_metadata where table_name = 'SIMPLE_BLOCKS';
exit;
