set echo on;
declare 
    blks VARCHAR(40);
    clouds VARCHAR(40);
    name VARCHAR(40);
metadata VARCHAR(200);
begin

name := '&1';
clouds := name ||'_CLOUD';
blks := name ||'_BLOCKS';

FOR i IN (SELECT table_name FROM user_tables WHERE table_name = clouds) LOOP
    EXECUTE IMMEDIATE 'DROP TABLE ' || clouds || ' PURGE';
END LOOP;

FOR i IN (SELECT table_name FROM user_tables WHERE table_name = blks) LOOP
    EXECUTE IMMEDIATE 'delete from ' || blks;
    EXECUTE IMMEDIATE 'DROP TABLE ' || blks || ' PURGE';
END LOOP;


execute immediate 'purge recyclebin';
execute immediate 'purge tablespace GRID';
execute immediate 'CREATE TABLE '|| clouds ||'(id number, CLOUD SDO_PC, BOUNDARY SDO_GEOMETRY)';
metadata := q'{delete from user_sdo_geom_metadata where table_name =  :blks}';

execute immediate metadata using blks;
commit;

end;
/

exit;