import os
import sys

src_directory='/path/to/diretory/with/las/files'
gtxfile='/path/to/yourgridfile.gtx'
jsonfile='/path/to/pipeline.json'
horizontal_srs='EPSG:2193'

dest_directory = src_directory + '/reprojected'
if not os.path.exists(dest_directory): os.mkdir(dest_directory)         
   
for filename in os.listdir(src_directory):
    if (filename.endswith('.las') or filename.endswith('.laz')):
        print('Reprojecting ' + filename)
        pdal_cmd ='pdal pipeline {} --readers.las.filename={}  --writers.las.filename={} --filters.reprojection.out_srs="+init={} +geoidgrids={}"'.format(jsonfile, src_directory + '/' + filename, dest_directory + '/' + filename, horizontal_srs,gtxfile)       
        os.system(pdal_cmd)
