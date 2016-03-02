
from osgeo import gdal
gdal.UseExceptions()

import struct

objs = {}

def wakeup(filename):

    objs['ds'] = gdal.Open(filename)
    objs['gt'] = objs['ds'].GetGeoTransform()
    objs['band'] = objs['ds'].GetRasterBand(1)
    return objs


def read(x, y, objs):

    # stolen from http://stackoverflow.com/questions/24537450/python-struct-error-unpack-requires-a-string-argument-of-length-2
    gt = objs['gt']
    ds = objs['ds']
    band = objs['band']

    px = int((x - gt[0]) / gt[1])
    py = int((y - gt[3]) / gt[5])

    val = band.ReadRaster(px,py,1,1,buf_type=gdal.GDT_Float32)
    z = struct.unpack('f' , val)
    return z[0]

def filter(ins,outs):
    HAG = ins['HAG']
    X = ins['X']
    Y = ins['Y']
    Z = ins['Z']

    objs = wakeup('@CMAKE_SOURCE_DIR@/test/data/autzen/autzen-surface.tif.min.tif')
    print (objs)
    for i in range(len(X)):
        x = X[i]
        y = Y[i]
        z = Z[i]
        surface_z = read(x, y, objs)
#        print (x, y, z, surface_z)
        hag = z - surface_z
        if (surface_z  != -9999):
            HAG[i] = hag
    outs['Z'] = HAG
    return True

if __name__=='__main__':
    objs = wakeup('@CMAKE_SOURCE_DIR@/test/data/autzen/autzen-surface.tif.min.tif')
    print (read(636436,850412, objs))
