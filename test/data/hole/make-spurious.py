from laspy.file import File
import copy
inFile = File("autzen-dd.las", mode = "r")
header = inFile.header
points = copy.deepcopy(inFile.points)

import numpy as np

X = inFile.X
Y = inFile.Y

minx = np.min(X)
miny = np.min(Y)

maxx = np.max(X)
maxy = np.max(Y)

print minx, miny, maxx, maxy
x = ((maxx - minx)/2.0 + minx) - 1.0/header.scale[0]
y = (maxy - miny)/2.0 + miny

print x, y

points[0][0][0] = x

print points[0][0][0]
outfile = File("spurious.las", mode='w', header = header)
outfile.points = points
import pdb;pdb.set_trace()