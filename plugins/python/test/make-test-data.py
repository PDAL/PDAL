
import laspy

f = laspy.file.File('../../../test/data/las/1.2-with-color.las')

import numpy
import pdb;pdb.set_trace()
out = open('../../../test/data/plang/1.2-with-color.npy','wb')
numpy.save(out, f.points['point'], allow_pickle=False)
out.close()
