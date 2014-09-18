import numpy as np

def fff(ins,outs):
  X = ins['X']
  Result = np.equal(X, 637501.67)
  #print X
  #print Mask
  outs['Mask'] = Result
  return True
