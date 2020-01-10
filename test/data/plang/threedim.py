import numpy as np

def load(filename):
    array = np.load(filename)
    return array

#a = load('threedim.npy')
#print (a)
