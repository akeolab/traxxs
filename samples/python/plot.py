import sys
import numpy as np
import matplotlib.pyplot as plt

argc = len( sys.argv )
if argc < 2:
  raise RuntimeError( "Expects an argument: path to csv file to plot.")

fp = sys.argv[1]
print( "Will plot {}".format( fp ) )
data = np.loadtxt( fp, delimiter = ";" )

t = data[:,0]
s = data[:,1]
ds = data[:,2]
dds = data[:,3]
j = data[:,4]

f, axarr = plt.subplots(4, sharex=True)
axarr[0].plot(t, s)
axarr[1].plot(t, ds)
axarr[2].plot(t, dds)
axarr[3].plot(t, j)


for i in range( data.shape[0] ):
  print( "t = {: >8.3f} \t {: >8.3f}  \t {: >8.3f}  \t {: >8.3f}  \t {: >8.3f} ".format( t[i], s[i], ds[i], dds[i], j[i] ) )

plt.show()
