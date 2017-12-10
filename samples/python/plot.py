
import sys
import numpy as np
import matplotlib.pyplot as plt

kIsCartesian = False
lbl_xyz = ['x','y','z']
lbl_quat = ['qx','qy','qz', 'qw']
plt.ion()

argc = len( sys.argv )
if argc < 2:
  raise RuntimeError( "Expects an argument: path to csv file to plot.")

fp = sys.argv[1]
print( "Will plot {}".format( fp ) )
data = np.loadtxt( fp, delimiter = ";" )

t = data[:,0]
seg = data[:,1]
s = data[:,2]
ds = data[:,3]
dds = data[:,4]
j = data[:,5]

plt.plot( t, seg )
plt.title( "segments" )

f, axarr = plt.subplots(3)
axarr[0].plot(t, s)
axarr[0].set_title( "s" )
axarr[1].plot(t, ds)
axarr[1].set_title( "ds" )
axarr[2].plot(t, dds)
axarr[2].set_title( "dds" )

fourNs = data.shape[1] - 6  # remaining x, dx, ddx, j 
if fourNs == (7 + 6*3):
  kIsCartesian = True
  N = 6
else:
  kIsCartesian = False
  N = int( fourNs / 4.0 )

idx = 6
if not kIsCartesian:
  x = data[:,idx:idx+N]
  idx = idx + N
else:
  x = data[:,idx:idx+7]
  idx = idx + 7
dx = data[:,idx:idx+N]
idx = idx + N
ddx = data[:,idx:idx+N]
idx = idx + N
#ddx = data[:,idx:idx+N]
#idx = idx + N


if kIsCartesian:
  f, axarr = plt.subplots(2,3)
  ax_i = 0
  ax_j = 0
  for i in range( 3 ):
    axarr[ax_i+0, ax_j].plot(t, x[:,i], label="dof={}".format( lbl_xyz[i] ) )
  axarr[ax_i+0, ax_j].set_title( "position" )
  axarr[ax_i+0, ax_j].legend()
  for i in range( 4 ):
    axarr[ax_i+1, ax_j].plot(t, x[:,3+i], label="dof={}".format( lbl_quat[i] ) )
  axarr[ax_i+1, ax_j].set_title( "orientation" )
  axarr[ax_i+1, ax_j].legend()
  ax_j += 1
  for (vec, name) in [( dx, 'velocity' ), ( ddx, 'acceleration' ) ]:
    for i in range( 3 ):
      axarr[ax_i+0, ax_j].plot(t, vec[:,i], label="dof={}".format( lbl_xyz[i] ) )
    axarr[ax_i+0, ax_j].set_title( "linear " + name )
    axarr[ax_i+0, ax_j].legend()
    for i in range( 3 ):
      axarr[ax_i+1, ax_j].plot(t, vec[:,3+i], label="dof={}".format( lbl_xyz[i] ) )
    axarr[ax_i+1, ax_j].set_title( "angular " + name )
    axarr[ax_i+1, ax_j].legend()
    ax_j += 1
    
else:
  f, axarr = plt.subplots(1,3)
  ax_j = 0
  for (vec, name) in [( x, 'position' ), ( dx, 'velocity' ), ( ddx, 'acceleration' ) ]:
    for i in range( N ):
      axarr[ax_j].plot(t, vec[:,i], label="dof={}".format( lbl_xyz[i] ) )
    axarr[ax_j].set_title( name )
    axarr[ax_j].legend()
    ax_j += 1
  


plt.show()

try:
  input("Enter to quit.")
except:
  pass
