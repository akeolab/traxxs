import sys
import numpy as np
import json

import pandas as pd
import matplotlib.pyplot as plt

def existsOR( data, keys ):
  ret = False
  for k in keys:
    ret = ret or (k in data)
  return ret

def existsAND( data, keys ):
  ret = True
  for k in keys:
    ret = ret and (k in data)
  return ret

def sub( series, i ):
  return [ v[i] for v in series.values[:-1] ]

kIsCartesian = False
lbl_xyz = ['x','y','z']
lbl_quat = ['qx','qy','qz', 'qw']
ln_color = ['r', 'g', 'b', 'k']

plt.ion()

argc = len( sys.argv )
if argc < 2:
  raise RuntimeError( "Expects an argument: path to json file to plot.")

fp = sys.argv[1]
print( "Will plot {}".format( fp ) )

f = open( fp )
f_str = f.read()
f_str = f_str.replace( "nan", "\"NaN\"" )
json_data = json.loads( f_str )

data = pd.DataFrame.from_dict( json_data['data'] )

kIsCartesian = False
if 'type' in json_data:
  kIsCartesian = ( json_data['type'].lower().startswith('cart') )
if 'x' in data and 'dx' in data:
  kIsCartesian = False
  if len( data['x'].values[0] ) == 7 and len( data['dx'].values[0] ) == 6:
    kIsCartesian = True
    

print( "{}".format( "CARTESIAN" if kIsCartesian else "VECTOR" ) )

t = data['time'].values[:-1]

if existsOR( data, ['s', 'ds', 'dds'] ):
  f, axarr = plt.subplots(3)
  if 's' in data:
    axarr[0].plot( t, data['s'].values[:-1] )
    axarr[0].set_title( "s" )
  if 'ds' in data:
    axarr[1].plot( t, data['ds'].values[:-1] )
    axarr[1].set_title( "ds" )
  if 'dds' in data:
    axarr[2].plot( t, data['dds'].values[:-1] )
    axarr[2].set_title( "dds" )
    
if existsOR( data, ['seg_idx',] ):
  f, axarr = plt.subplots(1)
  axarr.plot( t, data['seg_idx'].values[:-1] )
  axarr.set_title( "segment index" )

if kIsCartesian:
  suffixes = []
  if existsOR( data, ['x', 'dx', 'ddx'] ):
    suffixes.append( ('','-') )
  if existsOR( data, ['x_target', 'dx_target', 'ddx_target'] ):
    suffixes.append( ('_target',':') )
  ft, axarrt = plt.subplots(1,2)
  f, axarr = plt.subplots(2,2)
  for (suffix, ln) in suffixes:
    ax_i = 0
    ax_j = 0
    for i in range( 3 ):
      axarrt[ax_i+0].plot(t, sub( data['x'+suffix], i ), ln+ln_color[i], label="dof={}".format( lbl_xyz[i]+suffix ) )
    axarrt[ax_i+0].set_title( "position" )
    axarrt[ax_i+0].legend()
    for i in range( 4 ):
      axarrt[ax_i+1].plot(t, sub( data['x'+suffix], 3+i ), ln+ln_color[i], label="dof={}".format( lbl_quat[i]+suffix ) )
    axarrt[ax_i+1].set_title( "orientation" )
    axarrt[ax_i+1].legend()
    for (vec, name) in [( data['dx'+suffix], 'velocity' ), ( data['ddx'+suffix], 'acceleration' ) ]:
      for i in range( 3 ):
        axarr[ax_i+0, ax_j].plot(t, sub( vec, i ), ln+ln_color[i], label="dof={}".format( lbl_xyz[i]+suffix ) )
      axarr[ax_i+0, ax_j].set_title( "linear " + name )
      axarr[ax_i+0, ax_j].legend()
      for i in range( 3 ):
        axarr[ax_i+1, ax_j].plot(t, sub( vec, 3+i ), ln+ln_color[i], label="dof={}".format( lbl_xyz[i]+suffix ) )
      axarr[ax_i+1, ax_j].set_title( "angular " + name )
      axarr[ax_i+1, ax_j].legend()
      ax_j += 1

else: # NOT cartesian
  suffixes = []
  if existsOR( data, ['x', 'dx', 'ddx'] ):
    suffixes.append( ('','-') )
  if existsOR( data, ['x_target', 'dx_target', 'ddx_target'] ):
    suffixes.append( ('_target',':') )
  f, axarr = plt.subplots(1,3)
  for (suffix, ln) in suffixes:
    ax_j = 0
    for (vec, name) in [( data['x'+suffix], 'position' ), ( data['dx'+suffix], 'velocity' ), ( data['ddx'+suffix], 'acceleration' ) ]:
      for i in range( len(vec.values[0]) ):
        axarr[ax_j].plot(t, sub( vec, i ), label="dof={}".format( lbl_xyz[i] ) )
      axarr[ax_j].set_title( name )
      axarr[ax_j].legend()
      ax_j += 1
  

plt.show()

try:
  input("Enter to quit.")
except:
  pass

