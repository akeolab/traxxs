import numpy as np
import matplotlib.pyplot as plt

class Segment( object ):
  def __init__( self, t=0, j=0 ):
    self.init( t, j )
  def init( self, t, j ):
    self.t = t
    self.j = j
    
  def __str__(self):
    return "t={}, j={}".format( self.t, self.j )

class Conds( object ):
  def __init__( self, v=0, a=0, j=0 ):
    self.init( v, a, j )
  def init( self, v, a, j ):
    self.v = v
    self.a = a
    self.j = j
  
class Limits( object ):
  def __init__( self, maxVel=0, maxAcc=0, maxJerk=0 ):
    self.init( maxVel, maxAcc, maxJerk )
  def init( self, maxVel, maxAcc, maxJerk ):
    self.maxVel = maxVel
    self.maxAcc = maxAcc
    self.maxJerk = maxJerk
    
def find_duration_vel( v_i, v_target, a, j ):
  t_v = 0
  if j == 0:
    if a == 0:
      t_v = 0
    else:
      t_v = (v_target - v_i) * 1.0 / a
  else:
    p_a = j/2.0
    p_b = a 
    p_c = v_i - v_target
    delta = p_b * p_b - 4.0 * p_a * p_c
    if ( delta < 0 ) :
      raise( "Infeasibility encountered." ) # should NEVER happen
    t1 = ( -p_b - np.sqrt( delta ) ) / ( 2.0 * p_a )
    t2 = ( -p_b + np.sqrt( delta ) ) / ( 2.0 * p_a )
    if t1 < 0:
      t_v = t2
    elif t2 < 0:
      t_v = t1
    else:
      t_v = min( t1, t2 )
      
  return t_v
    
def compute_braking( ci, lims ):
  segments = []
  
  #
  # First phase: reach the closest acceleration bound
  #
  target_v = None
  target_a = None
  
  
  # The acceleration should reach its boundary first ! (e.g. vel > lim, acc < -lim -> we start with acc = -lim and wait for vel = lim)
  if ( ci.a > lims.maxAcc ):
    target_a = lims.maxAcc
  if ( ci.a < -lims.maxAcc ):
    target_a = -lims.maxAcc
    
  if ci.v <= lims.maxVel and ci.v >= -lims.maxVel and target_a is None: # no need to do anything
    return segments
  
  j = 0.0
  if target_a is not None and ci.a < target_a: # we need j > 0
    j = +lims.maxJerk
  if target_a is not None and ci.a > target_a: # we need j < 0
    j = -lims.maxJerk
  
  t = 0
  if target_a is not None:
    # find the duration of the first phase
    # target_a = a + t*j
    t = (target_a - ci.a) * 1.0 / j
  
    segments.append(Segment( t = t, j = j ))
  
  # At this point, acceleration is within the bounds. Only velocity might be out of bounds.
  
  #
  # Second phase: reach a velocity limit OR another acceleration limit in this direction
  #
  # compute the velocity and acceleration at the end of the first phase
  v = ci.v + t * ci.a + t*t/2.0 * j
  a = ci.a + t * j 
  
  target_v = v # by default, keep it that way
  if ( v > lims.maxVel ):
    target_v = lims.maxVel
  if ( v < -lims.maxVel ):
    target_v = -lims.maxVel
    
  # deduce a target acceleration based on velocity
  target_a = 0 # by default, we keep constant velocity
  if v > target_v : # we need to decelerate, i.e. reach a = -maxAcc
    target_a = -lims.maxAcc
  if v < target_v : # we need to accelerate, i.e. reach a = +maxAcc
    target_a = +lims.maxAcc
  
  j = 0
  if target_a is not None and a < target_a: # we need j > 0
    j = +lims.maxJerk
  if target_a is not None and a > target_a: # we need j < 0
    j = -lims.maxJerk
  
  # find the duration of the second phase
  
  # the duration to reach the target_v 
  # target_v = v + t*a + t*t/2*j
  t_v = find_duration_vel( v, target_v, a, j )
  
  # the duration to reach one of the acceleration limits
  target_a = 0
  if j > 0: # the limit we might reach is the max
    target_a = +lims.maxAcc
  elif j < 0:
    target_a = -lims.maxAcc
  t_a = np.Infinity
  if j != 0:
    t_a = (target_a - a) * 1.0 / j
  
  t = min( t_v, t_a )
  
  segments.append( Segment( t = t, j = j ) )
  
  v = v + t * a + t*t/2.0 * j
  a = a + t * j 
  
  
  #
  # Third phase: constant acceleration until one of the velocity limits is reached
  # this one is faster, but may pose viability issues (if jerk limited and deceleration too strong, velocity will reach the other bound (cross the entire domain) before we could reach acc = 0
  #j = 0.0
  #t = find_duration_vel( v, target_v, a, j )
  
  # So we use this one instead:
  # Third phase: linear acceleration until one of the velocity limits is reached with acc = 0
  #
  if ( np.abs( a ) > 0 ):
    t =  2 * ( target_v - v ) / a
    if t > 0:
      j = -a / t
    
  
  
  segments.append( Segment( t = t, j = j ) )
  
  v = v + t * a + t*t/2.0 * j
  a = a + t * j 
  
  #print( "{}, {} -> {}, {}".format( ci.v, ci.a, v, a ) )
  
  #
  # Optional segment: make sure we reach acceleration = 0
  #
  if ( np.abs( a ) > 0 ):
    if  a < 0: # we need j > 0
      j = +lims.maxJerk
    if a > 0: # we need j < 0
      j = -lims.maxJerk
    t = -a / j
    segments.append( Segment( t = t, j = j ) )
  
  return segments


def plot_segments( ci, limits, segs ):
  
  def plotlim( ax, t, lim, segs ):
    ax.plot( t, np.zeros( (len(t),) ), 'k' )
    ax.plot( t, +lim*np.ones( (len(t),) ), 'r:' )
    ax.plot( t, -lim*np.ones( (len(t),) ), 'r:' )
    
    t_0 = 0.0
    colors = ['r', 'g', 'b', 'c', 'm']
    for iseg in range(len(segs)):
      seg = segs[iseg]
      ax.plot( [t_0, t_0+seg.t], [0, 0], colors[iseg], linewidth=10.0, alpha=0.5 )
      t_0 += seg.t
  
  dt_0 = 0.01
  v = [ci.v,]
  a = [ci.a,]
  j = [ci.j,]
  t = [0.0,]
  for seg in segs:
    N = int( seg.t / dt_0 )
    if N == 0:
      continue
    dt = seg.t * 1.0 / N
    t_t = np.linspace( t[-1] + dt, t[-1] + seg.t + dt, num = N )
    t = t + list( t_t )
    for i in range( N ):
      v.append( v[-1] + dt * a[-1] + dt*dt/2.0 * seg.j )
      a.append( a[-1] + dt * seg.j )
      j.append( seg.j )
  
  plt.ion()
  f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=False)
  plotlim( ax1, t, limits.maxVel, segs )
  ax1.plot( t, v )
  plotlim( ax2, t, limits.maxAcc, segs )
  ax2.plot( t, a )
  plotlim( ax3, t, limits.maxJerk, segs )
  ax3.plot( t, j )
  plt.draw()
  plt.pause(0.001)
  raw_input("Enter to continue.")
  plt.close(f)

limits = Limits( 1.0, 1.0, 10.0 )

c_i_s = [ \
  Conds( +2.0, +2.0, 0.0 ),
  Conds( +2.0, +0.0, 0.0 ),
  Conds( +2.0, -2.0, 0.0 ),
  
  Conds( +0.0, +2.0, 0.0 ),
  Conds( +0.0, +0.0, 0.0 ),
  Conds( +0.0, -2.0, 0.0 ),
  
  Conds( -2.0, +2.0, 0.0 ),
  Conds( -2.0, +0.0, 0.0 ),
  Conds( -2.0, -2.0, 0.0 ),
]


#for c_i in c_i_s:
while True:
  
  params = list( 100*np.random.rand( 6 ) );
  limits = Limits( *params[:3] )
  c_i = Conds( *params[3:] )

  
  segs = compute_braking( c_i, limits )
  plot_segments( c_i, limits, segs )
  ts = [seg.t for seg in segs]
  print(ts)
  print( sum(ts) )
  #strg = ""
  #for seg in segs:
    #strg = strg + " -> " + str( seg )
  #print( strg )
