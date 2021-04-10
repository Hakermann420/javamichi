	package packrun;
	
	import java.util.ArrayList;
import java.util.Arrays;

import lejos.robotics.localization.OdometryPoseProvider;
	import lejos.robotics.localization.PoseProvider;
	import lejos.robotics.navigation.ArcMoveController;
	import lejos.robotics.navigation.MoveController;
	import lejos.robotics.navigation.NavigationListener;
	import lejos.robotics.navigation.Pose;
	import lejos.robotics.navigation.RotateMoveController;
	import lejos.robotics.navigation.Waypoint;
	import lejos.robotics.pathfinding.Path;
	import lejos.robotics.geometry.Point;
	
	public class Navi {
		
		private Path _path = new Path();
		  /**
	   * frequently tested by Nav.run() to break out of primary control loop
	   * reset by stop(), and in Nav if _singleStep is set. or end of path is reached
	   * set by followPath(xx) and goTo(xx)
	   */
	  private boolean _keepGoing = false;
	  /**
	   * if true, causes Nav.run to break whenever  waypoint is reached. 
	   */
	  private boolean _singleStep = false;
	  /** 
	   * set by Stop,  reset by followPath() , goTo()
	   * used by  Nav.run(), callListeners
	   */
	  private boolean _interrupted = false;
	  private MoveController _pilot;
	  private PoseProvider poseProvider;
	  private Pose _pose = new Pose();
	  private Waypoint _destination;
	  private double _radius;
	  private int _sequenceNr;
	  private ArrayList<NavigationListener> _listeners = new ArrayList<NavigationListener>();
	
		/**
	    * Allocates a Navigator object,  using pilot that implements the ArcMoveController interface. 
	    * @param pilot
	    */
	   public Navi(MoveController pilot)
	   {
	      this(pilot, null);
	   }
	
	   /**
	    * Allocates a Navigator object using a pilot and a custom poseProvider, rather than the default
	    * OdometryPoseProvider.  
	    * @param pilot  the pilot 
	    * @param poseProvider  the custom PoseProvider
	    */
	   public Navi(MoveController pilot, PoseProvider poseProvider)
	   {
	      _pilot = pilot;
	      if (poseProvider == null)
	         this.poseProvider = new OdometryPoseProvider(_pilot);
	      else
	         this.poseProvider = poseProvider;
	      _radius = (float) (_pilot instanceof ArcMoveController ? ((ArcMoveController) _pilot).getMinRadius() : 0);
	   }
	
	   /**
	    * Sets  the PoseProvider after construction of the Navigator
	    * @param aProvider  the PoseProvider
	    */
	   public void setPoseProvider(PoseProvider aProvider)
	   {
	      poseProvider = aProvider;
	   }
	
	   /**
	    * Adds a NavigationListener that is informed when a the robot stops or 
	    * reaches a WayPoint.
	    * @param listener  the NavitationListener
	    */
	   public void addNavigationListener(NavigationListener listener)
	   {
	      _listeners.add(listener);
	   }
	
	   /**
	    * Returns the PoseProvider
	    * @return the PoseProvider
	    */
	   public PoseProvider getPoseProvider()
	   {
	      return poseProvider;
	   }
	
	   /**
	    * Returns the MoveController belonging to this object.
	    * @return the pilot
	    */
	   public MoveController getMoveController()
	   {
	      return _pilot;
	   }
	
	   /**
	    * Sets the path that the Navigator will traverse.
	    * By default, the  robot will not stop along the way.
	    * If the robot is moving when this method is called,  it stops and the current
	    * path is replaced by the new one.
	    * @param path to be followed.
	    */
	   public void setPath(Path path)
	   {
	      if (_keepGoing)
	         stop();
	      _path = path;
	      _singleStep = false;
	      _sequenceNr = 0;
	   }
	   
	   /**
	    * Clears the current path.
	    * If the robot is moving when this method is called, it stops; 
	    */
	   public void clearPath() {
		   if (_keepGoing)
		         stop();
		   _path.clear();
	   }
	   
	   /**
	    * Gets the current path
	    * 
	    * @return the path
	    */
	   public Path getPath() {
		   return _path;
	   }
	
	   /**
	    * Starts the robot traversing the path. This method is non-blocking. 
	    * NOTE!!!	The smooth following is not implemented yet
	    * @param path  to be followed.
	    * @param smooth	value if the path should be followed exactly or smoothed out
	    */
	   public void followPath(Path path, boolean smooth)
	   {
	      _path = path;
	      if(!smooth)
	      followPath();
	      else
	      followPathSmooth();
	   }
	
	   /**
	    * Starts the robot traversing the current path. 
	    * This method is non-blocking; 
	    */
	   public void followPath()
	   {  
	      if (_path.isEmpty())
	         return;
	      _interrupted = false;
	      _keepGoing = true;
	      
	      
	      while(_keepGoing) {
	    	  while(!_path.isEmpty()){
	    		  
	    		  Waypoint waypoint = _path.get(0);
	    		  
	    		  float angle = getAngleToWaypoint(waypoint);
	    		  ((RotateMoveController) _pilot).rotate(angle,false);
	    		  while(_pilot.isMoving())Thread.yield();
	    		  if(!_keepGoing) break;
	    		  _pilot.travel(poseProvider.getPose().distanceTo(new Point(waypoint.x, waypoint.y)));
	    		  while(_pilot.isMoving())Thread.yield();
	    		  if(!_keepGoing) break;
	    		  
	    		  if(_path.size() == 1) {
	    			  if(waypoint.isHeadingRequired()) {
	    				  angle = (float) (waypoint.getHeading() - poseProvider.getPose().getHeading());
	    				  ((RotateMoveController) _pilot).rotate(angle,false);
	    			  }
	    		  }
	    		  
	    		  _path.remove(waypoint);
	    		  if(_singleStep)return;
	    	  }
	    	  _keepGoing = false;
	      }
	      
	   }
	   
	   
	   public void followPathSmooth() {
		   if (_path.isEmpty())
		         return;
		      _interrupted = false;
		      _keepGoing = true;
		      
		      double x,y,r = 0;
		      x = poseProvider.getPose().getX();
		      y = poseProvider.getPose().getY();
		      while(_keepGoing) {

	    		  _path.remove(0);
		    	  while(!_path.isEmpty()){
		    		  Waypoint waypoint = _path.get(0);
		    		  
		    		  double nextx,nexty,nextr = 0;
		    		  nextx = _path.get(0).getX();
		    		  nexty = _path.get(0).getY();
		    		  nextr = _path.size() > 1 ? getDistanceBetweenWaypoints(_path.get(0), _path.get(1)) * 0.1 : 0;
		    		  double[][]arr = getTangents(x,y,r,nextx,nexty,nextr);
		    		  for(int i = 0; i<arr.length; i++) {
		    			  for(int j = 0; j<arr[i].length; j++) {
				    		  System.out.println(arr[i][j]);
		    			  }
		    		  }
		    		  //calculating the index based on the position of goal to player
		    		  double x1 = poseProvider.getPose().getX();
		    		  double y1 = poseProvider.getPose().getY();
		    		  
		    		  double x2 = x;
		    		  double y2 = y;
		    		  
		    		  double xp = nextx;
		    		  double yp = nexty;
		    		  
		    		  double d = (xp - x1)*(y2 - y1) - (yp - y1)*(x2 - x1);
		    		  int first = 0;
		    		  if(d > 0) first = 1;
		    		  
		    		  
		    		  int second = 0;
		    		  if(_path.size() > 1) {
		    			  x1 = x;
		    			  y1 = y;
		    			  
		    			  x2 = nextx;
		    			  y2 = nexty;
		    			  
		    			  xp = _path.get(1).getX();
		    			  yp = _path.get(1).getY();
		    			  
		    			  d = (xp - x1)*(y2 - y1) - (yp - y1)*(x2 - x1);
		    			  if(d > 0) second = 1;
		    		  }
		    		  
		    		  //calculate angle
		    		  int index = (first + 2 * second) - 1; // which tangent to follow
		    		  if(index < 0) index = 3;
		    		  if(index > arr.length - 1) index = arr.length - 1;
		    		  
		    		  double angle = 0;
		    		  if(r == 0) angle = getAngleToWaypoint(new Waypoint(arr[index][0],arr[index][1]));
		    		  else angle = getAngleAroundCircle(new Waypoint(x,y), new Waypoint(arr[index][0],arr[index][1]));
		    		  System.out.println(angle);
		    		  if(r == 0) ((RotateMoveController) _pilot).rotate(angle,false);
		    		  else((ArcMoveController) _pilot).arc(angle,r,false);
		    		  while(_pilot.isMoving())Thread.yield();
		    		  if(!_keepGoing) break;
		    		  
		    		  _pilot.travel(poseProvider.getPose().distanceTo(new Waypoint (arr[index][2], arr[index][3])));
		    		  while(_pilot.isMoving())Thread.yield();
		    		  if(!_keepGoing) break;
		    		  
		    		  if(_path.size() == 1) {
		    			  if(waypoint.isHeadingRequired()) {
		    				  angle = waypoint.getHeading() - poseProvider.getPose().getHeading();
		    				  ((RotateMoveController) _pilot).rotate(angle,false);
		    			  }
		    		  }
		    		  
		    		  _path.remove(0);
		    		  if(_singleStep)return;
		    	  }

		    	  _keepGoing = false;
		      }
	   }
	   
	   
	   /**
	     *  Finds tangent segments between two given circles.
	     *
	     *  Returns an empty, or 2x4, or 4x4 array of doubles representing
	     *  the two exterior and two interior tangent segments (in that order).
	     *  If some tangents don't exist, they aren't present in the output.
	     *  Each segment is represent by a 4-tuple x1,y1,x2,y2.
	     * 
	     *  Exterior tangents exist iff one of the circles doesn't contain
	     *  the other. Interior tangents exist iff circles don't intersect.
	     *
	     *  In the limiting case when circles touch from outside/inside, there are
	     *  no interior/exterior tangents, respectively, but just one common
	     *  tangent line (which isn't returned at all, or returned as two very
	     *  close or equal points by this code, depending on roundoff -- sorry!)
	     */
	    public static double[][] getTangents(double x1, double y1, double r1, double x2, double y2, double r2) {
	        double d_sq = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	        if (d_sq <= (r1-r2)*(r1-r2)) return new double[0][4];

	        double d = Math.sqrt(d_sq);
	        double vx = (x2 - x1) / d;
	        double vy = (y2 - y1) / d;

	        double[][] res = new double[4][4];
	        int i = 0;

	        // Let A, B be the centers, and C, D be points at which the tangent
	        // touches first and second circle, and n be the normal vector to it.
	        //
	        // We have the system:
	        //   n * n = 1          (n is a unit vector)          
	        //   C = A + r1 * n
	        //   D = B +/- r2 * n
	        //   n * CD = 0         (common orthogonality)
	        //
	        // n * CD = n * (AB +/- r2*n - r1*n) = AB*n - (r1 -/+ r2) = 0,  <=>
	        // AB * n = (r1 -/+ r2), <=>
	        // v * n = (r1 -/+ r2) / d,  where v = AB/|AB| = AB/d
	        // This is a linear equation in unknown vector n.

	        for (int sign1 = +1; sign1 >= -1; sign1 -= 2) {
	            double c = (r1 - sign1 * r2) / d;

	            // Now we're just intersecting a line with a circle: v*n=c, n*n=1

	            if (c*c > 1.0) continue;
	            double h = Math.sqrt(Math.max(0.0, 1.0 - c*c));

	            for (int sign2 = +1; sign2 >= -1; sign2 -= 2) {
	                double nx = vx * c - sign2 * h * vy;
	                double ny = vy * c + sign2 * h * vx;

	                double[] a = res[i++];
	                a[0] = x1 + r1 * nx;
	                a[1] = y1 + r1 * ny;
	                a[2] = x2 + sign1 * r2 * nx;
	                a[3] = y2 + sign1 * r2 * ny;
	            }
	        }
	        
	        return Arrays.copyOf(res, i);
	    }
	   
	    private double getDistanceBetweenWaypoints(Waypoint a, Waypoint b) {
	    	double diffx = b.getX() - a.getX();
	    	double diffy = b.getY() - a.getY();
	    	return Math.sqrt(Math.pow(diffx, 2) + Math.pow(diffy, 2));
	    }
	   
	
	   /**
	    * Calculates the angle, the robot needs to turn to face the waypoint
	    * This fixed the weird Navigator issue
	    * @return the angle the robot needs to turn
	    */
	   public float getAngleToWaypoint(Waypoint w) {
		   float diffx = w.x - poseProvider.getPose().getX();
		   float diffy = w.y - poseProvider.getPose().getY();
		   float angle = (float) Math.toDegrees(Math.atan(diffy / diffx));
		   if(diffx < 0) angle+=180;
		   return angle - poseProvider.getPose().getHeading();
	   }
	   
	   /**
	    * Calculates the angle
	    * This fixed the weird Navigator issue
	    * @return the angle the robot needs to turn
	    */
	   public double getAngleAroundCircle(Waypoint center, Waypoint w) {
		   double diffx = w.x - center.getX();
		   double diffy = w.y - center.getY();
		   double angle = Math.toDegrees(Math.atan(diffy / diffx));
		   if(diffx < 0) angle+=180;
		   
		   diffx = center.getX() - poseProvider.getPose().getX();
		   diffy = center.getY() - poseProvider.getPose().getY();
		   double angleToCenter = Math.toDegrees(Math.atan(diffy / diffx));
		   if(diffx < 0) angleToCenter+=180;
		   
		   return angle - angleToCenter;
	   }
	
	   /**
	    * Controls whether the robot stops at each Waypoint; applies to the current path only.
	    * The robot will move to the next Waypoint if you call {@link #followPath()}.
	    * @param yes  if <code>true </code>, the robot stops at each Waypoint.  
	    */
	   public void singleStep(boolean yes)
	   {
	      _singleStep = yes;
	   }
	
	   /**
	    * Starts the robot moving toward the destination.
	    * If no path exists, a new one is created consisting of the destination,
	    * otherwise the destination is added to the path.  This method is non-blocking, and is 
	    * equivalent to <code>{@linkplain #addWaypoint(Waypoint) addWaypoint(destination);}
	    * {@linkplain #followPath() followPath();}</code>
	    * @param destination  the waypoint to be reached
	    */
	   public void goTo(Waypoint destination)
	   {
	      addWaypoint(destination);
	      followPath();
	 
	   }
	
	   /**
	    * Starts the  moving toward the destination Waypoint created from 
	    * the parameters. 
	    * If no path exists, a new one is created,  
	    * otherwise the new Waypoint is added to the path.  This method is non-blocking, and is 
	    * equivalent to 
	     <code>add(float x, float y);   followPath(); </code>
	    * @param x  coordinate of the destination
	    * @param y  coordinate of the destination
	    */
	   public void goTo(float x, float y)
	   {
	      goTo(new Waypoint(x, y));
	   }
	
	   /**
	    * Starts the  moving toward the destination Waypoint created from 
	    * the parameters. 
	     * If no path exists, a new one is created,  
	    * otherwise the new Waypoint is added to the path.  This method is non-blocking, and is 
	    * equivalent to 
	     <code>add(float x, float y);   followPath(); </code>
	    * @param x coordinate of the destination
	    * @param y coordinate of th destination
	    * @param heading  desired robot heading at arrival 
	    */
	   public void goTo(float x, float y, float heading)
	   {
	      goTo(new Waypoint(x, y, heading));
	   }
	
	   /**
	    * Rotates the robot to a new absolute heading. For example, rotateTo(0) will line the robot with the 
	    * x-axis, while rotateTo(90) lines it with the y-axis. If the robot is currently on the move to a
	    * coordinate, this method will not attempt to rotate and it will return false.
	    * @param angle The absolute heading to rotate the robot to. Value is 0 to 360.
	    * @return true if the rotation happened, false if the robot was moving while this method was called.
	    */
	   public boolean rotateTo(double angle) {
		   float head = getPoseProvider().getPose().getHeading();
		   double diff = angle - head;
		   while(diff > 180) diff = diff - 360;
		   while(diff < -180) diff = diff + 360;
		   if(isMoving()) return false;
		   if(_pilot instanceof RotateMoveController)
			   ((RotateMoveController) _pilot).rotate(diff,false);
		   return true;
		   
	   }
	   
	   /**
	    * Adds a  Waypoint  to the end of the path. 
	    * Call {@link #followPath()} to start moving the along the current path.
	    * 
	    * @param aWaypoint  to be added
	    */
	   public void addWaypoint(Waypoint aWaypoint)
	   {
		   if(_path.isEmpty())
			   {
			   _sequenceNr = 0;
			   _singleStep = false;
			   }
	      _path.add(aWaypoint);
	   }
	
	   /**
	    * Constructs an new Waypoint from the parameters and adds it to the end of the path.
	    * Call {@link #followPath()} to start moving the along the current path.
	    * 
	    * @param x coordinate of the waypoint
	    * @param y coordinate of the waypoint
	    */
	   public void addWaypoint(float x, float y)
	   {
	      addWaypoint(new Waypoint(x, y));
	   }
	
	   /**
	    * Constructs an new Waypoint from the parameters and adds it to the end of the path. 
	    * Call {@link #followPath()} to start moving the along the current path.
	    * 
	    * @param x coordinate of the waypoint
	    * @param y coordinate of the waypoint
	    * @param heading the heading of the robot when it reaches the waypoint
	    */
	   public void addWaypoint(float x, float y, float heading)
	   {
	      addWaypoint(new Waypoint(x, y, heading));
	   }
	
	   /**
	    * Stops the robot. 
	    * The robot will resume its path traversal if you call {@link #followPath()}.
	    */
	   public void stop()
	   {
	      _keepGoing = false;
	      _pilot.stop();
	      _interrupted = true;
	      callListeners();
	   }
	
	   /**
	    * Returns the waypoint to which the robot is presently moving.
	    * @return the waypoint ; null if the path is empty.
	    */
	   public Waypoint getWaypoint()
	   {
	      if (_path.size() <= 0)
	         return null;
	      return _path.get(0);
	   }
	
	   /** 
	    * Returns <code> true </code> if the the final waypoint has been reached 
	    * @return  <code> true </code>  if the path is completed
	    */
	   public boolean pathCompleted()
	   {
	      return _path.size() == 0;
	   }
	
	   /**
	    * Waits for the robot  to stop for any reason ;
	    * returns <code>true</code> if the robot stopped at the final Waypoint of
	    *the  path. 
	    * @return   <code> true </code>  if the path is completed
	    */
	   public boolean waitForStop()
	   {
	      while (_keepGoing)
	         Thread.yield();
	      return _path.isEmpty();
	   }
	
	   /**
	    * Returns <code>true<code> if the robot is moving toward a waypoint.
	    * @return  <code>true </code> if moving.
	    */
	   public boolean isMoving()
	   {
	      return _keepGoing;
	   }
	   
	   public void pathGenerated() {
			// Currently does nothing	
		}
	
	   private void callListeners()
	   {
	      if (_listeners != null)
	      {
	         _pose = poseProvider.getPose();
	//	            RConsole.println("listener called interrupt"+_interrupted +" done "+_path.isEmpty()+" "+_pose);
		         for (NavigationListener l : _listeners)
		            if (_interrupted)
		               l.pathInterrupted(_destination, _pose, _sequenceNr);
		            else
		            {
		               l.atWaypoint(_destination, _pose, _sequenceNr);
		               if (_path.isEmpty())
		                  l.pathComplete(_destination, _pose, _sequenceNr);
		            }
		      }
		   }
	}
