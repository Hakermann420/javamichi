
package packrun;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;
import lejos.hardware.Button;

public class TomTom{

	private static EV3MediumRegulatedMotor b;
	private static EV3MediumRegulatedMotor c;
	private static Wheel wheel1;
	private static Wheel wheel2;
	private static Chassis chassis;
	private static MovePilot mp;
	
	private static Navi n;//, new CompassPoseProvider(mp, new GyroDirectionFinder(new GyroscopeAdapter(g, 30))));;
	
	
	

	
	/****************************Waypoint Stuff Beginnt**********************************/
	
	
	private static LineMap map = Run.getMap(Run.wetter);
	public static ShortestPathFinder spf = new ShortestPathFinder(map);
	
	public static Waypoint start = new Waypoint(35,133,90);					// Startposition
	
	public static Waypoint wZusatzGelbLaden = new Waypoint(105, 550);		// Einladeanfahrt für gelbe Zusatzenergie
	public static Waypoint wGelbLaden0 = new Waypoint(484, 420);			// Linker bzw erster Anfahrtspunkt für normale Energie
	public static Waypoint wGelbLaden1 = new Waypoint(1135, 420);			// Rechter bzw zweiter Anfahrtspunkt für normale Energie

	public static Waypoint kHaus0 = new Waypoint(350, 750);					// Kreuzung von "hauptlinie" zum ersten Haus
	public static Waypoint haus0 = new Waypoint(350, 975);					// Abladepunkt des ersten Hauses




	public static Waypoint tet = new Waypoint (432,845);			// ich weis nicht wozu die waren, aber ich habe sie aus dem alten projekt übernommen
	public static Waypoint tet2 = new Waypoint (2362-432,1143-845);	// ich weis nicht wozu die waren, aber ich habe sie aus dem alten projekt übernommen
	

	
	/****************************Waypoint Stuff Vorbei**********************************/
	

	public static void TimeTest() throws Throwable{
		
		chassis.setVelocity(chassis.getMaxLinearSpeed(), chassis.getMaxAngularSpeed());
		
		chassis.travel(1500);
		chassis.waitComplete();
		chassis.rotate(180);
		chassis.waitComplete();
		chassis.travel(1500);
		chassis.waitComplete();
		chassis.rotate(180);
		chassis.waitComplete();

		/*Uninit();
		UnregulatedMotor b = new UnregulatedMotor(MotorPort.B);
		UnregulatedMotor c = new UnregulatedMotor(MotorPort.C);
		
		b.setPower(100);
		c.setPower(100);
		
		c.resetTachoCount();
		b.backward();
		c.forward();
		
		while(c.getTachoCount()<(1500*Math.PI/62.4)*36) {
			System.out.println((1500*Math.PI/62.4)*36);
			System.out.println(c.getTachoCount());
		}
		b.stop();
		c.stop();
		
		b.resetTachoCount();
		b.forward();
		c.forward();
		
		while(b.getTachoCount()<(88.6*2*Math.PI)/(62.4*2)*90) {
			
		}
		b.stop();
		c.stop();
		
		
		c.resetTachoCount();
		b.backward();
		c.forward();
		
		while(c.getTachoCount()<(1500*Math.PI/62.4)*36) {
			System.out.println(c.getTachoCount());
		}
		b.stop();
		c.stop();
		
		b.resetTachoCount();
		b.forward();
		c.forward();	
		
		while(b.getTachoCount()<(88.6*2*Math.PI)/(62.4*2)*90) {
			
		}
		b.stop();
		c.stop();
		*/
	}
	
	public static void StartToAdditiveYellow() throws Throwable{
		n.getPoseProvider().setPose(start.getPose());

		Path p = spf.findRoute(n.getPoseProvider().getPose(), wZusatzGelbLaden);
		Run.Wait();

		System.out.println("Betriebsbereit!");
		Button.waitForAnyPress();

		n.followPath(p, false);
		while(!n.pathCompleted()){
			continue;
		}
		
		n.rotateTo(0);
	}
	
	public static void StartToYellow() throws Throwable{
		n.getPoseProvider().setPose(start.getPose());

		Path p = spf.findRoute(n.getPoseProvider().getPose(), wGelbLaden0);
		Run.Wait();

		System.out.println("Betriebsbereit!");
		Button.waitForAnyPress();

		n.followPath(p, false);
		while(!n.pathCompleted()){
			continue;
		}
		
		n.rotateTo(180);
	}
	
	public static void ZusatzenergieAufnehmen(int distance) {
		StapelGabler.Up();
		Greifer.Up();
		
		//mp.travel(distance);
		UnregulatedDriving.Init();
		UnregulatedDriving.StraightDrive(distance);
		Greifer.Down();
	}
	
	/**
	 * Drives Robot backwards/forwards against wall
	 * @param duration The duration in milliseconds for how long to drive against wall
	 * @param speed Speed in robot values for how fast to drive against wall
	 */
	public static void DriveAgainstWall(float duration, int speed) {
		
		UnregulatedDriving.Init();
		
		long start = System.currentTimeMillis();
		UnregulatedDriving.drive(speed, speed);
		while(System.currentTimeMillis() - start < duration);
		
		UnregulatedDriving.drive(0,0);
		
		Init();
	}
	
	
	/**
	 * Dreht den Roboter 10 Mal, um die Achsenlänge kalibrieren zu können
	 */
	public static void Calibrate() {
		Button.waitForAnyPress();
		mp.travel(1000);
		mp.stop();
		Button.waitForAnyPress();
		mp.rotate(3600);
		mp.stop();
	}
	
	/**
	 * Schließt alle Motoren, damit unregulierte verwendet werden können
	 */
	public static void Uninit() {
		
		b.close();
		c.close();
		
		StapelGabler.Uninit();
	}
	
	/**
	 * Initialisiert alle Motoren, Navigatoren, MovePilots, um sie wieder benutzen zu können
	 */
	public static void Init() {
		
		UnregulatedDriving.Uninit();
		
		try{
			b = new EV3MediumRegulatedMotor(MotorPort.B);
		}catch(Exception e) {
			e.printStackTrace();
		}
		try{
			c = new EV3MediumRegulatedMotor(MotorPort.C);
		}catch(Exception e) {
			e.printStackTrace();
		}
		
		wheel1 = WheeledChassis.modelWheel(b, -63.5).offset(-123.5);;
		wheel2 = WheeledChassis.modelWheel(c, 63.5).offset(123.5);;
		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);;
		mp = new MovePilot(chassis);;
		n = new Navi(mp);;
		
		mp.setLinearSpeed(Run.DEFAULT_SPEED);
		mp.setAngularSpeed(Run.DEFAULT_SPEED);
		
		StapelGabler.Init();
		Greifer.Init();
	}
	
	public static Chassis getChassis() {
		return chassis;
	}
}
