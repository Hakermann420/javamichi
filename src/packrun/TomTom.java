
package packrun;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;

public class TomTom{

	private static EV3MediumRegulatedMotor b;
	private static EV3MediumRegulatedMotor c;
	private static Wheel wheel1;
	private static Wheel wheel2;
	private static Chassis chassis;
	private static MovePilot mp;
	
	private static Navi n;//, new CompassPoseProvider(mp, new GyroDirectionFinder(new GyroscopeAdapter(g, 30))));;
	
	private static EV3ColorSensor sr = Run.getSensor("r");
	private static EV3ColorSensor sl = Run.getSensor("l");
	private static EV3ColorSensor sh = Run.getSensor("s");
	
	private static LineMap map;
	public static ShortestPathFinder spf;
	

	public static Waypoint wp1 = new Waypoint(1822,572); 		// red/yellow-line
	public static Waypoint wp2 = new Waypoint(528,285);  		// cornerlines
	public static Waypoint wp3 = new Waypoint(1181,903); 		// another position
	public static Waypoint wpR5 = new Waypoint(805,572); 		// position R5
	public static Waypoint wpGCLZ1 = new Waypoint(432,845); 	// weird place we need to go to
	public static Waypoint gh = new Waypoint(525,426);			//Position of Green House
	public static Waypoint bh = new Waypoint(000,000);			//Position of Blue House
	
	public static Waypoint tet = new Waypoint (432,845);
	public static Waypoint tet2 = new Waypoint (2362-432,1143-845);
	

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
	
	/**
	 * Dreht den Roboter 10 Mal, um die Achsenlänge kalibrieren zu können
	 */
	public static void Calibrate() {
		mp.rotate(3600);
	}
	
	/**
	 * Schließt alle Motoren, damit unregulierte verwendet werden können
	 */
	public static void Uninit() {
		
		b.close();
		c.close();
	}
	
	/**
	 * Initialisiert alle Motoren, Navigatoren, MovePilots, um sie wieder benutzen zu können
	 */
	public static void Init() {
		
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
		
		wheel1 = WheeledChassis.modelWheel(b, -62.4).offset(-88.6);;
		wheel2 = WheeledChassis.modelWheel(c, 62.4).offset(88.6);;
		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);;
		mp = new MovePilot(chassis);;
		n = new Navi(mp);;
		
	}
	
	public static Chassis getChassis() {
		return chassis;
	}
}
