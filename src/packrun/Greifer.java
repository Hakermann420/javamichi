package packrun;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;


/** 
 * @author Hakermann
 * Klasse, um den Greifer am Roboter zu kontrollieren,
 * kann den Greifer hoch und runter fahren
 *
 */

public class Greifer {
	
	public static EV3LargeRegulatedMotor motor = new EV3LargeRegulatedMotor(MotorPort.A);
	
	/**
	 * Bewegt den Gabelstapler nach oben
	 */
	public static void Up() {
		motor.rotate(-40);
	}
	
	/**
	 * Bewegt den Gabelstapler nach unten
	 */
	public static void Down() {
		motor.rotate(40);
	}
}
