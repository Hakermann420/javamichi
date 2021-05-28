package packrun;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;


/**
 * 
 * @author Hakermann
 * Klasse, um den Gabelstapler am Roboter zu kontrollieren,
 * kann den Gabelstapler hoch und runter fahren
 *
 */
public class StapelGabler {
	public static EV3LargeRegulatedMotor motor;
	
	
	/**
	 * Initializes the motor
	 */
	public static void Init() {
		if(motor == null) motor = new EV3LargeRegulatedMotor(MotorPort.D);
	}
	
	public static void Uninit() {
		if(motor == null) motor.close();
	}
	
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
