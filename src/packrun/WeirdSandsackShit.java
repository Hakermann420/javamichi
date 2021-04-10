
package packrun;


import java.util.concurrent.TimeUnit;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;

public class WeirdSandsackShit {
	//CraneD

	private static EV3MediumRegulatedMotor m = new EV3MediumRegulatedMotor(MotorPort.D);
	
	/**
	 * Bewegt dern Greifer so, dass er die Sandsäcke aufnimmt
	 * @param c Ein Chassis, der benutzt werden kann (z.B. der in TomTom)
	 * @throws Throwable throws Throwable
	 */
	public static void Aufnehmen(Chassis c) throws Throwable {
		m.setSpeed(150);
		m.forward();
		TimeUnit.MILLISECONDS.sleep(500);
		m.stop();
		
		OnDeg(-30);
		TimeUnit.MILLISECONDS.sleep(500);
		m.setSpeed(250);						//Vorwärtsdrehen zum Aufnehmen
		OnDeg(-90);
		TimeUnit.SECONDS.sleep(2);
		
	}

	
	/**
	 * Fährt so, dass er die Sandsäcke vor sich in die Vorrichtungen reinlegt
	 * @param c Ein Chassis, der benutzt werden kann (z.B. der in TomTom)
	 * @throws InterruptedException
	 */
	public static void Ablegen(Chassis c) throws InterruptedException { 

		c.arc(-50, 18);
		while(c.isMoving()) {
			//wait
		}


		m.setSpeed(300);
		
		OnDeg(64);								//Rückwärts bis ein Sandsack runter fället
		TimeUnit.MILLISECONDS.sleep(500);
		
		m.setSpeed(250);
		OnDeg(-50);								//Wieder Hoch
		//TimeUnit.SECONDS.sleep(0);
		
		c.setSpeed(100, 50);
		c.arc(-50, -35);						//Drehen, um den 2. abzulegen
		while(c.isMoving()) {
			//wait
		}
		
		m.setSpeed(300);
		OnDeg(60);								//Rückwärts bis letzter Sandsack runter fället
		m.forward();
		TimeUnit.MILLISECONDS.sleep(500);
		m.stop();
		
		m.setSpeed(100);
		OnDeg(-150);						
		
		c.arc(-50, 15);							//Vorwärts fahren um sandsäcke in vorrichtung
		while(c.isMoving()) {
			//wait
		}
		
		
		c.travel(-300);
		while(c.isMoving()) {
			//wait
		}
		TimeUnit.MILLISECONDS.sleep(500);		//Greifer runter für rückwärts fahren
	}
	
	/**
	 * Dreht den Greifer um eine bestimmte Gradzahl
	 * @param deg Die Gradzahl, die gedreht werden soll
	 * @throws InterruptedException
	 */
	private static void OnDeg(int deg) throws InterruptedException {
		
		m.resetTachoCount();
		if(deg<0) {
			m.backward();	
		}
		else {
			m.forward();
		}
		while(Math.abs(m.getTachoCount()) < Math.abs(deg)) {
			//wait
		}
		m.stop();
		return;
	}

}
