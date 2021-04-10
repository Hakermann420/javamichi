
package packrun;

import java.io.FileWriter;
import java.io.IOException;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;

public class Align {

	/**
	 * Alignt der Roboter mit einer Linie
	 * <br>
	 * Muss evtl gefixt werden, wenn sich die Richtung bzw die Invertierung der Motoren des Roboters ändert
	 * @author Marcel
	 * 
	 * @param sl Der Sensor, der sich links am Roboter befindet
	 * @param sr Der Sensor, der sich rechts am Roboter befindet
	 * @param ml Der Motor links am Roboter. Im Normalfall Motor B
	 * @param mr Der Motor rechts am Roboter. Im Normalfall Motor C
	 * @param vornehinten ist "true", wenn sich die Linie vor dem Roboter befindet.
	 */
	
	public static void Ausrichten(EV3ColorSensor sl, EV3ColorSensor sr, RegulatedMotor ml, RegulatedMotor mr, boolean vornehinten) {
		
		boolean linverted = true;
		boolean rinverted = true;
		
		if(vornehinten) {
			linverted = true;
			rinverted = false;
		}else {
			rinverted = true;
			linverted = false;
		}
		
		float valuel = 0;
		float valuer = 0;
		valuel = Einlesen.getSample(sl)[0];
		valuer = Einlesen.getSample(sr)[0];
		
		System.out.println(valuel + ", " + valuer);

		/*FileWriter fw;
		try {
			fw = new FileWriter("sensorvalue.txt");
			fw.write(valuel + ", " + valuer);
			fw.close();
		} catch (IOException e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}*/
		
		ml.setSpeed(100);
		mr.setSpeed(100);
		
		
		while(true) {
			
			if(valuel > 0.25f) {
				if(!linverted) {
					ml.forward();
				} else {
					ml.backward();
				}
			}
			
			if(valuer > 0.25f) {
				if(!rinverted) {
					mr.forward();
				} else {
					mr.backward();
				}
			}
			
			
			if(valuel < 0.4f) {
				if(!linverted) {
					ml.backward();
				} else {
					ml.forward();
				}
			}
			
			if(valuer < 0.4f) {
				if(!rinverted) {
					mr.backward();
				} else {
					mr.forward();
				}
			}
			
			if(valuel < 0.4 && valuel > 0.3 && valuer < 0.4 && valuer > 0.3) {
				ml.stop(true);
				mr.stop(true);
				System.out.println("EXIT!");
				break;
			}
			
			valuel = Einlesen.getSample(sl)[0];
			valuer = Einlesen.getSample(sr)[0];
			
		}
		
		//ml.stop();
		//mr.stop();
	}
	
	
	/**
	 * Alignt der Roboter mit einer Linie mit Unregulierten Motoren
	 * <br>
	 * Muss evtl gefixt werden, wenn sich die Richtung bzw die Invertierung der Motoren des Roboters ändert
	 * @author Marcel
	 * 
	 * @param sl Der Sensor, der sich links am Roboter befindet
	 * @param sr Der Sensor, der sich rechts am Roboter befindet
	 * @param vornehinten ist "true", wenn sich die Linie vor dem Roboter befindet.
	 */
	public static void UnreguliertesAusrichten(EV3ColorSensor sl, EV3ColorSensor sr, boolean vornehinten) {
		
		float valuel = 0;
		float valuer = 0;
		float grayval = 0.4f;
		valuel = Einlesen.getSample(sl)[0];
		valuer = Einlesen.getSample(sr)[0];
		
		TomTom.Uninit();
		
		UnregulatedMotor b = new UnregulatedMotor(MotorPort.B);
		UnregulatedMotor c = new UnregulatedMotor(MotorPort.C);
		
		b.setPower(20);
		c.setPower(20);
		
		if(!vornehinten) {
			b.backward();
			c.forward();
		}else {
			b.forward();
			c.backward();
		}
		
		while(true) {

			valuel = Einlesen.getSample(sl)[0];
			valuer = Einlesen.getSample(sr)[0];
			
			System.out.println((int) ((grayval-valuel)*(grayval-valuel)*(grayval-valuel)/Math.abs(grayval-valuel)*150));
			
			b.setPower((int) ((grayval-valuel)*(grayval-valuel)*(grayval-valuel)/Math.abs(grayval-valuel)*150));
			c.setPower((int) ((grayval-valuer)*(grayval-valuer)*(grayval-valuer)/Math.abs(grayval-valuer)*150));
			
			if(valuel > grayval+0.08 && valuel < grayval-0.08 && valuer > grayval+0.08 && valuer < grayval-0.08) {
				break;
			}
		}
	}

}
