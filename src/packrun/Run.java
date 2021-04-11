
/**
 * @author Michisoft Inc.
 *
 */
package packrun;

import java.util.concurrent.TimeUnit;

import lejos.hardware.Power;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.mapping.LineMap;

public class Run{
	
	
	private static EV3ColorSensor sl = new EV3ColorSensor(SensorPort.S2);
	private static EV3ColorSensor sr = new EV3ColorSensor(SensorPort.S1);
	private static EV3ColorSensor sv = new EV3ColorSensor(SensorPort.S3); //Sensor nach vorne
	private static EV3ColorSensor ss = new EV3ColorSensor(SensorPort.S4); //Sensor zur Seite

	private static float pauselen = 0.5f;
	
	private static LineMap linemap;
	
	public static void main(String[] args) throws Throwable {	//throws Throwable
		
		Line[] line = new Line[14];

		//Gelbe Energie Nr 1
		line[0] = new Line(271,9,683,337);
		line[1] = new Line(271,337,683,9);
		
		//Gelbe Energie Nr 2
		line[2] = new Line(937,9,1347,337);
		line[3] = new Line(937,337,1347,9);
		
		//Gelbe Zusatzenergie
		line[4] = new Line(282,312,660,690);
		line[5] = new Line(282,690,660,312);
				
		
		//Grüne Energie
		line[6] = new Line(626,756,1286,1115);
		line[7] = new Line(626,1115,1286,756);

		//Grüne Zusatzenergie
		line[8] = new Line(1045,402,1482,779);
		line[9] = new Line(1045,779,1482,402);
		
		
		//Blaue Energie
		line[10] = new Line(2069,172,2436,671);
		line[11] = new Line(2069,671,2436,172);

		//Blaue Zusatzenergie
		line[12] = new Line(1679,190,2056,567);
		line[13] = new Line(1679,567,2056,190);
				
		
		Rectangle rec = new Rectangle(0,0,2362,1143);
		linemap = new LineMap(line, rec);

		
		
		TomTom.Init();
		
		getPower();
	}
	
	public static LineMap getMap() {
		return linemap;
	}
	
	/**
	 * Gibt aus und returnt Batterievolt
	 * @return Volt, die am Brick anliegen
	 */
	public static float getPower() {
		Power p = LocalEV3.ev3.getPower();
		System.out.println(p.getVoltage());
		return p.getVoltage();
	}
	
	/**
	 * Wartet die länge der in Run deklarierten Variable (float) in Sekunden
	 * @throws InterruptedException
	 */
	public static void Wait() throws InterruptedException {
		TimeUnit.MILLISECONDS.sleep((long)(pauselen * 1000));
	}
	
	/**
	 * Gibt Farbsensor zurück
	 * @param s Variiert Farbsensor (z.B. "r" den rechten Sensor, "l", den linken, "s" oder "h" für den Sensor am Greifer (Sandsack),etc...)
	 * @return
	 */
	public static EV3ColorSensor getSensor(String s) {
		if(s=="r") {
			return sr;
		}
		if(s=="l") {
			return sl;
		}
		if(s=="v") {
			return sv;
		}
		if(s=="s") {
			return ss;
		}
		return null;
	}
}

/*
 * 0,0,2362,1143
 * 
 * 810,0,810,520:
 *
 * 	710,0,710,520
 * 	910,0,910,520
 * 	710,520,910,520
 * 
 * 1552,0,1552,520:
 * 
 * 	1452,0,1452,520
 * 	1652,0,1652,520
 * 	1452,520,1652,520
 * 
 * 810,623,810,1143:
 * 
 * 	710,623,710,1143
 * 	910,623,910,1143
 * 	710,623,910,623
 * 
 * 1552,623,1552,1143:
 *	
 * 	1452,623,1452,1143
 * 	1652,623,1652,1143
 * 	1452,623,1652,623
 */

