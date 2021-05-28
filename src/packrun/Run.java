
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
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.mapping.LineMap;

public class Run{

	public static int wetter = 1;
	public static final int DEFAULT_SPEED = 500;
	private static boolean calibrate = false;								// calibrates robot instead of running main
	
	private static EV3ColorSensor sl = new EV3ColorSensor(SensorPort.S2);
	private static EV3ColorSensor sr = new EV3ColorSensor(SensorPort.S1);
	private static EV3ColorSensor sv = new EV3ColorSensor(SensorPort.S3);	//Sensor nach vorne
	private static EV3ColorSensor ss = new EV3ColorSensor(SensorPort.S4);	//Sensor zur Seite

	private static float pauselen = 1.5f;
	
	private static LineMap mapAvoidAll;
	private static LineMap mapAvoidGelb;
	private static LineMap mapAvoidGrün;
	private static LineMap mapAvoidBlau;
	
	public static void main(String[] args) throws Throwable {	//throws Throwable
		

		
		/*******************************Map Stuff Beginnt*************************************/
		
		Line[] lineall = new Line[14];
		
		Line[] linegelb = new Line[10];
		Line[] linegrün = new Line[10];
		Line[] lineblau = new Line[10];

		//Gelbe Energie Nr 1
		lineall[0] = new Line(271,9,683,337);
		lineall[1] = new Line(271,337,683,9);

		linegelb[0] = new Line(271,9,683,337);
		linegelb[1] = new Line(271,337,683,9);

		linegrün[0] = new Line(271,9,683,337);
		linegrün[1] = new Line(271,337,683,9);
		
		lineblau[0] = new Line(271,9,683,337);
		lineblau[1] = new Line(271,337,683,9);
		
		//Gelbe Energie Nr 2
		lineall[2] = new Line(937,9,1347,337);
		lineall[3] = new Line(937,337,1347,9);

		linegelb[2] = new Line(937,9,1347,337);
		linegelb[3] = new Line(937,337,1347,9);

		linegrün[2] = new Line(937,9,1347,337);
		linegrün[3] = new Line(937,337,1347,9);

		lineblau[2] = new Line(937,9,1347,337);
		lineblau[3] = new Line(937,337,1347,9);
		
		//Gelbe Zusatzenergie
		lineall[4] = new Line(282,312,660,690);
		lineall[5] = new Line(282,690,660,312);

		linegelb[4] = new Line(282,312,660,690);
		linegelb[5] = new Line(282,690,660,312);
				
		
		//Grüne Energie
		lineall[6] = new Line(626,756,1286,1115);
		lineall[7] = new Line(626,1115,1286,756);

		linegelb[6] = new Line(626,756,1286,1115);
		linegelb[7] = new Line(626,1115,1286,756);

		linegrün[4] = new Line(626,756,1286,1115);
		linegrün[5] = new Line(626,1115,1286,756);

		lineblau[4] = new Line(626,756,1286,1115);
		lineblau[5] = new Line(626,1115,1286,756);
		
		//Grüne Zusatzenergie
		lineall[8] = new Line(1045,402,1482,779);
		lineall[9] = new Line(1045,779,1482,402);

		linegrün[6] = new Line(1045,402,1482,779);
		linegrün[7] = new Line(1045,779,1482,402);
		
		//Blaue Energie
		lineall[10] = new Line(2069,172,2436,671);
		lineall[11] = new Line(2069,671,2436,172);

		linegelb[8] = new Line(2069,172,2436,671);
		linegelb[9] = new Line(2069,671,2436,172);

		linegrün[8] = new Line(2069,172,2436,671);
		linegrün[9] = new Line(2069,671,2436,172);
		
		lineblau[6] = new Line(2069,172,2436,671);
		lineblau[7] = new Line(2069,671,2436,172);

		//Blaue Zusatzenergie
		lineall[12] = new Line(1679,190,2056,567);
		lineall[13] = new Line(1679,567,2056,190);

		lineblau[8] = new Line(1679,190,2056,567);
		lineblau[9] = new Line(1679,567,2056,190);
		
		Rectangle rec = new Rectangle(0,0,2362,1143);
		
		mapAvoidAll = new LineMap(lineall, rec);
		mapAvoidGelb = new LineMap(linegelb, rec);
		mapAvoidGrün = new LineMap(linegrün, rec);
		mapAvoidBlau = new LineMap(lineblau, rec);

		/*******************************Map Stuff Vorbei*************************************/
		
		/********************************Fahrt Beginnt***************************************/
		TomTom.Init();
		
		
		if(calibrate) {
			
			TomTom.Calibrate();
			return;
		}
		
		if(wetter == 0) {
			TomTom.StartToAdditiveYellow();
			//TomTom.DriveAgainstWall(1500, -70);
			TomTom.ZusatzenergieAufnehmen(320);
		}
		else {
			TomTom.StartToYellow();
			TomTom.ZusatzenergieAufnehmen(350);
		}
		
		getPower();

		/********************************Fahrt Vorbei***************************************/
	}
	
	/**
	 * Returns the needed linemap according to the avoidNr parameter
	 * @param avoidNr is the selector which map to return:
	 * 			0: alle
	 * 			1: gelb
	 * 			2: grün
	 * 			3: blau
	 * @return the Linemap
	 */
	public static LineMap getMap(int avoidNr) {
		switch(avoidNr) {
			case 0:
				return mapAvoidGelb;
			case 1:
				return mapAvoidGrün;
			case 2:
				return mapAvoidBlau;
			default:
				return mapAvoidAll;
		}
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

