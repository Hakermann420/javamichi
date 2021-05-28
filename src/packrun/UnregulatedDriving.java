package packrun;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;

public class UnregulatedDriving {
	
	

	public static float[] COLOUR_VALUES = { 0.102f, 0.160f, 0.312f, 0.507f, 0.582f }; // Ich kenne den Zweck auch nicht, aber der dritte Wert wird für den PID Regler benutzt

	
	public static UnregulatedMotor b;
	public static UnregulatedMotor c;
	
	public static void Init() {
		TomTom.Uninit();
		b = new UnregulatedMotor(MotorPort.B);
		c = new UnregulatedMotor(MotorPort.C);
	}
	
	public static void Uninit() {
		if(b != null) b.close();
		if(c != null) c.close();
	}
	
	public static void drive(int l, int r) {
		// B-> to left C-> to right

		/*Motor.B.setSpeed(Math.abs(l));
		Motor.C.setSpeed(Math.abs(r));
		if (l > 0) {
			Motor.B.backward();
		} else if (l < 0) {
			Motor.B.forward();
		} else {
			Motor.B.stop(true);
		}

		if (r > 0) {
			Motor.C.forward();
		} else if (r < 0) {
			Motor.C.backward();
		} else {
			Motor.C.stop(true);
		}*/
		
		b.setPower(Math.abs(l));
		c.setPower(Math.abs(r));
		if (l > 0) {
			b.backward();
		} else if (l < 0) {
			b.forward();
		} else {
			b.stop();
		}

		if (r > 0) {
			c.forward();
		} else if (r < 0) {
			c.backward();
		} else {
			c.stop();
		}
	}
	
	
	
	public static void StraightDrive(int deg) {
		
		b.resetTachoCount();
		c.resetTachoCount();
		
		b.setPower(100);
		c.setPower(100);
		
		b.backward();
		c.forward();
		
		while(true) {
			if(b.isMoving() && c.isMoving()) {
				if(Math.abs(b.getTachoCount())>Math.abs(c.getTachoCount())){
					if(c.getPower()<95) {
						c.setPower(c.getPower()+1);
					}else {
						b.setPower(b.getPower()-1);
					}
				}
				else if(Math.abs(b.getTachoCount())<Math.abs(c.getTachoCount())){
					if(b.getPower()<95) {
						b.setPower(b.getPower()+1);
					}else {
						c.setPower(c.getPower()-1);
					}
				}
			}
			if(Math.abs(b.getTachoCount())>=deg) {
				b.stop();
			}
			if(Math.abs(c.getTachoCount())>=deg) {
				c.stop();
			}
			if(!b.isMoving()&&!c.isMoving()) {
				break;
			}
		}
	}
	
	
	
	/**
	 * Fährt extrem Gerade auf eine bestimmte Distanz
	 * <p>
	 * Merke: Bei Hardwareänderungen muss evtl der Raddurchmesser angepasst werden
	 * @param mm zu fahrende Distanz in mm
	 * @author Marcel
	 */
	public static void StraightDriveDistance(int mm) {
		//Grad = Distanz/umfangReifen / 360 == Distanz*360/umfangReifen
		int deg = (int) (360*mm/(62.4 * Math.PI));
		//deg = 10000000;
		@SuppressWarnings("resource")
		UnregulatedMotor b = new UnregulatedMotor(MotorPort.B);
		@SuppressWarnings("resource")
		UnregulatedMotor c = new UnregulatedMotor(MotorPort.C);
		
		float acc = 0;
		
		b.resetTachoCount();
		c.resetTachoCount();
		
		b.setPower((int) acc);
		c.setPower((int) acc);
		
		b.backward();
		c.forward();
		
		while(true) {
			if(acc<96) {
				acc+=4;
			}
			if(b.isMoving() && c.isMoving()) {
				int diff = Math.abs(b.getTachoCount())-Math.abs(c.getTachoCount());
				System.out.println(diff);
				b.setPower((int) Math.min(acc-diff, 100));
				c.setPower((int) Math.min(acc+diff, 100));
			}
			if(Math.abs(b.getTachoCount())>=deg) {
				b.stop();
			}
			if(Math.abs(c.getTachoCount())>=deg) {
				c.stop();
			}
			if(!b.isMoving()&&!c.isMoving()) {
				break;
			}
		}
		System.out.println(deg);
		System.out.println(b.getTachoCount());
	}
}
