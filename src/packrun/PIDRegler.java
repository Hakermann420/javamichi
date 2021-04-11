package packrun;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

class PIDRegler { 
	public static EV3ColorSensor cs = new EV3ColorSensor(SensorPort.S2);
	final float TARGET = UnregulatedDriving.COLOUR_VALUES[2];
	final float P = 350;
	final float I = 0;
	final float D = 0;
	final float BASE_SPEED = 700; 
	float leftSpeed, rightSpeed; 
	float[] sample;
	float integral = 0;
	float lastErr = 0; 
	float deriv = 0; 
	
	public void run(int degree) {
		
		Motor.C.resetTachoCount();
		
		while(true) {
			
			sample = Sampler.getSample(cs.getRedMode());
			float err = TARGET - sample[0];
		
			integral *= 0.98; 
			integral += err;
			deriv = err - lastErr; 
			lastErr = err; 
		
			leftSpeed = BASE_SPEED + P * err + I * integral + D * deriv; 
			rightSpeed = BASE_SPEED - (P * err + I * integral + D * deriv);
		
			UnregulatedDriving.drive(leftSpeed, rightSpeed);
			if(Motor.C.getTachoCount() >= degree) return;
			if(Button.ESCAPE.isDown()) return;
		}
	}
}
