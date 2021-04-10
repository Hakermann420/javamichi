package packrun;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.Gyroscope;
import lejos.robotics.SampleProvider;

public class GyroPoseProvider implements Gyroscope{
	
	public static EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S4);
	
	public static float[] getSample() {
		
		SampleProvider sampleProvider = gyro.getAngleAndRateMode();
    	int sampleSize = sampleProvider.sampleSize();
    	
        float[] sample = new float[sampleSize];

        // Gets the sample an returns it
        sampleProvider.fetchSample(sample, 0);
		
		return sample;
	}

	@Override
	public float getAngularVelocity() {
		float[] arr = getSample();
		return (int)arr[1];
	}

	@Override
	public void recalibrateOffset() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int getAngle() {
		float[] arr = getSample();
		return (int)arr[0];
	}

	@Override
	public void reset() {
		gyro.reset();
	}
}
