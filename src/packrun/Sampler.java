package packrun;

import lejos.robotics.SampleProvider;

class Sampler{
	
	public static float[] getSample(SampleProvider sampleProvider) {
		int sampleSize = sampleProvider.sampleSize();
        // Initializes the array for holding samples
        float[] sample = new float[sampleSize];

        // Gets the sample an returns it
        sampleProvider.fetchSample(sample, 0);
        return sample;
    }
}
