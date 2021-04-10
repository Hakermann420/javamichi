
package packrun;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class Einlesen {

    //private static EV3ColorSensor colorSensor;
    private static SampleProvider sampleProvider;
    private static int sampleSize;
    
    static float[] sample = new float[3];
    

    public static float[] getSample(EV3ColorSensor colorSensor) {
    	
        // Initializes the array for holding samples
    	sampleProvider = colorSensor.getRedMode();
    	sampleSize = sampleProvider.sampleSize();
    	
        float[] sample = new float[sampleSize];

        // Gets the sample an returns it
        sampleProvider.fetchSample(sample, 0);
        return sample;
    }
    
    
    /**
     * Gibt in CapsLock eine Gelesene Farbe zurück, basierend auf dem ColorID Modus des Sensors
     * @param s Farbsensor, mit dem gelesen werden soll
     * @return "NONE", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE" oder "BROWN"
     */
    public static String getEV3Color(EV3ColorSensor s) {
    	
        // Initializes the array for holding samples
    	sampleProvider = s.getColorIDMode();
    	sampleSize = sampleProvider.sampleSize();
    	
        float[] sample = new float[sampleSize];
        
        //sampleProvider.fetchSample(sample, 0);
        
        sample[0] = s.getColorID();
        
        System.out.println(sample[0]);
        
        switch((int)sample[0]) {
        	case -1:
        		return "NONE";
        	case 0:
        		return "BLACK";
        	case 1:
        		return "BLUE";
        	case 2:
        		return "GREEN";
        	case 3:
        		return "YELLOW";
        	case 4:
        		return "RED";
        	case 5:
        		return "WHITE";
        	case 6:
        		return "BROWN";
        	default:
        		return null;
        }
    }


    /*public static String lesen() {
    	String result = "";
    	LCD.clear();
    	float[] arr = new float[3];
    	arr = getSample();
    	System.out.println(arr[0] *255 + " " + arr[1]*255 + " " + arr[2]*255);
    	if(arr[0]*255 > 60 && arr[1]*255 < 25 && arr[2]*255 < 15) {

    		LCD.drawString("Red", 5, 5);
    		System.out.println("Red");
    		return "Red";
    	}
    	else if(arr[0]*255 < 25 && arr[1]*255 > 30 && arr[2]*255 < 30) {
    		
    		LCD.drawString("Green", 5, 5);
    		System.out.println("Green");
    		return "Green";
    	}
    	else if(arr[0]*255 < 25 && arr[1]*255 < 35 && arr[2]*255 > 4) {

    		LCD.drawString("Blue", 5, 5);
    		System.out.println("Blue");
    		return "Blue";
    	}
    	else if(arr[0]*255 > 95 && arr[1]*255 > 75 && arr[2]*255 < 20) {
    		
    		LCD.drawString("Yellow", 5, 5);
    		System.out.println("Yellow");
    		return "Yellow";
    	}

    	return null;
    }*/
 // red		>65  <20  <10 | 195 93  52
 // green	<15  >25  <10 | 73  113 62
 // blue 	<15  <30  >30 | 63  80  123
 // yellow  >100 >45  <15 | 254 215 57
 // white   >120 >90  >70 | 254 254 221
 // black				  | 14  16  12
    
}
