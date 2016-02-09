package ev3navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

//this code would be better commented but there's nothing really to say, everything here is a formality besides the getter at the bottom
public class UltrasonicPoller extends Thread{
	
	//constants
	private static final int PERIOD = 15;
	private static final int FILTER_TO = 45;
	
	//necessary resources
	private static Port portUS = LocalEV3.get().getPort("S2");
	private static EV3UltrasonicSensor ussensor = new EV3UltrasonicSensor(portUS);
	private static SampleProvider ussp = ussensor.getDistanceMode();
	
	//member variables
	private static float[] ussample = new float[ussensor.sampleSize()];
	private static int currentvalue;
	
	//default constructor
	public UltrasonicPoller() {
		
	}
	
	//the run loop
	@Override
	public void run() {
		while(true) {
			
			ussp.fetchSample(ussample, 0);
			currentvalue = (int)(ussample[0]*100);		
			filterCurrentValue(); //unnecessary but comforting/filter filter
			
			try {
				Thread.sleep(PERIOD);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	//ensures that the values stored are reasonable
	private void filterCurrentValue() {
		//very simple filter, just keeps all data less than or equal to 100
		if (currentvalue > FILTER_TO) {currentvalue = FILTER_TO;}
	}

	public int getUSdistance() {
		return currentvalue;
	}
}
