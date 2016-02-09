// Lab2.java

package ev3navigation;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab3NAV {
	
	// Static Resources:
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C")); //IMPORTANT: NOTE
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D")); //PORTS FOR MOTORS
	private static final UltrasonicPoller usPoller = new UltrasonicPoller();
	

	private static Odometer odometer = new Odometer(leftMotor,rightMotor);
	private static Navigator nav = new Navigator(odometer, leftMotor, rightMotor, usPoller);
	
	// Constants
	public static final double WHEEL_RADIUS = 2.125;
	public static final double TRACK = 15.5;

	public static void main(String[] args) {
		int buttonChoice;

		// some objects that need to be instantiated
		
		
		
		final TextLCD t = LocalEV3.get().getTextLCD();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" Float | Navig- ", 0, 2);
			t.drawString("motors | ate to ", 0, 3);
			t.drawString("       | points ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			
			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();
			
			odometer.start();
			odometryDisplay.start();
			
		} else {
			// start the odometer, the odometry display, the ultrasonic poller and the navigator			
			odometer.start();
			odometryDisplay.start();
			usPoller.start();
			nav.start();
			
			// spawn a new Thread to avoid this loop from blocking
			(new Thread() {
				public void run() {
					double[][] waypoints = {{0,60},{60,0}};
					for (double[] point : waypoints) {
						nav.setDestAndNavigate(point[0],point[1]);
						while (nav.isNavigating()) {
							try {
								sleep(500);
							} catch (InterruptedException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}
					}
				}
			}).start();
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}