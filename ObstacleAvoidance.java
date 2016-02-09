package ev3navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


//basic implementation of P-Controller
public class ObstacleAvoidance extends Thread{
	
	//constants
	private static final int PERIOD = 20;
	private static final double PROPORTIONAL_CONSTANT = 1.4;
	private static final double BAND_CENTER = 15;
	private static final double BAND_MARGIN = 1.7;
	private static final int ROTATE_SPEED = 60;
	private static final int FORWARD_SPEED = 120;
	private static final double WR = 2.125; //wheel radius
	private static final double WB = 15.5;	//distance between wheels
	private static final double WBR = WB/2; //half distance between wheels (effective radius of rotation)
	private static final double ANGULAR_MARGIN = Math.PI * 0.05;
	
	//resources
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static UltrasonicPoller usPoller;
	private static Odometer odometer;
	private static Navigator nav;
	
	//member variables
	private static double finalTheta;
	
	//public variables
	public boolean resolved;
	
	
//CONSTRUCTOR
//
	public ObstacleAvoidance(Navigator nav) {
		this.nav = nav;
		this.leftMotor = nav.leftMotor;
		this.rightMotor = nav.rightMotor;
		this.usPoller = nav.usPoller;
		this.odometer = nav.odometer;
			
	}

//THE RUN LOOP
//
	@Override
	public void run() {
		
		turnBy90();
		sensorMotor.setSpeed(ROTATE_SPEED);
		sensorMotor.rotate(-90, false);
		finalTheta = oppositeTheta(this.odometer.getTheta());
		
		runloop:
		while (true) {
			
			leftMotor.setSpeed(FORWARD_SPEED - (int)(PROPORTIONAL_CONSTANT * amountOutOfRange(usPoller.getUSdistance(), BAND_CENTER, BAND_MARGIN)));
			rightMotor.setSpeed(FORWARD_SPEED + (int)(PROPORTIONAL_CONSTANT * amountOutOfRange(usPoller.getUSdistance(), BAND_CENTER, BAND_MARGIN)));
			leftMotor.forward();
			rightMotor.forward();
			
			if(withinRange(odometer.getTheta(), finalTheta, ANGULAR_MARGIN)) {
				resolved = true;
				leftMotor.stop();
				rightMotor.stop();
				sensorMotor.rotate(90,false);
				break runloop;
			}
			
			try {
				Thread.sleep(PERIOD);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	
	}
	
	
//HELPER METHODS
//
	//returns theta 180 degrees different from argument. result is positive and less than 2PI
	private static double oppositeTheta(double t) {
		//t is always greater than 0 so add PI and mod by 2PI for opposite theta
		return (t+Math.PI)%(Math.PI*2);
	}
	
	//true if test is within the range [testAgainst - margin, testAgainst + margin]
	private static boolean withinRange(double test, double testAgainst, double margin) {
		return (test > testAgainst - margin) && (test < testAgainst + margin);
	}
	
	//rotates robot by diffTheta degrees
	public static void turnBy90() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		int wheelangle = (int)rad2deg(((WBR/WR) * (Math.PI/2)));
		
		leftMotor.rotate(wheelangle, true); //this statement doesn't stop the flow of execution so that the other wheel can also be activated
		rightMotor.rotate(-wheelangle, false); //this statement does stop the flow of execution so that the program waits for the robot to finish rotating
		
	}
	
	//converts an angle in radians to an angle in degrees
	public static double rad2deg(double t) {
		return (t/(2*Math.PI))*360;
	}
	
	//given test and the range [testAgainst - margin, testAgainst + margin], this function
	//returns the difference between test and the closest extreme of the range if test is outside the range
	//otherwise it returns 0. 
	//test < lower_limit_of_range --> result < 0 and vice versa
	private static double amountOutOfRange(double test, double testAgainst, double margin) {
		double result;
		if ((result = (testAgainst - margin)) - test > 0) {
			return test;
		} else if ((result = (testAgainst + margin) - test) < 0) {
			return test;
		} else {
			return 0;
		}
	}
}

