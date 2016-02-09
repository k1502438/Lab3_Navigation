/*
 * Odometer.java
 */

package ev3navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class Odometer extends Thread {
	
	//Class Constants
	public static final int SINTERVAL=50;	// Period of sampling f (mSec)
	public static final int SLEEPINT=500;	// Period of display update (mSec)
	public static final double WB=15.5;		// Wheelbase (cm)
	public static final double WR=2.125;		// Wheel radius (cm)
	public static final double TWOPI = Math.PI*2;
	
	//Class Variables
	public static int lastTachoL;			// Tacho L at last sample
	public static int lastTachoR;			// Tacho R at last sample 
	public static int nowTachoL;			// Current tacho L
	public static int nowTachoR;			// Current tacho R
	public static double distL, distR, deltaD, deltaT, dX, dY;
	
	//robot position
	public static double X;					// Current X position
	public static double Y;					// Current Y position
	public static double Theta;				// Current orientation
	
	//resources
	private static EV3LargeRegulatedMotor rightMotor;
	private static EV3LargeRegulatedMotor leftMotor;
	
//	// robot position
//	private double x, y, theta;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 15;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		X = 0.0;
		Y = 0.0;
		Theta = 0.0;
		lock = new Object();
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		leftMotor.resetTachoCount();
	    rightMotor.resetTachoCount();
	    lastTachoL=leftMotor.getTachoCount();
	    lastTachoR=rightMotor.getTachoCount();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			nowTachoL = leftMotor.getTachoCount();      		// get tacho counts
			nowTachoR = rightMotor.getTachoCount();
			distL = 3.14159*WR*(nowTachoL-lastTachoL)/180;		// compute L and R wheel displacements
			distR = 3.14159*WR*(nowTachoR-lastTachoR)/180;
			lastTachoL=nowTachoL;								// save tacho counts for next iteration
			lastTachoR=nowTachoR;
			deltaD = 0.5*(distL+distR);							// compute vehicle displacement
			deltaT = (distL-distR)/WB;							// compute change in heading

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				Theta += deltaT;									// update heading
				
				//bounded theta, and negative theta converted to positive equivalent
				if (Theta < 0) Theta += TWOPI;
				Theta %= TWOPI;
				
			    dX = deltaD * Math.sin(Theta);						// compute X component of displacement
				dY = deltaD * Math.cos(Theta);						// compute Y component of displacement
				X = X + dX;											// update estimates of X and Y position
				Y = Y + dY;	
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = X;
			if (update[1])
				position[1] = Y;
			if (update[2])
				position[2] = Theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = X;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = Y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = Theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				X = position[0];
			if (update[1])
				Y = position[1];
			if (update[2])
				Theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.X = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.Y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.Theta = theta;
		}
	}
}