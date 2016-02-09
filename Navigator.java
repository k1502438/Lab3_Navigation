package ev3navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread{
//constructor params - the necessary objects for the functioning of this class
	Odometer odometer;
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	UltrasonicPoller usPoller;
	ObstacleAvoidance avoidance;
	
//member variables
	//control flow variables
	private enum State {INIT, TURNING, TRAVELLING, EMERGENCY};
	private State state = State.INIT; //state is given an initial value of INIT
	private boolean isNavigating;
	private boolean safeToNavigate;
	private boolean alreadyTravelling;
	
	//some constants
	private static final double TWOPI = Math.PI * 2; //to avoid re-computation throughout runtime
	private static final double ANGULAR_MARGIN = 0.04 * Math.PI; //margin of error in radians such that an angle of desiredAngle +/- ANGULAR_MARGIN is acceptable
	private static final double DISTANCE_MARGIN = 2;
	private static final int ROTATE_SPEED = 60;
	private static final int FORWARD_SPEED = 150;
	private static final double WR = 2.125; //wheel radius
	private static final double WB = 15.5;	//distance between wheels
	private static final double WBR = WB/2; //half distance between wheels (effective radius of rotation)
	private static final float EMERGENCY_DISTANCE = 15;
	
	//destination coordinates
	double destX;
	double destY;
	double destTheta;
	
	
//CONSTRUCTOR
//
	public Navigator(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, UltrasonicPoller usPoller) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usPoller = usPoller;
	}

//THE RUN LOOP
//
	@Override
	public void run() {
		runloop:
		while (true) {
			switch (state) {
			//the INIT case is for dealing with the possibility of not-yet-defined destination coordinates
			//which get defined in the setDestAndNavigate calls made in Lab3NAV (the "main" thread) 4 times, once for each point to travel to
			case INIT:
				//if isNavigating is false, sleep and wait for it to be true.
				//otherwise, enter the TURNING state on the next iteration
				if (isNavigating) { //isNavigating is set to true by the setDestAndNavigate function
					state = State.TURNING;
				}
				break;
			//in this state, the robot will orient itself to aim at its destination point
			case TURNING:
				//in the TURNING state, there is only one thing to do: turn to face the destination coordinate.  Then, prepare to enter TRAVELLING state
				turnToDest();
				state = State.TRAVELLING;
				//System.err.println("rem_ang: " + rad2deg(angleToTurnBy()));
				break;
			//the TRAVELLING state is for when the robot has already been made to point towards its destination pont
			//in the TURNING state and now just needs to travel to the point
			case TRAVELLING:
				//travels the correct distance and does not send redundant commands to wheels
				if (!alreadyTravelling) {
					travelToDest();
					alreadyTravelling = true;
				}
				//check if approaching obstacle
				if (checkEmergency()) {
					leftMotor.stop();
					rightMotor.stop();
					state = State.EMERGENCY;
					alreadyTravelling = false;	
					avoidance = new ObstacleAvoidance(this);
					avoidance.start();
				//check to see if it has gotten to the destination coordinates in which case it waits for new coordinates
				} else if (arrived()){
					state = State.INIT;
					isNavigating = false;
					alreadyTravelling = false;
				}
				break; //this is to test that it gets to this point ok
				//break;
			//the EMERGENCY state is for dealing with obstacles
			case EMERGENCY:
				if (avoidance.resolved) {
					state = State.TURNING;
				}
				break;
			}
			
			//frequency of run-loop is 1/(30 ms) = 1/(30 E-3) = 1000/30 = 33.3... iterations per second
			//(not including execution of statements in loop)
			try {
				sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		//System.err.println("exited run loop");
	}
	

//THE CRUCIAL METHODS
//
	//sets destination point and allows run-loop logic to be executed by setting isNavigating to true
	public void setDestAndNavigate(double x, double y) {
		setDest(x,y);
		this.isNavigating = true;		
	}
	
	//rotates robot by diffTheta # of degrees
	public void turnToDest() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		int wheelangle = (int)rad2deg(((WBR/WR) * angleToTurnBy()));
		
		//debug printing
		//System.err.println("attb: " + rad2deg(angleToTurnBy()));
		//System.err.println("wa: " + wheelangle);
		
		leftMotor.rotate(wheelangle, true); //this statement doesn't stop the flow of execution so that the other wheel can also be activated
		rightMotor.rotate(-wheelangle, false); //this statement does stop the flow of execution so that the program waits for the robot to finish rotating
		
	}
	
	//now that the robot is facing its destination point, move it forward by the correct amount so that it reaches that point
	public void travelToDest() {

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		int wheelangle = (int)rad2deg(distance2destination() / WR);
		
		leftMotor.rotate(wheelangle, true); //since both calls to rotate() take false as an argument
		rightMotor.rotate(wheelangle, true); //any code that follows these calls may execute during the rotations

	}
	
	
//GETTERS
//
	//getter for private boolean isNavigating
	public boolean isNavigating() {
		return isNavigating;
	}
	
//HELPER FUNCTIONS - there are many
//
	//converts an angle in radians to an angle in degrees
	public static double rad2deg(double t) {
		return (t/TWOPI)*360;
	}
	//converts an angle in degrees to an angle in radians
	public static double deg2rad(double d) {
		return (d/360)*TWOPI;
	}
	
	//determines if toTest is in the range [testAgainst - margin, testAgainst + margin]
	private boolean withinRange(double toTest, double testAgainst, double margin) {
		return (toTest > (testAgainst - margin)) && (toTest < (testAgainst + margin));
	}
	
	//returns the difference between the desired angle and the current angle
	//the result is corrected to its MINIMAL value as suggested in lecture/lab slides
	private double angleToTurnBy() {
		double result = destTheta - odometer.getTheta();
		if (result < -Math.PI) {
			result += TWOPI;
		} else if (result > Math.PI) {
			result -= TWOPI;
		}
		return result;
	}
	
	//true if the current theta is acceptably close to the desired theta, destTheta
	private boolean facingDest() {
		return withinRange(odometer.getTheta(), destTheta, ANGULAR_MARGIN);
	}
	
	//returns true if acceptably close to destination coordinates
	private boolean arrived() {
		return withinRange(odometer.getX(),destX,DISTANCE_MARGIN) && withinRange(odometer.getY(),destY,DISTANCE_MARGIN);
	}
	
	//sets destination points and update destTheta
	private void setDest(double x, double y) {
		this.destX = x;
		this.destY = y;
		updateDestTheta();
	}
	//updates destTheta using odometer coordinataes and destination coordinates
	private void updateDestTheta() {
		this.destTheta = Math.atan2(destX - odometer.getX(), destY - odometer.getY());
	}
	
	//returns the distance to the destination coordinates from the current current x and y as taken from the odometer
	private double distance2destination() {
		return Math.sqrt(Math.pow(destX - odometer.getX(), 2) + Math.pow(destY - odometer.getY(), 2));
	}
	
	//returns true if the most recent distance measurement from the ultrasonic sensor is
	//below the acceptable amount
	private boolean checkEmergency() {
		return usPoller.getUSdistance() < EMERGENCY_DISTANCE;
	}
}
