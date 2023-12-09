package org.ftc6448.simulator.webots;

import java.util.Arrays;

import com.cyberbotics.webots.controller.GPS;

/**
 * Simulates a goBilda odometry pod;
 */
public class WebotsOdometryPod {

	protected final double inchesPerTick;
	

	private static final double METERS_TO_INCHES = 39.3701;
	
	protected double heading;
		
	protected double[] lastPos;
	
	public static final int X=0;
	public static final int Y=1;
	
	protected final boolean vertical;
	
	protected final GPS encoder;
	
	protected double encoderInches;
	
	protected final WebotsBNO055IMU imu;
	
	protected double []startPosition;
	
	public WebotsOdometryPod(WebotsBNO055IMU imu, GPS gps, boolean vertical, double inchesPerTick) {
		this.vertical=vertical;
		this.encoder=gps;
		this.inchesPerTick=inchesPerTick;
		this.imu=imu;
		
	}
	
	public int getPosition() {
		double[] pos=encoder.getValues();
		
		pos[0]=pos[0]*METERS_TO_INCHES;
		pos[1]=pos[1]*METERS_TO_INCHES;
		pos[2]=pos[2]*METERS_TO_INCHES;
		
		
		this.heading=imu.getAngularOrientation().thirdAngle;
		
		/*System.out.println(encoder.getName()+"Global webots heading: "+(this.heading* 180.0 / Math.PI));
		System.out.println(encoder.getName()+" Encoder pos: "+pos[0]+  " "+pos[1]+" "+pos[2]);
		*/
		if (lastPos==null) {
			lastPos=pos;
			startPosition=pos;
			return 0;
		}
	
		/*System.out.println(encoder.getName()+" Encoder pos diff from start: "+(pos[0]-startPosition[0])+ 
				" "+(pos[1]-startPosition[1])+" "+(pos[2]-startPosition[2]));
		*/
		double inches;
		
		if (vertical ) {
			double y=(-(pos[0]-lastPos[0])*Math.sin(this.heading))+
					((pos[1]-lastPos[1])*Math.cos(this.heading));
			
			inches=y;
			
		}
		else  {
			double x=(-(pos[0]-lastPos[0])*Math.cos(this.heading))+
					((pos[1]-lastPos[1])*Math.sin(this.heading));
			
			inches=x;
			
		}
		
		encoderInches+=inches;

	//	System.out.println(encoder.getName()+" => "+encoderInches+" inches");
		
		lastPos=pos;
	
		int ticks=(int)(encoderInches/inchesPerTick);

	//	System.out.println(encoder.getName()+" => "+ticks+" ticks");
		return ticks;
	}
	
	

}
