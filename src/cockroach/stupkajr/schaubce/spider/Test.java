package cockroach.stupkajr.schaubce.spider;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Test {

	// Motors
	static RegulatedMotor leftMotor = Motor.D;
	static RegulatedMotor rightMotor = Motor.A;

	// Sensors
	static EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

	 static int speed = 720; // 1 RPM


	 static SampleProvider sonicSample;
	 static SampleProvider gyroSample;

	/******
	* Caitlin:" HEY JOHN HERE ARE SOME CODING STANDARDS FOR YOU TO REMEMBER:
	* 
	* 1. CamelCase 2. No Underscores 3. Variables/Methods should start
	* lowercase"
	* 
	* John: "Thanks! <3"
	* 
	*/

	public static void main(String[] args) {
	sonicSample = sonicSensor.getDistanceMode();
	gyroSample = gyroSensor.getAngleAndRateMode();
	
	leftMotor.setSpeed(speed);
	rightMotor.setSpeed(speed);
	 
	leftMotor.forward();
	rightMotor.forward();
	
	
	
	float[] distance = new float[sonicSample.sampleSize()];
	sonicSample.fetchSample(distance, 0);
	
	
	while(atWall()){
		
		fullSpin();
				
		
		
		if(Button.ENTER.isDown()){
			System.exit(0);
		}
	}
	

	}


	private static boolean atWall() {
		float[] distance = new float[sonicSample.sampleSize()];
		sonicSample.fetchSample(distance, 0);
		System.out.println(distance[0]);
		if(distance[0] <= 0.25){
			return false;
		}else{
			return true;	
		}		
	}
	
	private static void fullSpin(){
		
		float[] angle = new float[gyroSample.sampleSize()];
		gyroSample.fetchSample(angle, 0);
		System.out.println(angle);		
		
	}
	

}
