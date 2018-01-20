package cockroach.stupkajr.schaubce.spider;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

/*
 * Things to consider:
 * Apparently it might have to go through the light to find the darkest place?
 * Will probably use steepest ascent because Annealing may take too long 
 * Two minute time limit
 * 
 * 
 */

public class Cockroach {
	// Motors
	static RegulatedMotor leftMotor = Motor.D;
	static RegulatedMotor rightMotor = Motor.A;

	// Sensors
	static EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

	static SampleProvider colorSample;
	static SampleProvider angleSample;

	private static float originalColor;
	
	//Nodes
	private static Node bestState; 
	private static Node current; 

	public static void main(String[] args) {
		boolean light = false;
		float[] color = new float[colorSample.sampleSize()];
		colorSample.fetchSample(color, 0);

		originalColor = color[0];

		// while there is no light
		while (true) {
			// sense for light
			light = senseLight();

			// When the spider sees light, find the dark
			if (light) {
				findTheDark();
			}

			if (Button.ENTER.isDown()) {
				leftMotor.stop();
				rightMotor.stop();

				System.exit(0);
			}
		}
	}

	private static boolean senseLight() {
		// get the color sample to save
		float[] color = new float[colorSample.sampleSize()];
		colorSample.fetchSample(color, 0);

		return color[0] != originalColor;
	}

	public static void findTheDark() {
		float[] color = new float[colorSample.sampleSize()];
		colorSample.fetchSample(color, 0);

		//initialize the nodes
		current = new Node(color[0], 0);
		bestState = current; 

		boolean darkestPlace = false;
		// while the robot hasn't found the darkest place or the current state
		// hasnt changed
		while (!darkestPlace) {

			Node nextSpot = hillClimb(current);
			if (nextSpot.getAmbient() == current.getAmbient()) {
				darkestPlace = true;
			} else {
				// move next spot
				moveTo(nextSpot.getAngle());

				// currentNode = nextNode;
				current = nextSpot;
			}
		}

		// signal it is in the darkest place
		Sound.beep();
		Sound.beep();
		Sound.beep();
	}

	private static void moveTo(float finalAngle) {
		gyroSensor.reset();
		double angle = 0;
		double tempAngle = finalAngle; 
		float[] gyroAngle = new float[angleSample.sampleSize()];
		angleSample.fetchSample(gyroAngle, 0);
		angle = gyroAngle[0];
		
		//rotate until it hits the angle
		while (Math.abs(angle) != finalAngle) {
			rightMotor.rotate((int)tempAngle);
			angleSample.fetchSample(gyroAngle, 0);
			angle = gyroAngle[0];
			tempAngle = 1;
		}
		
		//move once it is facing the right angle

	}

	public static Node hillClimb(Node currentNode) {
		// get the children (neighbors in this case)
		ArrayList<Node> neighbors = getNeighbors();

		//pick the best neighbor
		Node nextNode = getBestNeighbor(neighbors); 

		//if the next best node is darker, save it
		if (nextNode.getAmbient() < bestState.getAmbient()){
			currentNode = nextNode; 
			bestState = currentNode; 
		}

		return currentNode;
	}

	private static Node getBestNeighbor(ArrayList<Node> neighbors) {
		// sort the points by x
		Node min = Collections.min(neighbors, new Comparator<Node>() {

			@Override
			public int compare(Node arg0, Node arg1) {
				if (arg0.getAmbient() < arg1.getAmbient()) {
					return -1;
				} else if (arg0.getAmbient() == arg1.getAmbient()) {
					return 0;
				} else {
					return 1;
				}
			}
		});

		return min;
	}

	private static ArrayList<Node> getNeighbors() {
		ArrayList<Node> neighbors = new ArrayList<Node>();

		// get 36 neighbors
		for (int i = 0; i < 36; i++) {
			angleSample = gyroSensor.getAngleMode();
			gyroSensor.reset();

			// get the sample
			float[] gyroAngle = new float[angleSample.sampleSize()];
			angleSample.fetchSample(gyroAngle, 0);
			float angle = gyroAngle[0];

			// spin 10 degrees
			int rotated = 10;
			while (Math.abs(angle) != 10) {
				rightMotor.rotate(rotated);
				angleSample.fetchSample(gyroAngle, 0);
				angle = gyroAngle[0];
				rotated = 1;
			}

			// get the color sample to save
			float[] color = new float[colorSample.sampleSize()];
			colorSample.fetchSample(color, 0);

			// add the node to the neighbors
			neighbors.add(new Node(color[0], (float) (i + 1) * 10));
		}

		return neighbors;
	}
}
