package linefollower;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class lineFollower {

    // Hom Bahadur Adhikari & Bishnu.
    // Motors Connection
    private final RegulatedMotor leftMotor = Motor.A;
    private final RegulatedMotor rightMotor = Motor.B;

    // Sensors Connection
    private final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1); // Sensor 1 is used as light sensor.
    private final EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2); // Sensor 2 is used for
                                                                                                 // object
                                                                                                 // detection.(Ultrasonic
                                                                                                 // sensor)

    // starting the robot to execute the follwong program made by our group.
    public static void main(String[] args) {
        lineFollower robot = new lineFollower();
        robot.run();
    }

    public void run() {
        // Main loop. We have made infinite loop so until we manually stops the robot it
        // runs.
        leftMotor.setAcceleration(300);
        rightMotor.setAcceleration(300);

        while (true) {
            followLine();

            if (isObstacleDetected()) {
                avoidObstacle();
            }
        }
    }

    // Raj did the line following and obstacle detection. and everyone fixed and
    // made final code.
    // Basic line-following logic
    private void followLine() {
        float intensity = readColorSensor();

        // On the line (dark)? Turn left slightly
        if (intensity < 0.3) {
            leftMotor.setSpeed(150); // Slow left
            rightMotor.setSpeed(200); // Fast right
        }
        // Off the line (light)? Turn right slightly
        else {
            leftMotor.setSpeed(200); // Fast left
            rightMotor.setSpeed(150); // Slow right
        }

        // Keep moving forward
        leftMotor.forward();
        rightMotor.forward();
    }

    // Check if obstacle is near (distance < 25 cm)
    private boolean isObstacleDetected() {
        float distance = readUltrasonicSensor();
        return distance < 0.10f;
    }

    // Avoid obstacle: Turn left, move forward, then turn right
    private void avoidObstacle() {
        // 1. Stop
        leftMotor.stop();
        rightMotor.stop();
        sleep(1000);

        // 2. Turn right (adjust degrees if needed)
        leftMotor.rotate(180, true);
        rightMotor.rotate(-180);

        // 3. Move forward for 2 second
        leftMotor.setSpeed(200);
        rightMotor.setSpeed(200);
        leftMotor.forward();
        rightMotor.forward();
        sleep(3000); // small break.

        // === 4. Move straight for 1 second ===
        leftMotor.forward();
        rightMotor.forward();
        sleep(1000);

        // === 5. Turn left the same amount as first right turn (90 degrees) ===
        leftMotor.rotate(-180, true); // Left motor backward
        rightMotor.rotate(180); // Right motor forward

        searchForLine();
    }

    private void searchForLine() {
        // Move forward slowly while checking for the line
        leftMotor.setSpeed(150);
        rightMotor.setSpeed(150);
        leftMotor.forward();
        rightMotor.forward();

        // Keep checking until the line is found
        while (readColorSensor() > 0.3) { // While not on the line
            sleep(50); // Small delay
        }
    }

    private float readColorSensor() {
        SampleProvider color = colorSensor.getRedMode();
        float[] sample = new float[color.sampleSize()];
        color.fetchSample(sample, 0);
        return sample[0];
    }

    // Read distance from ultrasonic sensor (in meters)
    private float readUltrasonicSensor() {
        SampleProvider distance = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distance.sampleSize()];
        distance.fetchSample(sample, 0);
        return sample[0];
    }

    // Pause execution 
    private void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
        }
    }
}
