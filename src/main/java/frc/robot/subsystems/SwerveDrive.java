package frc.robot.subsystems;

import frc.robot.functions.Constants;
import frc.robot.functions.TelemetryPublisher;

public class SwerveDrive {

    private Drivetrain backRight;
    private Drivetrain backLeft;
    private Drivetrain frontRight;
    private Drivetrain frontLeft;

    public double backRightAngle = 0;
    public double backLeftAngle = 0;
    public double frontRightAngle = 0;
    public double frontLeftAngle = 0;
    public double backRightSpeed = 0;
    public double backLeftSpeed = 0;
    public double frontRightSpeed = 0;
    public double frontLeftSpeed = 0;

    private TelemetryPublisher telemetryPublisher = new TelemetryPublisher();

    public SwerveDrive(Drivetrain backRight, Drivetrain backLeft, Drivetrain frontRight, Drivetrain frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        
      }

    public void drive(double strafeInput, double forwardInput, double rotate, double maxSpeed) {

        // TODO: Commenting out for now, may need to bring back later.
        //boolean isForward = true;
        //forwardInput *= -1;

        // Calculate the omega components
        double a = strafeInput - rotate * Constants.OperatorConstants.scaler_l;
        double b = strafeInput + rotate * Constants.OperatorConstants.scaler_l;
        double c = forwardInput - rotate * Constants.OperatorConstants.scaler_w;
        double d = forwardInput + rotate * Constants.OperatorConstants.scaler_w;

        // Drive equation vectors
        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        // Normalize speeds
        double[] normSpeeds = normalizeSpeeds(backRightSpeed, backLeftSpeed, frontRightSpeed, frontLeftSpeed, maxSpeed);

        backRightAngle = (Math.atan2 (a, c) / Math.PI) * 180;
        backLeftAngle = (Math.atan2 (a, d) / Math.PI) * 180;
        frontRightAngle = (Math.atan2 (b, c) / Math.PI) * 180;
        frontLeftAngle = (Math.atan2 (b, d) / Math.PI) * 180;

        backRight.drive(normSpeeds[0], backRightAngle, false);
        backLeft.drive(normSpeeds[1], backLeftAngle, false);
        frontRight.drive(normSpeeds[2], frontRightAngle, false);
        frontLeft.drive(normSpeeds[3], frontLeftAngle, false);

        // Publish Telemetry for now
        telemetryPublisher.publishTelemetry("Back Right Speed", normSpeeds[0]); // Test Wheel
        telemetryPublisher.publishTelemetry("Back Left Speed", normSpeeds[1]);
        telemetryPublisher.publishTelemetry("Front Right Speed", normSpeeds[2]);
        telemetryPublisher.publishTelemetry("Front Left Speed", normSpeeds[3]);

        telemetryPublisher.publishTelemetry("Back Right Angle", backRightAngle); // Test Wheel
        telemetryPublisher.publishTelemetry("Back Left Angle", backLeftAngle);
        telemetryPublisher.publishTelemetry("Front Right Angle", frontRightAngle);
        telemetryPublisher.publishTelemetry("Front Left Angle", frontLeftAngle);
    }

    public static double[] normalizeSpeeds(double a, double b, double c, double d, Double maxValue) {
        double maxLimit = (maxValue != null) ? maxValue : 1.0;
        double[] speeds = {a, b, c, d};
        double max = Math.abs(speeds[0]);

        // Find the maximum absolute value among the speeds
        for (int i = 1; i < speeds.length; i++) {
            if (Math.abs(speeds[i]) > max) {
                max = Math.abs(speeds[i]);
            }
        }

        // Scale down the speeds if the maximum is greater than the maxLimit
        if (max > maxLimit) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= max;
                speeds[i] *= maxLimit; // Scale to the maxLimit
            }
        }

        return speeds;
    }

    // Overloaded method for convenience when maxLimit is not specified
    public static double[] normalizeSpeeds(double a, double b, double c, double d) {
        return normalizeSpeeds(a, b, c, d, 1.0);
    }
}
