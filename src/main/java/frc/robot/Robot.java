// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//package frc.robot.subsystems;

import java.sql.DriverAction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.functions.TelemetryPublisher;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveDrive;

import frc.robot.functions.Constants;

public class Robot extends TimedRobot {
  private final XboxController drivController = new XboxController(0);
  private TelemetryPublisher telemetryPublisher = new TelemetryPublisher();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private static Drivetrain backRight = new Drivetrain(8, 4);
  private static Drivetrain backLeft = new Drivetrain(7, 3);
  private static Drivetrain frontRight = new Drivetrain(6, 2);
  private static Drivetrain frontLeft = new Drivetrain(5, 1);
  public static SwerveDrive m_swerve = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);

  private static final boolean enableRobustTM = true;
  private static int rateCounter = 0;
  private static double defaultSpeed = 1.0;
  private static double lastStrf = 0.0;
  private static double lastFwd = 0.3;
  private static double lastRot = 0.0;

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick();
  }

  @Override
  // Teleop mode gets called every 20ms
  public void teleopPeriodic() {
    rateCounter++;
    driveWithJoystick();

    // Robot SmartDashboard Telemetry, service every 100ms (for now).
    if (rateCounter % 5 == 0) {
      robotTelemetry();
    }

    if (rateCounter == 100) {
      rateCounter = 0; // Reset to zero to prevent overflows.
    }
  }

  private void driveWithJoystick() {
    // Get the y speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    // Determine if the joystick was released
    double currentLeftY = drivController.getLeftY();
    double currentLeftX = drivController.getLeftX();
    double currentRightX = drivController.getRightX();

    if(wasReleased(currentLeftY) && wasReleased(currentLeftX) && wasReleased(currentRightX)) {
      defaultSpeed = 0.0;
    }
    else {
      lastFwd = currentLeftY;
      lastStrf = currentLeftX;
      lastRot = currentRightX;
      defaultSpeed = 1.0;
    }

    final var ySpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(lastFwd, Constants.deadZone))
            * Drivetrain.kMaxSpeed;

    // Get the x speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var xSpeed =
        m_yspeedLimiter.calculate(MathUtil.applyDeadband(lastStrf, Constants.deadZone))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        m_rotLimiter.calculate(MathUtil.applyDeadband(lastRot, Constants.deadZone))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, defaultSpeed);

    if (enableRobustTM == true) {
      telemetryPublisher.publishTelemetry("xSpeed", xSpeed);
      telemetryPublisher.publishTelemetry("ySpeed", ySpeed);
      telemetryPublisher.publishTelemetry("rot", rot);
    }
  }

  private void robotTelemetry() {
    // Controller variables
    telemetryPublisher.publishTelemetry("Driver Left X", drivController.getLeftX());
    telemetryPublisher.publishTelemetry("Driver Left Y", drivController.getLeftY());
    telemetryPublisher.publishTelemetry("Driver Right X", drivController.getRightX());

    // Sensor Outputs
    telemetryPublisher.publishTelemetry("FL Sensor Raw", frontLeft.getRawSensorVal());
    telemetryPublisher.publishTelemetry("FR Sensor Raw", frontRight.getRawSensorVal());
    telemetryPublisher.publishTelemetry("BL Sensor Raw", backLeft.getRawSensorVal());
    telemetryPublisher.publishTelemetry("BR Sensor Raw", backRight.getRawSensorVal());

    // Sensor Degree Calcs
    telemetryPublisher.publishTelemetry("FL Sensor Deg", frontLeft.getSensorDegree());
    telemetryPublisher.publishTelemetry("FR Sensor Deg", frontRight.getSensorDegree());
    telemetryPublisher.publishTelemetry("BL Sensor Deg", backLeft.getSensorDegree());
    telemetryPublisher.publishTelemetry("BR Sensor Deg", backRight.getSensorDegree());    
  }

  private boolean wasReleased(double inputAxis) {
    if(Math.abs(inputAxis) <= 0.30) {
      return true;
    }
    else
    return false;
  }

}
