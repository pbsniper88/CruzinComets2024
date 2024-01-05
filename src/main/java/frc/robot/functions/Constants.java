// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int COLLISION_THRESHOLD = 10;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Assuming the lenth and width of the wheel spacings are even, these can be
    // equal
    // and can be made a constant so its not calculated on each method call. -Shane
    static final double L = 1.0;
    static final double W = 1.0;
    public static final double drive_r = Math.sqrt((L * L) + (W * W));
    public static final double scaler_l = L / drive_r;
    public static final double scaler_w = W / drive_r;
  }

  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;

  public static final double kP = 0.2;
  public static final double kI = 0.0;
  public static final double kD = 0.1;
  public static final double kF = 10.0;
  public static final double kIz = 0.0;
  public static final double kPeakOut = 1.0;
  public static final int timeoutMs = 30;
  public static boolean isDriving = false;
  public static final double scalarSwerve = 0.08;
  public static final double deadZone = 0.15;

  // public static final int frontLeftPosition = 159/1000*360;
  // public static final int frontRightPosition = -42/1000 *360;
  // public static final int backLeftPosition = 12/1000 *360;
  // public static final int backRightPosition = -70/1000 *360;

  public static XboxController driverController = new XboxController(0);
  public static XboxController auxController = new XboxController(1);
}
