package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.functions.Constants;
import frc.robot.functions.TelemetryPublisher;

public class Drivetrain {
    private TalonSRX angleMotor;
    private CANSparkMax driveMotor;
    private int counter = 0;
    public PIDController anglePIDController;
    public double PIDVal = 0;
    public static final double kMaxSpeed = 1.0;
    public static final double kMaxAngularSpeed = 1.0; // 1/2 rotation per second  
    private TelemetryPublisher telemetryPublisher = new TelemetryPublisher();  
    
    public Drivetrain(int driveMotor, int angleMotor) {
        this.angleMotor = new WPI_TalonSRX(angleMotor);
        this.driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);

        this.angleMotor.configFactoryDefault();

        this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);

        this.angleMotor.configNeutralDeadband(0.001, Constants.timeoutMs);

        this.angleMotor.setSensorPhase(true);
        this.angleMotor.setInverted(false);

        		/* Set relevant frame periods to be at least as fast as periodic rate */
        this.angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        this.angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

        		/* Set the peak and nominal outputs */
        this.angleMotor.configNominalOutputForward(0, Constants.timeoutMs);
        this.angleMotor.configNominalOutputReverse(0, Constants.timeoutMs);
        this.angleMotor.configPeakOutputForward(1, Constants.timeoutMs);
        this.angleMotor.configPeakOutputReverse(-1, Constants.timeoutMs);

        this.angleMotor.selectProfileSlot(angleMotor, 0);
        this.angleMotor.config_kF(angleMotor, Constants.kF, Constants.timeoutMs);
        this.angleMotor.config_kP(angleMotor, Constants.kP, Constants.timeoutMs);
        this.angleMotor.config_kI(angleMotor, Constants.kI, Constants.timeoutMs);
        this.angleMotor.config_kD(angleMotor, Constants.kD, Constants.timeoutMs);

        

        // Set the resolution of the encoder
        this.angleMotor.configFeedbackNotContinuous(true, 10);
        //this.angleMotor.configSelectedFeedbackCoefficient(1.0 / 1024, 0, 10);

        // Configure Motion Magic parameters
        this.angleMotor.configMotionCruiseVelocity(500, 10);
        this.angleMotor.configMotionAcceleration(500, 10);

        this.angleMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);


    }

    public void drive(double speed, double desiredAngle, boolean isForward) {
        driveMotor.set(speed);
        int posTicks = convertToTicks(1004, desiredAngle );
        //angleMotor.set(ControlMode.Position, posTicks);
        angleMotor.set(ControlMode.MotionMagic, posTicks);
        telemetryPublisher.publishTelemetry("Converted Ticks Angle", posTicks);
    }

    public static int convertToTicks(int setValue, double degree) {
        // Calculate the number of ticks per degree
        double ticksPerDegree = 1024.0 / 360.0;

        // Calculate the tick offset based on the degree
        int tickOffset = (int) Math.round(degree * ticksPerDegree);

        // Calculate the resultant tick value and wrap around if necessary
        int resultTick = (setValue + tickOffset) % 1024;

        // Handle negative values by wrapping around
        if (resultTick < 0) {
            resultTick += 1024;
        }

        return resultTick;
    }

    public double getSensorDegree() {
        double tempPos = angleMotor.getSelectedSensorPosition();
        double degrees = (2 * ((tempPos / 1023) - Math.floor(1.0 / 2 + tempPos / 1023)) * 180);
        return -degrees;        
    }

    public double getRawSensorVal() {
        return angleMotor.getSelectedSensorPosition();
    }    
}
