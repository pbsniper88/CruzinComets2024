package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.functions.Constants;

public class Drivetrain {
    private TalonSRX angleMotor;
    private CANSparkMax driveMotor;
    private int counter = 0;
    public PIDController anglePIDController;
    public double PIDVal = 0;
    public static final double kMaxSpeed = 1.0;
    public static final double kMaxAngularSpeed = 1.0; // 1/2 rotation per second    
    
    public Drivetrain(int driveMotor, int angleMotor) {
        this.angleMotor = new TalonSRX(angleMotor);
        this.driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);

        this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    }

    public void drive(double speed, double Motorangle, double desiredAngle, boolean isForward) {
        driveMotor.set(speed);
        angleMotor.set(ControlMode.Position, desiredAngle);
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
