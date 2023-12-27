package frc.robot.functions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelemetryPublisher {
    // Constructor
    public TelemetryPublisher() {
        // Initialize your telemetry system here if needed
    }

    // Method to publish telemetry data
    public void publishTelemetry(String name, double value) {
        // Publish the telemetry data to the Smart Dashboard
        SmartDashboard.putNumber(name, value);
    }

    // You can overload this method for different data types or sensors if needed    
}
