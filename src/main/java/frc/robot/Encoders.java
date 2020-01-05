package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.lang.Math;

/**
 * Encoder code
 * 
 * @author Jacob Guglielmin
 */

public class Encoders {
    
    //Creates left and right encoder objects
    private WPI_TalonSRX leftMotor;
    private WPI_TalonSRX rightMotor;

    private double leftTicksPerInch;
    private double rightTicksPerInch;
    private double wheelCircumference = 6 * Math.PI;

    public Encoders(WPI_TalonSRX leftMotor, WPI_TalonSRX rightMotor) {

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        this.leftMotor.setSelectedSensorPosition(0);
        this.rightMotor.setSelectedSensorPosition(0);

        //TODO Calculate ticks per inch

        leftTicksPerInch = 0.0 / wheelCircumference;
        rightTicksPerInch = 0.0 / wheelCircumference;
    }
    public double getRight() {
        return rightMotor.getSelectedSensorPosition();
    }
    public double getLeft() {
        return leftMotor.getSelectedSensorPosition() * -1;
    }
    public double getLeftDistance() {
        return (leftMotor.getSelectedSensorPosition() * -1) / leftTicksPerInch;     
    }

    public double getRightDistance() {
        return rightMotor.getSelectedSensorPosition() / rightTicksPerInch;
    }

    public void reset() {
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
    }
}
