package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Encoder code
 *
 * @author Jacob Guglielmin
 */

public class Encoders {

    //Creates left and right encoder objects
    private TalonSRX leftMotor;
    private TalonSRX rightMotor;

    private double leftTicksPerInch;
    private double rightTicksPerInch;

    public Encoders(TalonSRX leftMotor, TalonSRX rightMotor) {

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        this.leftMotor.setSelectedSensorPosition(0);
        this.rightMotor.setSelectedSensorPosition(0);

        leftTicksPerInch = 1820;
        rightTicksPerInch = 1820;
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
