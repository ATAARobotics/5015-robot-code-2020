package frc.robot;

import com.revrobotics.CANEncoder;

import java.lang.Math;

/**
 * Encoder code
 *
 * @author Jacob Guglielmin
 */

public class Encoders {

    //Creates left and right encoder objects
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private double leftTicksPerInch;
    private double rightTicksPerInch;
    private double wheelCircumference = 6 * Math.PI;

    public Encoders(CANEncoder leftEncoder, CANEncoder rightEncoder) {

        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;

        this.leftEncoder.setPosition(0);
        this.rightEncoder.setPosition(0);

        //TODO Calculate ticks per inch

        leftTicksPerInch = 0.0 / wheelCircumference;
        rightTicksPerInch = 0.0 / wheelCircumference;
    }
    public double getRight() {
        return rightEncoder.getPosition();
    }
    public double getLeft() {
        return leftEncoder.getPosition() * -1;
    }
    public double getLeftDistance() {
        return (leftEncoder.getPosition() * -1) / leftTicksPerInch;
    }

    public double getRightDistance() {
        return rightEncoder.getPosition() / rightTicksPerInch;
    }

    public void reset() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
}
