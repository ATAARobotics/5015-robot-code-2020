package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    // Drive values
    double Dp = 0;
    double Di = 0;
    double Dd = 0;

    public Encoders(TalonSRX leftMotor, TalonSRX rightMotor) {

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        this.leftMotor.setSelectedSensorPosition(0);
        this.rightMotor.setSelectedSensorPosition(0);

        leftMotor.configNominalOutputForward(0, 30);
		leftMotor.configNominalOutputReverse(0, 30);
		leftMotor.configPeakOutputForward(1, 30);
		leftMotor.configPeakOutputReverse(-1, 30);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		leftMotor.configAllowableClosedloopError(0, 0, 30);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		leftMotor.config_kP(0, Dp, 30);
		leftMotor.config_kI(0, Di, 30);
        leftMotor.config_kD(0, Dd, 30);
        
                /* Config the peak and nominal outputs, 12V means full */
		rightMotor.configNominalOutputForward(0, 30);
		rightMotor.configNominalOutputReverse(0, 30);
		rightMotor.configPeakOutputForward(1, 30);
		rightMotor.configPeakOutputReverse(-1, 30);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		rightMotor.configAllowableClosedloopError(0, 0, 30);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		rightMotor.config_kP(0, Dp, 30);
		rightMotor.config_kI(0, Di, 30);
		rightMotor.config_kD(0, Dd, 30);

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

    public boolean PID(double target) {
        leftMotor.set(ControlMode.Position, target*leftTicksPerInch);
        rightMotor.set(ControlMode.Position, target*rightTicksPerInch);
        return (leftMotor.getClosedLoopError(0) < 100 && rightMotor.getClosedLoopError(0) < 100);
    }
}
