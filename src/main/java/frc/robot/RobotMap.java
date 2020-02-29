package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.cuforge.libcu.Lasershark;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.vision.LimeLight;

/**
 * A centralized file that keeps track of all robot actuators and physical components
 *
 */

public class RobotMap {
    // Color wheel / Control Panel
    private final I2C.Port colorPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensorHardware = new ColorSensorV3(colorPort);

    // Motors
    // Drive
    private WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(2);
    private WPI_TalonSRX rearLeftMotor = new WPI_TalonSRX(1);
    private WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(4);
    private WPI_TalonSRX rearRightMotor = new WPI_TalonSRX(3);
    private SpeedControllerGroup rightMotors = new SpeedControllerGroup(rearRightMotor, frontRightMotor); // Group
    private SpeedControllerGroup leftMotors = new SpeedControllerGroup(rearLeftMotor, frontLeftMotor); // Group
    //Ball magazine
    private VictorSPX magazineMotor = new VictorSPX(6);

    //Add shooter and conveyor belt
    private CANSparkMax shooterMotor = new CANSparkMax(7, MotorType.kBrushless);
    private VictorSPX intakeMotor = new VictorSPX(5);
    private CANPIDController shooterController = shooterMotor.getPIDController();

    // Add climber
    private CANSparkMax climbMotor = new CANSparkMax(10, MotorType.kBrushless);


    // Encoders
    // Shooter
    private CANEncoder shooterEncoder = new CANEncoder(shooterMotor);

    //Climber
    private CANEncoder climbEncoder = new CANEncoder(climbMotor);

    // Drivetrain
    private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    // Pneumatics
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(0, 1);
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(6, 7);

    // Gyro
    private Gyro gyro = new Gyro();

    private Lasershark intakeDetector = new Lasershark(5);
    private Lasershark shooterDetector = new Lasershark(6);

    private DigitalInput climbLimit = new DigitalInput(7);

    // Controllers for specific actions on the robot, these classes should be
    // accessed directly because they have nice interfaces
    public SWATDrive swatDrive;
    public ColorSensor colorSensor;
    public LimeLight limeLight;
    public Shooter shooter;
    public Encoders driveEncoders;
    public Climber climber;
    public Align align;

    public RobotMap() {

        // Init submodules
        swatDrive = new SWATDrive(this);
        colorSensor = new ColorSensor(this);
        limeLight = new LimeLight();
        shooter = new Shooter(this);
        driveEncoders = new Encoders(rearLeftMotor, rearRightMotor);
        climber = new Climber(this);
        align = new Align(this);

        // Make each side controlled with only one motor object each
        frontLeftMotor.follow(rearLeftMotor);
        frontRightMotor.follow(rearRightMotor);
    }

    //// Drive train ////

    /**
     * Returns the double solenoid accessed with the gear shifting functionality
     * TODO: make a wrapper class for this.
     */
    public DoubleSolenoid getGearShift() {
        return gearShiftSolenoid;
    }

    /**
     * Returns the encoders associated with the drive train.
     * TODO: make a wrapper class for this.
     */
    public Encoders getDriveEncoders() {
        return driveEncoders;
    }

    /**
     * For internal use in SWATDrive.java.
     * Returns the DifferentialDrive object associated with the drive train.
     */
    protected DifferentialDrive getDriveTrain() {
        return driveTrain;
    }

    /**
     * For internal use in Auto.java.
     * Returns the Gyro object associated with the NavX.
     */
    protected Gyro getGyro() {
        return gyro;
    }

    //// Shooter / Elevator ////

    /**
     * For internal use in Shooter.java.
     * Returns the current speed of the shooter.
     */
    protected double checkShooterVelocity() {
        return shooterEncoder.getVelocity();
    }

    /**
     * For internal use in Shooter.java.
     * Sets the speed of the magazine motor.
     */
    protected void setMagazineMotor(double speed) {
        magazineMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * For internal use in Shooter.java.
     * Sets the speed of the intake motor.
     */
    protected void setIntakeMotor(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * For internal use in Shooter.java.
     * Sets the shooter's PID, izone and ff values.
     */
    protected void setShooterPID(double p, double i, double d, double iz, double ff) {
        shooterController.setP(p);
        shooterController.setI(i);
        shooterController.setD(d);
        shooterController.setIZone(iz);
        shooterController.setFF(ff);
    }

    /**
     * For internal use in Shooter.java.
     * Sets the shooter's output range.
     */
    protected void setShooterOutputRange(double min, double max) {
        shooterController.setOutputRange(min, max);
    }

    /**
     * For internal use in Shooter.java.
     * Sets the shooter's PID setpoint.
     */
    protected void setShooterSetPoint(double setPoint) {
        shooterController.setReference(setPoint, ControlType.kVelocity);
    }

    /**
     * For internal use in Shooter.java.
     * Returns the distance in inches of the shooter lasershark.
     */
    protected double checkShooterDetector() {
        return shooterDetector.getDistanceInches();
    }

    /**
     * For internal use in Shooter.java.
     * Returns the distance in inches of the intake lasershark.
     */
    protected double checkIntakeDetector() {
        return intakeDetector.getDistanceInches();
    }

    /**
     * For internal use in Shooter.java
     * Returns the intake up/down solenoid (down = true).
     */
    protected void setIntakeExtended(boolean enabled) {
        if (enabled) {
            intakeSolenoid.set(Value.kForward);
        } else {
            intakeSolenoid.set(Value.kReverse);
        }
    }
    //// Colour Wheel ////

    /**
     * For internal use in ColourWheel.java.
     * Returns the hardware color sensor for the control panel.
     */
    protected ColorSensorV3 getColorSensor() {
        return colorSensorHardware;
    }

    //// Climber ////

    /**
     * For internal use in Climber.java
     * Returns the motor used for climbing.
     */
    protected CANSparkMax getClimberMotor() {
        return climbMotor;
    }

    /**
     * For internal use in Climber.java
     * Returns the encoder used for climbing.
     */
    protected CANEncoder getClimbEncoder() {
        return climbEncoder;
    }

    /**
     * For internal use in Climber.java
     * Returns the switch that limits the climb.
     */
    protected DigitalInput getClimbLimit() {
        return climbLimit;
    }

    //// Miscellaneous ////

    /**
     * Return the temperature of the drivetrain in degrees celcius
     */
    public double getDriveTrainTemperature() {
        return Math.max(
            Math.max(
                rearRightMotor.getTemperature(),
                rearLeftMotor.getTemperature()),
            Math.max(
                frontRightMotor.getTemperature(),
                frontLeftMotor.getTemperature()));
    }
    /**
     *
     */
     public double getShooterTemperature() {
         return shooterMotor.getMotorTemperature();
     }
}
