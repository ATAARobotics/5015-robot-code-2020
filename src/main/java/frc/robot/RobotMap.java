package frc.robot;


import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
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
    private CANSparkMax frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax rearLeftMotor = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax frontRightMotor = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rearRightMotor = new CANSparkMax(2, MotorType.kBrushless);
    private SpeedControllerGroup rightMotors = new SpeedControllerGroup(rearRightMotor, frontRightMotor); // Group
    private SpeedControllerGroup leftMotors = new SpeedControllerGroup(rearLeftMotor, frontLeftMotor); // Group

    // Elevator
    private VictorSPX elevatorMotor1 = new VictorSPX(5);
    private VictorSPX elevatorMotor2 = new VictorSPX(6);

    // Shooter
    private CANSparkMax shooterMotor = new CANSparkMax(7, MotorType.kBrushless);
    private DigitalInput intakeDetector = new DigitalInput(0);
    private DigitalInput shooterDetector = new DigitalInput(1);

    // Encoders
    // Drive
    // private CANEncoder leftEncoder = new CANEncoder(frontLeftMotor);
    // private CANEncoder rightEncoder = new CANEncoder(frontRightMotor);
    // private Encoders driveEncoders = new Encoders(leftEncoder, rightEncoder); // Group
    // Shooter
    private CANEncoder shooterEncoder = new CANEncoder(shooterMotor);

    // Drivetrain
    private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    // Pneumatics
    // private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(2, 3);

    // Gyro
    private Gyro NavX = new Gyro();

    // Controllers for specific actions on the robot, these classes should be
    // accessed directly because they have nice interfaces
    public SWATDrive swatDrive;
    public ColorSensor colorSensor;
    public LimeLight limeLight;
    public Shooter shooter;

    public RobotMap() {

        // TODO: Reenable camera code
        // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
        // Shuffleboard.getTab("Camera").add(camera);
        // camera.setFPS(30);
        // camera.setResolution(160, 120);

        // Init submodules
        swatDrive = new SWATDrive(this);
        colorSensor = new ColorSensor(this);
        limeLight = new LimeLight();
        shooter = new Shooter(this);

        // Make each side controlled with only one motor object each
        rearLeftMotor.follow(frontLeftMotor);
        rearRightMotor.follow(frontRightMotor);

        NavX.initializeNavX();
    }

    // Drive train

    /**
     * Returns the double solenoid accessed with the gear shifting functionality
     * TODO: make a wrapper class for this.
     */
    // public DoubleSolenoid getGearShift() {
    //     return gearShiftSolenoid;
    // }

    /**
     * Returns the encoders associated with the drive train.
     * TODO: make a wrapper class for this.
     */
    // public Encoders getDriveEncoders() {
    //     return driveEncoders;
    // }

    /**
     * For internal use in SWATDrive.java.
     * Returns the DifferentialDrive object associated with the drive train.
     */
    protected DifferentialDrive getDriveTrain() {
        return driveTrain;
    }

    /**
     * Returns the navx attached to the robot.
     * TODO: make a wrapper class for this.
     */
    public Gyro getGyro() {
        return NavX;
    }

    // Shooter / Elevator

    /**
     * For internal use in Shooter.java.
     * Returns the hardware encoder on the shooter motor.
     */
    protected CANEncoder getShooterEncoder() {
        return shooterEncoder;
    }

    /**
     * For internal use in Shooter.java.
     * Returns the hardware shooter motor.
     */
    protected CANSparkMax getShooterMotor() {
        return shooterMotor;
    }

    /**
     * For internal use in Shooter.java.
     * Returns the first elevator motor.
     */
    protected VictorSPX getElevatorMotor1() {
        return elevatorMotor1;
    }

    /**
     * For internal use in Shooter.java.
     * Returns the seconed elevator motor.
     */
    protected VictorSPX getElevatorMotor2() {
        return elevatorMotor2;
    }

    /**
     * For internal use in Shooter.java.
     * Returns the detector for balls waiting at the intake
     */
    protected DigitalInput getIntakeDetector() {
        return intakeDetector;
    }

    /**
     * For internal use in Shooter.java.
     * TODO: Replace this with a detector for the shooter encoder slowing down.
     * Returns the shooting detector, which detects balls exiting the robot
     */
    protected DigitalInput getShooterDetector() {
        return shooterDetector;
    }

    /**
     * For internal use in Shooter.java.
     * Returns the hardware color sensor for the control panel.
     */
    protected ColorSensorV3 getColorSensor() {
        return colorSensorHardware;
    }
}
