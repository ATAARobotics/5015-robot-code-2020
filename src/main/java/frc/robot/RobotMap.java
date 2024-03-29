package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.cuforge.libcu.Lasershark;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.vision.LimeLight;

/**
 * A centralized file that keeps track of all robot actuators and physical components
 *
 */

public class RobotMap {
    // Color wheel / Control Panel
    // private final I2C.Port colorPort = I2C.Port.kOnboard;
    // private final ColorSensorV3 colorSensorHardware = new ColorSensorV3(colorPort);

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
    private CANSparkMax shootMotor = new CANSparkMax(7, MotorType.kBrushless);
    private VictorSPX intakeMotor = new VictorSPX(5);
    private CANPIDController shootController = shootMotor.getPIDController();

    // Add climber
    private CANSparkMax climbMotor = new CANSparkMax(10, MotorType.kBrushless);


    // Encoders
    // Shooter
    private CANEncoder shooterEncoder = new CANEncoder(shootMotor);

    //Climber
    private CANEncoder climbEncoder = new CANEncoder(climbMotor);

    // Drivetrain
    private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    // Pneumatics
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(0, 1);
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(6, 7);

    // Gyro
    private Gyro NavX = new Gyro();

    private Lasershark intakeLaserShark = new Lasershark(5);
    private Lasershark shootLaserShark = new Lasershark(6);

    private DigitalInput climbLimit = new DigitalInput(7);

    // Controllers for specific actions on the robot, these classes should be
    // accessed directly because they have nice interfaces
    public SWATDrive swatDrive;
    //public ColorSensor colorSensor;
    public LimeLight limeLight;
    public Shooter shooter;
    public RangeFinder intakeDetector;
    public RangeFinder shootDetector;
    public Encoders driveEncoders;
    public Climber climber;
    public Align align;

    public RobotMap() {

        // Init submodules
        swatDrive = new SWATDrive(this);
        //colorSensor = new ColorSensor(this);
        limeLight = new LimeLight();
        intakeDetector = new RangeFinder(intakeLaserShark);
        shootDetector = new RangeFinder(shootLaserShark);
        shooter = new Shooter(shootMotor, magazineMotor, intakeMotor, intakeSolenoid, shooterEncoder, intakeDetector, shootDetector, shootController);
        driveEncoders = new Encoders(rearLeftMotor, rearRightMotor);
        climber = new Climber(this);
        align = new Align(this);

        // Make each side controlled with only one motor object each
        frontLeftMotor.follow(rearLeftMotor);
        frontRightMotor.follow(rearRightMotor);
        // PID coefficients


        NavX.initializeNavX();
    }

    // Drive train

    /**
     * Returns the double solenoid accessed with the gear shifting functionality
     */
    public DoubleSolenoid getGearShift() {
        return gearShiftSolenoid;
    }

    /**
     * Returns the encoders associated with the drive train.
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
     * Returns the navx attached to the robot.
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
        return shootMotor;
    }

    /**
     * For internal use in Shooter.java.
     * Returns the first conveyor motor.
     */
    protected VictorSPX getConveyorMotor() {
        return magazineMotor;
    }

    /**
     * For internal use in Shooter.java.
     * Returns the hardware color sensor for the control panel.
     */
    /* protected ColorSensorV3 getColorSensor() {
        return colorSensorHardware;
    } */

    public CANPIDController getShooterController() {
        return shootController;
    }

     public CANSparkMax getClimberMotor() {
        return climbMotor;
    }

    public CANEncoder getClimbEncoder() {
        return climbEncoder;
    }

    public DigitalInput getClimbLimit() {
        return climbLimit;
    }

    public double getDrivetrainTemperature() {
        return (rearRightMotor.getTemperature() + rearLeftMotor.getTemperature() + frontRightMotor.getTemperature() + frontLeftMotor.getTemperature()) / 4;
    }
}
