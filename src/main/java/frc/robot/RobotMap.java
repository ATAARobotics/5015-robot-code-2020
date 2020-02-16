package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.cuforge.libcu.Lasershark;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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
    private CANSparkMax shootMotor = new CANSparkMax(7, MotorType.kBrushless);
    private VictorSPX intakeMotor = new VictorSPX(5);
    private CANPIDController shootController = shootMotor.getPIDController();
    private Ultrasonic intakeDetector = new Ultrasonic(0, 1);
    // Add climber
    private CANSparkMax leftClimbMotor = new CANSparkMax(10, MotorType.kBrushless);
    private CANSparkMax rightClimbMotor = new CANSparkMax(9, MotorType.kBrushless);


    // Encoders
    // Shooter
    private CANEncoder shooterEncoder = new CANEncoder(shootMotor);
    // Climber
    private CANEncoder climbEncoder = getClimberMotors().getEncoder();

    // Drivetrain
    private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    // Pneumatics
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(0, 1);
    private DoubleSolenoid climberSolenoid = new DoubleSolenoid(2, 3);
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(6, 7);

    // Gyro
    private Gyro NavX = new Gyro();

    private Lasershark laserShark = new Lasershark(5);

    // Controllers for specific actions on the robot, these classes should be
    // accessed directly because they have nice interfaces
    public SWATDrive swatDrive;
    public ColorSensor colorSensor;
    public LimeLight limeLight;
    public Shooter shooter;
    public RangeFinder rangeFinder;
    public Encoders driveEncoders;
    public Climber climber;
    public Align align;

    public RobotMap() {

        // TODO: Reenable camera code
        // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
        // Shuffleboard.getTab("Camera").add(camera);
        // camera.setFPS(30);
        // camera.setResolution(160, 120);

        leftClimbMotor.setInverted(true);
        rightClimbMotor.follow(leftClimbMotor, true);

        // Init submodules
        swatDrive = new SWATDrive(this);
        colorSensor = new ColorSensor(this);
        limeLight = new LimeLight();
        rangeFinder = new RangeFinder(laserShark);
        shooter = new Shooter(shootMotor, magazineMotor, intakeMotor, intakeSolenoid, shooterEncoder, rangeFinder, shootController);
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
     * Returns the detector for balls waiting at the intake
     */
    protected Ultrasonic getIntakeDetector() {
        return intakeDetector;
    }

    /**
     * For internal use in Shooter.java.
     * Returns the hardware color sensor for the control panel.
     */
    protected ColorSensorV3 getColorSensor() {
        return colorSensorHardware;
    }

    public CANPIDController getShooterController() {
        return shootController;
    }

     public CANSparkMax getClimberMotors() {
        return rightClimbMotor;
    }

    public DoubleSolenoid getClimberSolenoid() {
        return climberSolenoid;
    }

    public CANEncoder getClimbEncoder() {
        return climbEncoder;
    }

    public double getDrivetrainTemperature() {
        return (rearRightMotor.getTemperature() + rearLeftMotor.getTemperature() + frontRightMotor.getTemperature() + frontLeftMotor.getTemperature()) / 4;
    }
}
