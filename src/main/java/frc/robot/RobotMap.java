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
/**
 * A centralized file that keeps track of all robot actuators and physical components
 *
 */

public class RobotMap {
    // Color wheel / Control Panel
    private final I2C.Port colorPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(colorPort);

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
    private CANSparkMax shooter = new CANSparkMax(7, MotorType.kBrushless);
    private DigitalInput intakeDetector = new DigitalInput(0);
    private DigitalInput shooterDetector = new DigitalInput(1);

    //Encoders
    // Drive
    // private CANEncoder leftEncoder = new CANEncoder(frontLeftMotor);
    // private CANEncoder rightEncoder = new CANEncoder(frontRightMotor);
    // private Encoders driveEncoder = new Encoders(leftEncoder, rightEncoder); // Group
    // Shooter
    private CANEncoder shooterEncoder = new CANEncoder(shooter);

    // Drivetrain
    private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    // Pneumatics
    // private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(2, 3);

    // Gyro
    private Gyro NavX = new Gyro();


    public RobotMap() {

        // TODO: Uncomment/add camera code
        // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
        // Shuffleboard.getTab("Camera").add(camera);
        // camera.setFPS(30);
        // camera.setResolution(160, 120);

        // Make each side controlled with only one motor object each
        rearLeftMotor.follow(frontLeftMotor);
        rearRightMotor.follow(frontRightMotor);

        NavX.initializeNavX();
    }

    public SpeedControllerGroup getLeftMotors() {
        return leftMotors;
    }

    public SpeedControllerGroup getRightMotors() {
        return rightMotors;
    }

    /*public DoubleSolenoid getGearShift() {
        return gearShiftSolenoid;
    }*/

    /*public Encoders getDriveEncoder() {
        return driveEncoder;
    }*/

    public CANEncoder getShooterEncoder() {
        return shooterEncoder;
    }

    public DifferentialDrive getDriveTrain() {
        return driveTrain;
    }

    public Gyro getGyro() {
        return NavX;
    }

    public CANSparkMax getShooter() {
        return shooter;
    }

    public VictorSPX getElevatorMotor1() {
        return elevatorMotor1;
    }

    public VictorSPX getElevatorMotor2() {
        return elevatorMotor2;
    }

    public DigitalInput getIntakeDetector() {
        return intakeDetector;
    }

    public DigitalInput getShooterDetector() {
        return shooterDetector;
    }

    public ColorSensorV3 getColorSensor() {
        return colorSensor;

    }
}
