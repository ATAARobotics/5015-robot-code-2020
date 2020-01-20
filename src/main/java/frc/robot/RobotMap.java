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

    private final I2C.Port colorPort = I2C.Port.kOnboard;

    private final ColorSensorV3 colorSensor = new ColorSensorV3(colorPort);

    // Motors
    public CANSparkMax frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless); // TODO: Make this private.
    private CANSparkMax rearLeftMotor = new CANSparkMax(4, MotorType.kBrushless);
    public CANSparkMax frontRightMotor = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rearRightMotor = new CANSparkMax(2, MotorType.kBrushless);

    private VictorSPX elevatorMotor1 = new VictorSPX(5); // Actually an SPX
    private VictorSPX elevatorMotor2 = new VictorSPX(6);
    
    //Add shooter and conveyor belt
    private CANSparkMax shooter = new CANSparkMax(7, MotorType.kBrushless);
    private DigitalInput intakeDetector = new DigitalInput(0);
    private DigitalInput shooterDetector = new DigitalInput(1);

    //Encoders
    //private CANEncoder leftEncoder = new CANEncoder(frontLeftMotor);
    //private CANEncoder rightEncoder = new CANEncoder(frontRightMotor);
    private CANEncoder shooterEncoder = new CANEncoder(shooter);

    //Group Drive
    private SpeedControllerGroup rightMotors = new SpeedControllerGroup(rearRightMotor, frontRightMotor);
    private SpeedControllerGroup leftMotors = new SpeedControllerGroup(rearLeftMotor, frontLeftMotor);

    //Add drivetrain
    private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    //Add pneumatics
    //private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(2, 3);

    
    //Add gyro
    private Gyro NavX = new Gyro();

    //Add encoders
    //private Encoders driveEncoder = new Encoders(leftEncoder, rightEncoder);

    public RobotMap() {

        // TODO: Uncomment/add camera code
        //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
        //Shuffleboard.getTab("Camera").add(camera);
        //camera.setFPS(30);
        //camera.setResolution(160, 120);

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
