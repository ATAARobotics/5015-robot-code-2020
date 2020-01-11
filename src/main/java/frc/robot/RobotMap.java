package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

    public CANSparkMax frontLeftMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rearLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
    public CANSparkMax frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rearRightMotor = new CANSparkMax(3, MotorType.kBrushless);
    
    //Encoders
    private CANEncoder leftEncoder = new CANEncoder(frontLeftMotor);
    private CANEncoder rightEncoder = new CANEncoder(frontRightMotor);

    //Group Drive
    private SpeedControllerGroup rightMotors = new SpeedControllerGroup(rearRightMotor, frontRightMotor);
    private SpeedControllerGroup leftMotors = new SpeedControllerGroup(rearLeftMotor, frontLeftMotor);

    //Add drivetrain
    private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);
    
    //Add pneumatics    
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(2, 3);

    //Add shooter and conveyor belt
    private CANSparkMax shooter = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax conveyorBeltMotor = new CANSparkMax(5, MotorType.kBrushless);
    private CANEncoder conveyorEncoder = new CANEncoder(conveyorBeltMotor);
    private DigitalInput ballDetector = new DigitalInput(0);

    //Add gyro
    private Gyro NavX = new Gyro();

    //Add encoders
    private Encoders driveEncoder = new Encoders(leftEncoder, rightEncoder);

    public RobotMap() {

        //TODO Uncomment/add camera code
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
    
    public DoubleSolenoid getGearShift() {
        return gearShiftSolenoid;
    }

    public Encoders getDriveEncoder() {
        return driveEncoder;
    }

    public DifferentialDrive getDriveTrain() {
        return driveTrain;
    }

    public Gyro getGyro() {
        return NavX;
    }

    public CANEncoder getBallMagazineEncoder() {
        return conveyorEncoder;
    }

    public CANSparkMax getBallMagazineMotor() {
        return conveyorBeltMotor;
    }

    public CANSparkMax getShooter() {
        return shooter;
    }

    public DigitalInput getBallDetector() {
        return ballDetector;
    }
}
