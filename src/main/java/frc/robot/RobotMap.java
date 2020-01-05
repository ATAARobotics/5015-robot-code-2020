package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
/**
 * A centralized file that keeps track of all robot actuators and physical components
 * 
 */

public class RobotMap {

    public WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(0);
    private WPI_VictorSPX rearLeftMotor = new WPI_VictorSPX(1);
    public WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(2);
    private WPI_VictorSPX rearRightMotor = new WPI_VictorSPX(3);
    
    private static WPI_TalonSRX staticFrontLeftMotor = new WPI_TalonSRX(0);
    private static WPI_TalonSRX staticFrontRightMotor = new WPI_TalonSRX(2);
    //Group Drive
    private SpeedControllerGroup rightMotors = new SpeedControllerGroup(rearRightMotor, frontRightMotor);
    private SpeedControllerGroup leftMotors = new SpeedControllerGroup(rearLeftMotor, frontLeftMotor);

    //Add drivetrain
    private DifferentialDrive driveTrain = new DifferentialDrive(leftMotors, rightMotors);
    
    //Add pneumatics    
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(2, 3);

    //Add gyro
    private Gyro NavX = new Gyro();

    //Add encoders
    private static Encoders encoder = new Encoders(staticFrontLeftMotor, staticFrontRightMotor);

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

    static public Encoders getEncoder() {
        return encoder;
    }

    public DifferentialDrive getDriveTrain() {
        return driveTrain;
    }

    public Gyro getGyro() {
        return NavX;
    }
}
