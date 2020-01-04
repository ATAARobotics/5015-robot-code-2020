package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
class OI {

    //Driver Variables
    private XboxController driveStick = new XboxController(0);
    private String driverScheme = "Default";
    private double XSpeed;
    private double ZRotation;
    private boolean gearShift;
    private boolean slow;
    private boolean visionButton;

    //Gunner variables
    private Joystick gunnerStick = new Joystick(1);
    private String gunnerScheme = "Default";

    //Special Function variables
    boolean leftTriggerPressed = false;
    boolean rightTriggerPressed = false;
    
    public OI() {

    }
    //periodic function to update controller input
    public void checkInputs() {
        gearShift = driveStick.getXButtonReleased();
        slow = driveStick.getAButtonReleased();
        visionButton = driveStick.getBackButtonReleased();

        //TODO Add any new controls for driver

        //Switch statement to determine controls for the driver
        switch (driverScheme) {
            case "Reverse Turning":
                XSpeed = -driveStick.getY(Hand.kLeft);
                ZRotation = driveStick.getX(Hand.kRight);
                break;    
            default:
                XSpeed = driveStick.getY(Hand.kLeft);
                ZRotation = -driveStick.getX(Hand.kRight);
                break;
        }
        
        //TODO Add controls for gunner

        //Switch statement to detirmine controls for the gunner
        
        switch (gunnerScheme) {
            
            default:

                break;   
        }
    }

    //Getter functions for controls
    public double getXSpeed() {
        return XSpeed;
    }
    public double getZRotation() {
        return ZRotation;
    }

    public boolean getGearShift() {
        return gearShift;
    }

    public boolean getSlow() {
        return slow;
    }

    public boolean getVisionButton() {
        return visionButton;
    }
}