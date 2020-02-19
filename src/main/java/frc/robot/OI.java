package frc.robot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean climbToggle;
    private boolean manualClimb;
    private boolean climbRelease;
    private boolean intakeToggle;

    //Gunner variables
    private XboxController gunnerStick = new XboxController(1);
    private String gunnerScheme = "Default";
    private boolean shoot;
    private boolean overrideSafeties = false;
    private boolean overriding = false;
    private boolean discoToggle;

    public OI() {

    }
    //periodic function to update controller input
    public void checkInputs() {
        gearShift = driveStick.getXButtonReleased();
        slow = driveStick.getAButtonReleased();
        visionButton = driveStick.getBackButtonReleased();
        climbToggle = driveStick.getBumperReleased(Hand.kLeft);
        climbRelease = driveStick.getStartButton();
        manualClimb = gunnerStick.getBumperReleased(Hand.kRight);
        intakeToggle = driveStick.getYButtonReleased();

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

        //Switch statement to determine controls for the gunner
        switch (gunnerScheme) {
            case "Fun Mode":

                discoToggle = gunnerStick.getStartButtonReleased();
                break;

            default:
                shoot = gunnerStick.getBButton();

                if ((gunnerStick.getTriggerAxis(Hand.kRight) >= 0.75) && (gunnerStick.getTriggerAxis(Hand.kLeft) >= 0.75) && !overriding) {
                    overrideSafeties = !overrideSafeties;
                    overriding = true;
                } else if ((gunnerStick.getTriggerAxis(Hand.kRight) <= 0.75) && (gunnerStick.getTriggerAxis(Hand.kLeft) <= 0.75)) {
                    overriding = false;
                }

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
    public boolean getDiscoButton(){
        return discoToggle;
    }
    public void setDriveScheme(String driveScheme){
        driverScheme = driveScheme;
    }
    public void setGunnerScheme(String gunnerScheme){
        this.gunnerScheme = gunnerScheme;
    }
	public boolean getShoot() {
		return shoot;
    }

    public boolean getOverride() {
        return overrideSafeties;
    }

    public boolean getClimbButton() {
        return climbToggle;
    }

    public boolean getManualClimb() {
        return manualClimb;
    }

    public boolean getClimbRelease() {
        return climbRelease;
    }
    public boolean getIntakeToggle(){
        return intakeToggle;
    }
}
