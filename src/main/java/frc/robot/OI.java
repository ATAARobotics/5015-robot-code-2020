package frc.robot;
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
    private boolean visionShoot;
    private boolean climbToggle;
    private boolean manualClimb;
    private boolean climbRelease;
    private boolean intakeToggle;
    private boolean intakeReverse;
    private boolean magazineReverse;
    private boolean resetBalls;

    //Gunner variables
    private XboxController gunnerStick = new XboxController(1);
    private String gunnerScheme = "Default";
    private boolean manualShoot;
    private boolean overrideSafeties = false;
    private boolean overriding = false;
    private boolean discoToggle;
    private boolean toggleIntakeMotors;

    public OI() {

    }
    //periodic function to update controller input
    public void checkInputs() {

        gearShift = driveStick.getXButtonReleased();
        slow = driveStick.getAButtonReleased();

        //climbToggle = driveStick.getBumperReleased(Hand.kLeft);
        climbToggle = false;
        climbRelease = driveStick.getStartButton();

        resetBalls = gunnerStick.getStartButtonReleased();
        manualClimb = driveStick.getBumper(Hand.kLeft) && driveStick.getBumper(Hand.kRight);
        visionShoot = gunnerStick.getBButtonReleased();
        intakeToggle = gunnerStick.getAButtonReleased();
        intakeReverse = gunnerStick.getXButtonReleased();
        magazineReverse = gunnerStick.getBackButtonReleased();
        toggleIntakeMotors = gunnerStick.getBumperReleased(Hand.kRight);

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
                manualShoot = gunnerStick.getYButton();

                // if ((gunnerStick.getTriggerAxis(Hand.kRight) >= 0.75) && (gunnerStick.getTriggerAxis(Hand.kLeft) >= 0.75) && !overriding) {
                //     overrideSafeties = !overrideSafeties;
                //     overriding = true;
                // } else if ((gunnerStick.getTriggerAxis(Hand.kRight) <= 0.75) && (gunnerStick.getTriggerAxis(Hand.kLeft) <= 0.75)) {
                //     overriding = false;
                // }

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

    public boolean getVisionShoot() {
        return visionShoot;
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
	public boolean getManualShoot() {
		return manualShoot;
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
    public boolean getIntakeReverse(){
        return intakeReverse;
    }
    public boolean getMagazineReverse(){
        return magazineReverse;
    }
    public boolean getBallReset(){
        return resetBalls;
    }
    public boolean getToggleIntakeMotors(){
        return toggleIntakeMotors;
    }
}
