package frc.robot.subsystems.drive;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Calibration;
import frc.robot.Wiring;

public class PracticeDriveTrain extends DriveSubsystem {

    private SwerveModule moduleA;
    private SwerveModule moduleB;
    private SwerveModule moduleC;
    private SwerveModule moduleD;

        /* Use a singleton design pattern to assist in migrating from ubiquitous static class operations */
    private static class PracticeDriveTrainSingleton {
        private static final PracticeDriveTrain instance = new PracticeDriveTrain();
    }

    public static PracticeDriveTrain getInstance(){
        return PracticeDriveTrainSingleton.instance;
    }


    private PracticeDriveTrain(){

        Calibration.loadSwerveCalibration();

        moduleA = new SwerveModuleVortex(Calibration.DT_A_DRIVE_ID, Calibration.DT_A_TURN_ID, Wiring.TURN_ABS_ENC_A, Calibration.getTurnZeroPos('A'), "A"); // Front right
        moduleB = new SwerveModuleVortex(Calibration.DT_B_DRIVE_ID, Calibration.DT_B_TURN_ID, Wiring.TURN_ABS_ENC_B, Calibration.getTurnZeroPos('B'), "B"); // Back left
        moduleC = new SwerveModuleVortex(Calibration.DT_C_DRIVE_ID, Calibration.DT_C_TURN_ID, Wiring.TURN_ABS_ENC_C, Calibration.getTurnZeroPos('C'), "C"); // Back right
        moduleD = new SwerveModuleVortex(Calibration.DT_D_DRIVE_ID, Calibration.DT_D_TURN_ID, Wiring.TURN_ABS_ENC_D, Calibration.getTurnZeroPos('D'), "D"); // Front left

        this.addChild(moduleA.getName(), moduleA);
        this.addChild(moduleB.getName(), moduleB);
        this.addChild(moduleC.getName(), moduleC);
        this.addChild(moduleD.getName(), moduleD);

        SmartDashboard.putNumber("TURN P", Calibration.getTurnP());
        SmartDashboard.putNumber("TURN I", Calibration.getTurnI());
        SmartDashboard.putNumber("TURN D", Calibration.getTurnD());

        Calibration.initializeSmartDashboard(); 

    }

    @Override
    public void doArm() {
        stopDriveAndTurnMotors();
        reset();
    }

    @Override
    public void doDisarm() {
        stopDriveAndTurnMotors();
    }

    @Override
    public void periodic(){
        smartDashboardOutputABSRotations();
        showTurnEncodersOnDash();

        if(isDisarmed()){
            if (Calibration.shouldCalibrateSwerve()) {
                double[] pos = getAllAbsoluteTurnOrientations();
                Calibration.saveSwerveCalibration(pos[0], pos[1], pos[2], pos[3]);
            }
          
            // see if we want to reset the calibration to whatever is in the program
            // basically setting "Delete Swerve Calibration" to true will trigger
            // this, which deletes the calibration file.
            Calibration.checkIfShouldDeleteCalibration();
        }
    }

    @Override
    public void reset(){
        allowTurnEncoderReset();
        resetTurnEncoders();
    }

    // define robot dimensions. L=wheel base W=track width
    private static final double l = 19, w = 19, r = Math.sqrt((l * l) + (w * w));

    public void resetTurnZeroToCurrentPos() {
   		// sets the known "zero position" to be whatever we're at now.
		// should only be called when the modules are KNOWN to be straight.

        moduleA.resetZeroPosToCurrentPos();
        moduleB.resetZeroPosToCurrentPos();
        moduleC.resetZeroPosToCurrentPos();
        moduleD.resetZeroPosToCurrentPos();
    }

    public void resetTurnReversedFlag() {
        moduleA.resetTurnReversedFlag();
        moduleB.resetTurnReversedFlag();
        moduleC.resetTurnReversedFlag();
        moduleD.resetTurnReversedFlag();
    }

    public void setDrivePower(double modAPower, double modBPower, double modCPower, double modDPower) {
        moduleA.setDrivePower(modAPower);
        moduleB.setDrivePower(modBPower);
        moduleC.setDrivePower(modCPower);
        moduleD.setDrivePower(modDPower);
    }

    public void setDriveMMAccel(int accel) {
        moduleA.setDriveMMAccel(accel);
        moduleB.setDriveMMAccel(accel);
        moduleC.setDriveMMAccel(accel);
        moduleD.setDriveMMAccel(accel);
    }

    public void setDriveMMVelocity(int velocity) {
        moduleA.setDriveMMVelocity(velocity);
        moduleB.setDriveMMVelocity(velocity);
        moduleC.setDriveMMVelocity(velocity);
        moduleD.setDriveMMVelocity(velocity);
    }

    public boolean hasDriveCompleted(double inchesError) {
        // just checking two of the modules to see if they are within the desired accuracy
        return moduleB.hasDriveCompleted(inchesError) && moduleA.hasDriveCompleted(inchesError);
    }

    public boolean hasDriveCompleted() {
        return hasDriveCompleted(0.25);
    }

    public void setTurnPower(double modAPower, double modBPower, double modCPower, double modDPower) {
        moduleA.setTurnPower(modAPower);
        moduleB.setTurnPower(modBPower);
        moduleC.setTurnPower(modCPower);
        moduleD.setTurnPower(modDPower);
    }

    public void setTurnOrientation(double modAPosition, double modBPosition, double modCPosition,
            double modDPosition) {
        setTurnOrientation(modAPosition, modBPosition, modCPosition, modDPosition, false);
    }

    public void setTurnOrientation(double modAPosition, double modBPosition, double modCPosition,
            double modDPosition, boolean optimizeTurn) {

        // position is a value from 0 to 1 that indicates
        // where in the rotation of the module the wheel should be set.
        // e.g. a value of .5 indicates a half turn from the zero position

        moduleA.setTurnOrientation(modAPosition, optimizeTurn);
        moduleB.setTurnOrientation(modBPosition, optimizeTurn);
        moduleC.setTurnOrientation(modCPosition, optimizeTurn);
        moduleD.setTurnOrientation(modDPosition, optimizeTurn);

        SmartDashboard.putNumber("A pos call", round(modAPosition,3));
        SmartDashboard.putNumber("B pos call", round(modBPosition,3));
        SmartDashboard.putNumber("C pos call", round(modCPosition,3));
        SmartDashboard.putNumber("D pos call", round(modDPosition,3));
        // SmartDashboard.putNumber("Mod A ABS Rotations", moduleA.getABSRotations());
        // SmartDashboard.putNumber("Mod B ABS Rotations", moduleB.getABSRotations());
        // SmartDashboard.putNumber("Mod C ABS Rotations", moduleC.getABSRotations());
        // SmartDashboard.putNumber("Mod D ABS Rotations", moduleD.getABSRotations());
    }

    public void setAllTurnOrientation(double position) {
        setTurnOrientation(position, position, position, position, true);
    }

    public void smartDashboardOutputABSRotations() {
        SmartDashboard.putNumber("Mod A ABS Rotations", moduleA.getABSRotations());
        SmartDashboard.putNumber("Mod B ABS Rotations", moduleB.getABSRotations());
        SmartDashboard.putNumber("Mod C ABS Rotations", moduleC.getABSRotations());
        SmartDashboard.putNumber("Mod D ABS Rotations", moduleD.getABSRotations());
        SmartDashboard.putString("Test", "Test");
    }

    /**
     * @param position     - position, 0 to 1, to turn to.
     * @param optimizeTurn - allow turn optimization
     */
    public void setAllTurnOrientation(double position, boolean optimizeTurn) {
        setTurnOrientation(position, position, position, position, optimizeTurn);
    }

    public void setAllDrivePosition(int position) {
        setDrivePosition(position, position, position, position);
    }

    public void setDrivePosition(double modAPosition, double modBPosition, double modCPosition,
            double modDPosition) {
        moduleA.setDrivePIDToSetPoint(modAPosition);
        moduleB.setDrivePIDToSetPoint(modBPosition);
        moduleC.setDrivePIDToSetPoint(modCPosition);
        moduleD.setDrivePIDToSetPoint(modDPosition);
    }

    public void addToAllDrivePositions(double ticks) {
        setDrivePosition(moduleA.getDriveEnc() + ((moduleA.modulesReversed() ? -1 : 1) * ticks),
                moduleB.getDriveEnc() + ((moduleB.modulesReversed() ? -1 : 1) * ticks),
                moduleC.getDriveEnc() + ((moduleC.modulesReversed() ? -1 : 1) * ticks),
                moduleD.getDriveEnc() + ((moduleD.modulesReversed() ? -1 : 1) * ticks));
    }

    public double getDriveEnc() {
        return (moduleA.getDriveEnc() + moduleB.getDriveEnc() + moduleC.getDriveEnc() + moduleD.getDriveEnc()) / 4;
    }

    public void autoSetRot(double rot) {
        swerveDrive(0, 0, rot);
    }

    public void setAllTurnPower(double power) {
        setTurnPower(power, power, power, power);
    }

    public void setAllDrivePower(double power) {
        setDrivePower(power, power, power, power);
    }

    public boolean isModuleATurnEncConnected() {
        return moduleA.isTurnEncConnected();
    }

    public boolean isModuleBTurnEncConnected() {
        return moduleB.isTurnEncConnected();
    }

    public boolean isModuleCTurnEncConnected() {
        return moduleC.isTurnEncConnected();
    }

    public boolean isModuleDTurnEncConnected() {
        return moduleD.isTurnEncConnected();
    }

    public void resetDriveEncoders() {
        moduleA.resetDriveEnc();
        moduleB.resetDriveEnc();
        moduleC.resetDriveEnc();
        moduleD.resetDriveEnc();
    }

    public void stopDriveAndTurnMotors() {
        moduleA.stopDriveAndTurnMotors();
        moduleB.stopDriveAndTurnMotors();
        moduleC.stopDriveAndTurnMotors();
        moduleD.stopDriveAndTurnMotors();
    }

    public void stopDrive() {
        moduleA.stopDrive();
        moduleB.stopDrive();
        moduleC.stopDrive();
        moduleD.stopDrive();
    }

    public void stopTurn() {
        moduleA.stopTurn();
        moduleB.stopTurn();
        moduleC.stopTurn();
        moduleD.stopTurn();
    }

    public static double angleToPosition(double angle) {
        if (angle < 0) {
            return .5d + ((180d - Math.abs(angle)) / 360d);
        } else {
            return angle / 360d;
        }
    }

    private boolean allowTurnEncoderReset = false;

    public void allowTurnEncoderReset() {
        allowTurnEncoderReset = true;
    }

    public void resetTurnEncoders() {
        if (allowTurnEncoderReset) {
            moduleA.resetTurnEncoder();
            moduleB.resetTurnEncoder();
            moduleC.resetTurnEncoder();
            moduleD.resetTurnEncoder();

            // new robot
			// moduleA.setEncPos((calculatePositionDifference(modAOff, Calibration.GET_DT_A_ABS_ZERO())));
			// moduleB.setEncPos( (calculatePositionDifference(modBOff, Calibration.GET_DT_B_ABS_ZERO())));
			// moduleC.setEncPos( (calculatePositionDifference(modCOff, Calibration.GET_DT_C_ABS_ZERO())));
			// moduleD.setEncPos( (calculatePositionDifference(modDOff, Calibration.GET_DT_D_ABS_ZERO())));

           
            allowTurnEncoderReset = false;
        }
    }

    public void setDriveBrakeMode(boolean b) {
        moduleA.setBrakeMode(b);
        moduleB.setBrakeMode(b);
        moduleC.setBrakeMode(b);
        moduleD.setBrakeMode(b);
    }

    public double getAverageTurnError() {
        return (Math.abs(moduleA.getTurnError()) + Math.abs(moduleB.getTurnError()) + Math.abs(moduleC.getTurnError())
                + Math.abs(moduleD.getTurnError())) / 4d;
    }

    /*
     * 
     * Drive methods
     */
    public void swerveDrive(double fwd, double strafe, double rot) {
        double a = strafe - (rot * (l / r));
        double b = strafe + (rot * (l / r));
        double c = fwd - (rot * (w / r));
        double d = fwd + (rot * (w / r));

        double ws1 = Math.sqrt((b * b) + (c * c)); // front_right (CHECK THESE
                                                   // AGAINST OUR BOT)
        double ws2 = Math.sqrt((b * b) + (d * d)); // front_left
        double ws3 = Math.sqrt((a * a) + (d * d)); // rear_left
        double ws4 = Math.sqrt((a * a) + (c * c)); // rear_right

        double wa1 = Math.atan2(b, c) * 180 / Math.PI;
        double wa2 = Math.atan2(b, d) * 180 / Math.PI;
        double wa3 = Math.atan2(a, d) * 180 / Math.PI;
        double wa4 = Math.atan2(a, c) * 180 / Math.PI;

        double max = ws1;
        max = Math.max(max, ws2);
        max = Math.max(max, ws3);
        max = Math.max(max, ws4);
        if (max > 1) {
            ws1 /= max;
            ws2 /= max;
            ws3 /= max;
            ws4 /= max;
        }

        // DriveTrain.setTurnOrientation(angleToPosition(wa4), angleToPosition(wa2), angleToPosition(wa1),
        //         angleToPosition(wa3), true);
        setTurnOrientation(angleToPosition(wa4), angleToPosition(wa2), angleToPosition(wa1),
            angleToPosition(wa3), true);
        setDrivePower(ws4, ws2, ws1, ws3);
    }

    public void showDriveEncodersOnDash() {
        SmartDashboard.putNumber("Mod A Drive Enc", (int)moduleA.getDriveEnc());
        SmartDashboard.putNumber("Mod B Drive Enc", (int)moduleB.getDriveEnc());
        SmartDashboard.putNumber("Mod C Drive Enc", (int)moduleC.getDriveEnc());
        SmartDashboard.putNumber("Mod D Drive Enc", (int)moduleD.getDriveEnc());

        SmartDashboard.putNumber("Mod A Drive Setpt", (int)moduleA.getCurrentDriveSetpoint());
        SmartDashboard.putNumber("Mod B Drive Setpt", (int)moduleB.getCurrentDriveSetpoint());
        SmartDashboard.putNumber("Mod C Drive Setpt", (int)moduleC.getCurrentDriveSetpoint());
        SmartDashboard.putNumber("Mod D Drive Setpt", (int)moduleD.getCurrentDriveSetpoint());
    }

    public void showTurnEncodersOnDash() {
        SmartDashboard.putNumber("TURN A RAW", round(moduleA.getTurnAbsolutePosition(), 3));
        SmartDashboard.putNumber("TURN B RAW", round(moduleB.getTurnAbsolutePosition(), 3));
        SmartDashboard.putNumber("TURN C RAW", round(moduleC.getTurnAbsolutePosition(), 3));
        SmartDashboard.putNumber("TURN D RAW", round(moduleD.getTurnAbsolutePosition(), 3));

        SmartDashboard.putNumber("TURN A ENC", round(moduleA.getTurnRelativePosition(),3));
        SmartDashboard.putNumber("TURN B ENC", round(moduleB.getTurnRelativePosition(),3));
        SmartDashboard.putNumber("TURN C ENC", round(moduleC.getTurnRelativePosition(),3));
        SmartDashboard.putNumber("TURN D ENC", round(moduleD.getTurnRelativePosition(),3));

        SmartDashboard.putNumber("TURN A POS", round(moduleA.getTurnPosition(), 2));
        SmartDashboard.putNumber("TURN B POS", round(moduleB.getTurnPosition(), 2));
        SmartDashboard.putNumber("TURN C POS", round(moduleC.getTurnPosition(), 2));
        SmartDashboard.putNumber("TURN D POS", round(moduleD.getTurnPosition(), 2));

        SmartDashboard.putNumber("TURN A ANGLE", round(moduleA.getTurnAngle(), 0));
        SmartDashboard.putNumber("TURN B ANGLE", round(moduleB.getTurnAngle(), 0));
        SmartDashboard.putNumber("TURN C ANGLE", round(moduleC.getTurnAngle(), 0));
        SmartDashboard.putNumber("TURN D ANGLE", round(moduleD.getTurnAngle(), 0));

        SmartDashboard.putNumber("TURN A ERR", round(moduleA.getTurnError(),2));
        SmartDashboard.putNumber("TURN B ERR", round(moduleB.getTurnError(),2));
        SmartDashboard.putNumber("TURN C ERR", round(moduleC.getTurnError(),2));
		SmartDashboard.putNumber("TURN D ERR", round(moduleD.getTurnError(),2));

        SmartDashboard.putNumber("TURN A ZERO", moduleA.getTurnZero());
        SmartDashboard.putNumber("TURN B ZERO", moduleB.getTurnZero());
        SmartDashboard.putNumber("TURN C ZERO", moduleC.getTurnZero());
        SmartDashboard.putNumber("TURN D ZERO", moduleD.getTurnZero());
		
		//SmartDashboard.putNumber("TURN A SETPOINT", moduleA.getTurnSetpoint());
        SmartDashboard.putNumber("A Encoder Raw Value", moduleA.getTurnRelativePosition());
        SmartDashboard.putNumber("B Encoder Raw Value", moduleB.getTurnRelativePosition());
        SmartDashboard.putNumber("C Encoder Raw Value", moduleC.getTurnRelativePosition());
        SmartDashboard.putNumber("D Encoder Raw Value", moduleD.getTurnRelativePosition());
    }

    @Override
    public void driveSpeedControl(double fwd, double strafe, double rotate){
        if (Math.abs(rotate) < 0.01)
            rotate = 0;

        if (Math.abs(fwd) < .15 && Math.abs(strafe) < .15 && Math.abs(rotate) < 0.01) {
            setDriveBrakeMode(true);
            stopDrive();
        } else {
            setDriveBrakeMode(false);
            swerveDrive(fwd, strafe, rotate);
        }
    }

    public void tankDrive(double left, double right) {
        setAllTurnOrientation(0);
        setDrivePower(right, left, right, left);
    }

    public double[] getAllAbsoluteTurnOrientations() {
        return new double[] { moduleA.getTurnAbsolutePosition(), moduleB.getTurnAbsolutePosition(),
                moduleC.getTurnAbsolutePosition(), moduleD.getTurnAbsolutePosition() };
    }

    public void setDrivePIDValues(double p, double i, double d, double f) {
        moduleA.setDrivePIDValues(p, i, d, f);
        moduleB.setDrivePIDValues(p, i, d, f);
        moduleC.setDrivePIDValues(p, i, d, f);
        moduleD.setDrivePIDValues(p, i, d, f);
    }

    public void setTurnPIDValues(double p, double i, double d, double iZone, double f) {
        moduleA.setTurnPIDValues(p, i, d, iZone, f);
        moduleB.setTurnPIDValues(p, i, d, iZone, f);
        moduleC.setTurnPIDValues(p, i, d, iZone, f);
        moduleD.setTurnPIDValues(p, i, d, iZone, f);
    }

    private static Double round(Double val, int scale) {
        return new BigDecimal(val.toString()).setScale(scale, RoundingMode.HALF_UP).doubleValue();
    }
}
