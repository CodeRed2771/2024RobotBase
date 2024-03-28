// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/** Add your docs 
 * When load is command - run loader motor until note is in loader position
 * stop on sensor tells us we have note
 * hold then load for firing 
 * 
 * call prime to get motors up to speed 
 * 
 * Fire runs load for time if isPrimed until sensor says otherwise
 * 
 * 
 */
public class RollerLauncherCompetition extends RollerLauncher {

    public enum LauncherPresets {
        OFF(0,30,0),
        AMP(750, 95,0), // max back
        PICKUP(0, 38, 0),
        SAFE_ZONE(3400, 30,0),
        SUBWOOFER(2750, 49,0),
        CLIMB(0, 80,0),
        STOW(0,10,0),
        MAX_ANGLE(0, 75,0);

        private double speed;
        private double angle;
        private double bias;
        
        private LauncherPresets(double newSpeed, double newAngle,double newBias) {
            speed = newSpeed;
            angle = newAngle;
            bias = newBias;
        }

        public double getSpeed() {
            return speed;
        }
        public double getAngle() {
            return angle;
        }
        public double getBias() {
            return bias;
        }
    }
 
    private CANSparkMax aimMotor;

    private SparkPIDController aimPIDController = null;
    
    private RelativeEncoder aimEncoder = null;
    private DutyCycleEncoder aimAbsoluteEncoder = null;

    public double aim_kP, aim_kI, aim_kD, aim_kIz, aim_kFF;
    
    public double aim_kMaxOutput, aim_kMinOutput, aim_maxRPM;
    

    private static final double ABS_FULL_BACK = 0.166;
    private static final double ABS_FULL_FORWARD = 0.430;
    private static final double DEG_TO_TICK = -1261.065;

    private static final double aimMargin = 0/360.0; // no margin anymore
    private static final double ABS_BACK_STOP = ABS_FULL_BACK + aimMargin;
    private static final double ABS_FORWARD_STOP = ABS_FULL_FORWARD - aimMargin;
    private static final double AIM_MAX_RANGE = MathUtil.inputModulus(ABS_FORWARD_STOP - ABS_BACK_STOP,0,1);

    private double aimBias = 0;

    private double rollerDegreesToTicks(double input_degrees){
        double rotations = (input_degrees + aimBias) / 360.0 ;
        return DEG_TO_TICK * MathUtil.clamp(rotations, 0, AIM_MAX_RANGE);
    }

    private double rawRollerRotationsToTicks(double raw_rotation){
        return DEG_TO_TICK * (ABS_FULL_FORWARD - raw_rotation);
    }

    private double rawRollerRotationsToAngle(double raw_rotation){
        return 360.0 * (ABS_FULL_FORWARD - raw_rotation);
    }

    public RollerLauncherCompetition(Map<String,Integer> wiring, Map<String,Double> calibration) {
        super(wiring, calibration);

        aimBias = calibration.getOrDefault("aim bias", aimBias);

        aimMotor = new CANSparkMax(wiring.get("aim"), MotorType.kBrushless);
        aimMotor.setSmartCurrentLimit(10);
        aimAbsoluteEncoder = new DutyCycleEncoder(wiring.get("aim encoder"));
        aimEncoder =  aimMotor.getEncoder();

        aimPIDController = aimMotor.getPIDController();

        aim_kP = .1/5 ; 
        aim_kI = 0;
        aim_kD = 0; 
        aim_kIz = 0; 
        aim_kFF = 0.0; 
        aim_kMaxOutput = 1; 
        aim_kMinOutput = -1;

        resetAimEncoder();
        
        aimPIDController.setP(aim_kP);
        aimPIDController.setI(aim_kI);
        aimPIDController.setD(aim_kD);
        aimPIDController.setFF(aim_kFF);
        aimPIDController.setIZone(aim_kIz);
        aimPIDController.setOutputRange(aim_kMinOutput, aim_kMaxOutput);

        aimMotor.burnFlash();
    }

    @Override
    protected void doArm(){
        super.doArm();
        resetAimEncoder();
    }

    private void resetAimEncoder() {
        aimEncoder.setPosition(rawRollerRotationsToTicks(aimAbsoluteEncoder.getAbsolutePosition()));
    }

    @Override
    public void load(double power) {

        loaderMotor.set(-power*2.00);
        intakeMotor.set(-power);
    }

    @Override
    public void unload() {

        loaderMotor.set(.5);
        intakeMotor.set(.5);
        loadState = LoaderState.Unloading;
    }

    public double getAngle(){
        return rawRollerRotationsToAngle(aimAbsoluteEncoder.getAbsolutePosition());
    }

    public void aim(LauncherPresets preset) {
        prime(preset.getSpeed(),preset.getBias());
        aim(preset.getAngle());
    }

    public boolean isPrimed() {
        boolean speedTracking = super.isPrimed();
        boolean aimTracking = true;

        return speedTracking && aimTracking;
    }

    double angleInTicks;

    public void reset() {
        angleInTicks = aimEncoder.getPosition();
    }

    @Override
    public void aim(double angle) {
        angleInTicks = rollerDegreesToTicks(angle);
        aimPIDController.setReference(angleInTicks, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Aim Angle", getAngle());
        SmartDashboard.putNumber("Aim Absolute Encoder", aimAbsoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Aim Relative Encoder", aimEncoder.getPosition());
        SmartDashboard.putNumber("Aim Setpoint", angleInTicks);
    }

    public void postTuneParameters(){
        SmartDashboard.putNumber("Aim Bias",aimBias);
    }

    public void handleTuneParameters(){
        aimBias = SmartDashboard.getNumber("Aim Bias",aimBias);
    }

}