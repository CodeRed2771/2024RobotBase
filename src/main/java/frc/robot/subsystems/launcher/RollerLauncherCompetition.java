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
    private CANSparkMax aimMotor;

    private SparkPIDController aimPIDController = null;
    
    private RelativeEncoder aimEncoder = null;
    private DutyCycleEncoder aimAbsoluteEncoder = null;

    public double aim_kP, aim_kI, aim_kD, aim_kIz, aim_kFF;
    
    public double aim_kMaxOutput, aim_kMinOutput, aim_maxRPM;
    

    private static final double ABS_FULL_BACK = 0.176;
    private static final double ABS_FULL_FORWARD = 0.440;
    private static final double DEG_TO_TICK = -1261.065;

    private static final double aimMargin = 10/360.0;
    private static final double ABS_BACK_STOP = ABS_FULL_BACK + aimMargin;
    private static final double ABS_FORWARD_STOP = ABS_FULL_FORWARD - aimMargin;

    private double aimBias = 0;

    private double rollerDegreesToTicks(double input_degrees){
        double rotations = (input_degrees + aimBias) / 360.0 ;
        return DEG_TO_TICK * rotations;
    }

    private double rawRollerRotationsToTicks(double raw_rotation){
        return DEG_TO_TICK * (MathUtil.clamp(raw_rotation , ABS_BACK_STOP, ABS_FORWARD_STOP) - ABS_FORWARD_STOP);
    }

    public RollerLauncherCompetition(Map<String,Integer> wiring, Map<String,Double> calibration) {
        super(wiring, calibration);

        aimBias = calibration.getOrDefault("aim bias", aimBias);

        aimMotor = new CANSparkMax(wiring.get("aim"), MotorType.kBrushless);
    
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
        aimEncoder.setPosition(rawRollerRotationsToTicks(aimAbsoluteEncoder.get()));
    }

    public void load(double power) {

        loaderMotor.set(-power*2.00);
        intakeMotor.set(-power);
    }

    public void unload() {

        loaderMotor.set(.5);
        intakeMotor.set(.5);
        loadState = LoaderState.Unloading;
    }

    @Override
    public void prime(LauncherSpeeds speedSetting) {
        super.prime(speedSetting);
        aim(speedSetting);
    }

    public boolean isPrimed() {
        boolean speedTracking = super.isPrimed();
        boolean aimTracking = true;

        return speedTracking && aimTracking;
    }
    double angleInTicks;
    @Override
    public void aim(double angle) {
        angleInTicks = rollerDegreesToTicks(angle);
        aimPIDController.setReference(angleInTicks, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Aim Absolute Encoder", aimAbsoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Aim Relative Encoder", aimEncoder.getPosition());
        SmartDashboard.putNumber("Aim Setpoint", angleInTicks);
    }
}