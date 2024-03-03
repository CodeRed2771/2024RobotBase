// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewSwerveModuleVortex extends SwerveModuleBase {
  // Gains are zero'd in abstract class. Update in particular Swerve constructor
  protected SparkPIDController m_drivePIDController;
  protected SparkPIDController m_turningPIDController;
  
  private CANSparkFlex m_driveMotor;
  private CANSparkMax m_turningMotor;
  /**
   * A RelativeEncoder object is constructed using the GetEncoder() method on an existing CANSparkMax object. The
   * assumed encoder type is the hall effect, or a sensor type and counts per revolution can be passed in to specify a
   * different kind of sensor. Here, it's a quadrature encoder with 4096 CPR.
   */
  private AnalogEncoder turnAbsEncoder;
  private RelativeEncoder m_driveEncoder;
  private RelativeEncoder m_turnEncoder;

  private double velCmd = 0;
  private double rotCmd = 0;
  private double rawRotCmd = 0;
  protected class PIDGains {
    public double kP=0.0;
    public double kI=0.0;
    public double kD=0.0;
    public double kIz=0.0;
    public double kFF=0.0;
    public double maxVel=0.0;
    public double maxAcc=0.0;
  }

  private enum DriveMode {
    SpeedMode,
    PositionMode,
    Disabled
  }

  private DriveMode currentControlMode = DriveMode.Disabled;
  private double targetDrivePosition = 0.0;

  private PIDGains driveGains;
  private PIDGains turnGains;

  private static final double kWheelRadius = 1.88 * 2.54/100;// Meters needed for kinematics
  private static final double kDriveMaxRPM = 6600;

  public static final double kMaxAngularSpeed =  1000* 2 *  (2 * Math.PI); // radians per second
  private static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 1000 * 5 * (2 * Math.PI); // radians per second squared

  /** Creates a new NewSwerveModuleVortex. */
  public NewSwerveModuleVortex(int driveMotorID, int turnMotorID, int turnAbsEncID, String moduleID) {
    super();

    this.setName(moduleID);

    Timer.delay(0.15);

    // Use addRequirements() here to declare subsystem dependencies.
    m_driveMotor = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();

    Timer.delay(0.5);

    m_driveMotor.setOpenLoopRampRate(.5);
    m_driveMotor.setSmartCurrentLimit(30);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setInverted(false);
    m_driveMotor.burnFlash();
    Timer.delay(0.5);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(kWheelRadius);
    m_driveEncoder.setVelocityConversionFactor(1/kDriveMaxRPM);


    m_turningMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    Timer.delay(0.5);
    m_turningMotor.setOpenLoopRampRate(1);
    m_turningMotor.setSmartCurrentLimit(30);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setInverted(false);
    m_turningMotor.burnFlash(); 
    Timer.delay(0.5);

    turnAbsEncoder = new AnalogEncoder(new AnalogInput(turnAbsEncID));
    m_turnEncoder = m_turningMotor.getEncoder();

    m_turnEncoder.setPositionConversionFactor(1.0/12.805);
    m_turnEncoder.setVelocityConversionFactor(1.0/12.805);

    /************ SET PID VALUES HERE ******************/
    driveGains = new PIDGains();
    driveGains.kP = 0.4;
    driveGains.kI = 0.0;
    driveGains.kD = 0.0;
    driveGains.kIz = 0.0;
    driveGains.kFF = 1.0;
    driveGains.maxVel = 1.0;
    driveGains.maxAcc = 10.0;

    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setP(driveGains.kP);
    m_drivePIDController.setI(driveGains.kI);
    m_drivePIDController.setIZone(driveGains.kIz);
    m_drivePIDController.setD(driveGains.kD);
    m_drivePIDController.setFF(driveGains.kFF);
    m_drivePIDController.setSmartMotionMaxVelocity(driveGains.maxVel, 0);
    m_drivePIDController.setSmartMotionMaxAccel(driveGains.maxAcc, 0);
    m_drivePIDController.setOutputRange(-1,1);


    turnGains = new PIDGains();
    turnGains.kP = 3.0;
    turnGains.kI = 0.0;
    turnGains.kD = 0.0;
    turnGains.kIz = 0.0;
    turnGains.kFF = 0.0;
    turnGains.maxVel = 7000.0;
    turnGains.maxAcc = 5000.0;


    m_turningPIDController = m_turningMotor.getPIDController();
    m_turningPIDController.setP(turnGains.kP);
    m_turningPIDController.setI(turnGains.kI);
    m_turningPIDController.setIZone(turnGains.kIz);
    m_turningPIDController.setD(turnGains.kD);
    m_turningPIDController.setFF(turnGains.kFF);
    m_turningPIDController.setSmartMotionMaxVelocity(turnGains.maxVel, 0);
    m_turningPIDController.setSmartMotionMaxAccel(turnGains.maxAcc, 0);
    m_turningPIDController.setOutputRange(-1, 1);

    m_turningPIDController.setFeedbackDevice(m_turnEncoder);

    resetEncoders();
    SmartDashboard.putBoolean("Enable Drive Tuning", false);
    SmartDashboard.putBoolean("Enable Turn Tuning", false);
    SmartDashboard.putBoolean("Reset Encoders", false);

  }

  public void updateSwerveState(){
    curDriveSpeed = m_driveEncoder.getVelocity();
    curDriveDistance = m_driveEncoder.getPosition();
    curTurnAngle = turnAbsEncoder.getAbsolutePosition();
  }

  @Override
  protected void commandSwerveState(SwerveModuleState targetState) {

    double delta = targetState.angle.getRotations() - curTurnAngle;
    delta = delta - Math.round(delta);
    targetState.angle = Rotation2d.fromRotations(curTurnAngle + delta);

    commandSwerveMotors(targetState.speedMetersPerSecond, targetState.angle.getRotations());
  }

  protected void commandSwervePositionOffset(SwerveModulePosition targetPosition) {
    double delta = targetPosition.angle.getRotations();
    SwerveModulePosition current = getPosition();
    delta = delta - Math.round(delta);
    targetPosition.angle = Rotation2d.fromRotations(delta);
    commandSwerveMotorsPosition(current.distanceMeters + targetPosition.distanceMeters, targetPosition.angle.getRotations());
  }

  @Override
  public SwerveModulePosition getPosition() {
    curDriveDistance = m_driveEncoder.getPosition();
    return new SwerveModulePosition(curDriveDistance, getRotation());
  }

  public Rotation2d getAbsOrentation() {
    return Rotation2d.fromRotations(turnAbsEncoder.getAbsolutePosition());
  }

  @Override
  public Rotation2d getRotation() {
    Rotation2d ang = Rotation2d.fromRotations(m_turnEncoder.getPosition());
    curTurnAngle = ang.getRotations();
    return ang;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Override
  public SwerveModuleState getState() {
    curDriveSpeed = m_driveEncoder.getVelocity();

    return new SwerveModuleState(curDriveSpeed, getRotation());
  }

  @Override
  protected void logState(SwerveModuleState cmd){
    rawRotCmd = cmd.angle.getRotations();
  }

  protected void logSpeedCmd(double driveCmd, double turnCmd){
    velCmd = driveCmd;
    rotCmd = turnCmd;
  }

  protected void commandSwerveMotors(double driveCmd, double turnCmd) {
    logSpeedCmd(driveCmd,turnCmd);

    if (isArmed()) {
      currentControlMode = DriveMode.SpeedMode;
      m_drivePIDController.setReference(driveCmd, ControlType.kVelocity);
      m_turningPIDController.setReference(turnCmd, ControlType.kPosition);
    }
  }

  protected void commandSwerveMotorsPosition(double driveCmd, double turnCmd) {
    logSpeedCmd(driveCmd,turnCmd);

    if (isArmed()) {
      targetDrivePosition = driveCmd;
      currentControlMode = DriveMode.PositionMode;
      m_drivePIDController.setReference(driveCmd, ControlType.kSmartMotion);
      m_turningPIDController.setReference(turnCmd, ControlType.kPosition);
    }
  }

  protected double offsetDriveError(){
    double error = 0.0;
    if(currentControlMode == DriveMode.PositionMode)
      error = (targetDrivePosition - m_driveEncoder.getPosition());
    return error;
  }

  protected boolean atSwervePosition(double allowedError) {
    boolean result = true;
    if (currentControlMode == DriveMode.PositionMode && (Math.abs(offsetDriveError()) >= allowedError)) {
      result = false;
    }
    return result;
  }
  
  private void postTuneParams(String prefix, PIDGains gains){
    SmartDashboard.putNumber(prefix + " P", gains.kP);
    SmartDashboard.putNumber(prefix + " I", gains.kI);
    SmartDashboard.putNumber(prefix + " D", gains.kD);
    SmartDashboard.putNumber(prefix + " I Zone", gains.kIz);
    SmartDashboard.putNumber(prefix + " FF", gains.kFF);
    SmartDashboard.putNumber(prefix + " Max Velocity", gains.maxVel);
    SmartDashboard.putNumber(prefix + " Max Acceleration", gains.maxAcc);
  }
  private void handleTuneParams(String prefix, PIDGains gains, SparkPIDController controller){
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber(prefix + " P", 0);
    double i = SmartDashboard.getNumber(prefix + " I", 0);
    double d = SmartDashboard.getNumber(prefix + " D", 0);
    double iz = SmartDashboard.getNumber(prefix + " I Zone", 0);
    double ff = SmartDashboard.getNumber(prefix + " FF", 0);
    double maxV = SmartDashboard.getNumber(prefix + " Max Velocity", 0);
    double maxA = SmartDashboard.getNumber(prefix + " Max Acceleration", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != gains.kP)) {
      controller.setP(p);
      gains.kP = p;
    }
    if ((i != gains.kI)) {
      controller.setI(i);
      gains.kI = i;
    }
    if ((d != gains.kD)) {
      controller.setD(d);
      gains.kD = d;
    }
    if ((iz != gains.kIz)) {
      controller.setIZone(iz);
      gains.kIz = iz;
    }
    if ((ff != gains.kFF)) {
      controller.setFF(ff);
      gains.kFF = ff;
    }
    if ((maxV != gains.maxVel)) {
      controller.setSmartMotionMaxVelocity(maxV, 0);
      gains.maxVel = maxV;
    }
    if ((maxA != gains.maxAcc)) {
      controller.setSmartMotionMaxAccel(maxA, 0);
      gains.maxAcc = maxA;
    }
  }

  private boolean bEnableTurnTuning = false;
  private boolean bEnableDriveTuning = false;
  private int decimate = 10;

  private void checkTuningEnables() {
    boolean bTurn = SmartDashboard.getBoolean("Enable Turn Tuning", false);
    if (!bEnableTurnTuning && bTurn) {
      postTuneParams("Turn", turnGains);
    }
    bEnableTurnTuning = bTurn;
    if (bEnableTurnTuning) {
      handleTuneParams("Turn", turnGains, m_turningPIDController);

      SmartDashboard.putNumber(this.getName() + " Raw Cmd R", rawRotCmd);
      SmartDashboard.putNumber(this.getName() + " Cmd R", rotCmd);
      SmartDashboard.putNumber(this.getName() + " Abs R", turnAbsEncoder.getAbsolutePosition());
      SmartDashboard.putNumber(this.getName() + " Rel R", m_turnEncoder.getPosition());
      SmartDashboard.putNumber(this.getName() + " T Output", m_turningMotor.getAppliedOutput());
    }

    boolean bDrive = SmartDashboard.getBoolean("Enable Drive Tuning", false);
    if (!bEnableDriveTuning && bDrive) {
      postTuneParams("Drive", driveGains);
    }
    bEnableDriveTuning = bDrive;
    if (bEnableDriveTuning) {
      handleTuneParams("Drive", driveGains, m_drivePIDController);

      SmartDashboard.putNumber(this.getName() + " Cmd V", velCmd);
      SmartDashboard.putNumber(this.getName() + " Vel", m_driveEncoder.getVelocity());
      SmartDashboard.putNumber(this.getName() + " Pos", m_driveEncoder.getPosition());
      SmartDashboard.putNumber(this.getName() + " D Output", m_driveMotor.getAppliedOutput());
    }
  }

  @Override
  public void periodic() {

    if (decimate == 0) {
      decimate = 10;
      checkTuningEnables();
    } else {
      decimate--;
    }
  }

  @Override
  public void doArm() {
    resetEncoders();
  }

  @Override
  public void doDisarm() {
    commandSwerveMotors(0, 0);
    currentControlMode = DriveMode.Disabled;

  }

  public void setTurnOffset(double value) {
    turnAbsEncoder.setPositionOffset(value);
    resetEncoders();
  }

  public void resetEncoders() {
    double offset = turnAbsEncoder.getPositionOffset();
    m_driveEncoder.setPosition(0);
    m_driveEncoder.getPosition();
    m_driveEncoder.getVelocity();
    turnAbsEncoder.reset();
    turnAbsEncoder.setPositionOffset(offset);
    m_turnEncoder.setPosition(turnAbsEncoder.get());
  }
}
