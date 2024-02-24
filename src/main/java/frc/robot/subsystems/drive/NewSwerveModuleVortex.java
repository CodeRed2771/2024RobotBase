// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewSwerveModuleVortex extends SwerveModuleBase {
  private CANSparkFlex m_driveMotor;
  private CANSparkMax m_turningMotor;

  private SparkPIDController m_drivePIDController;

  /**
   * A RelativeEncoder object is constructed using the GetEncoder() method on an existing CANSparkMax object. The
   * assumed encoder type is the hall effect, or a sensor type and counts per revolution can be passed in to specify a
   * different kind of sensor. Here, it's a quadrature encoder with 4096 CPR.
   */
  private AnalogEncoder turnAbsEncoder;
  private RelativeEncoder m_driveEncoder;

  private double velCmd = 0;
  private double speedCmd = 0;
  private double rotCmd = 0;

  private static final double kWheelRadius = 2.0 * 2.54/100;// Meters needed for kinematics
  private static final double kDriveMaxRPM = 6000;

  public static final double kMaxAngularSpeed =  1000* 2 *  (2 * Math.PI); // radians per second
  private static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 1000 * 5 * (2 * Math.PI); // radians per second squared

  /** Creates a new NewSwerveModuleVortex. */
  public NewSwerveModuleVortex(int driveMotorID, int turnMotorID, int turnAbsEncID, String moduleID) {
    super();

    this.setName(moduleID);

    // Use addRequirements() here to declare subsystem dependencies.
    m_driveMotor = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setOpenLoopRampRate(.5);
    m_driveMotor.setSmartCurrentLimit(40);
    m_driveMotor.setIdleMode(IdleMode.kBrake);

    m_turningMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    turnAbsEncoder = new AnalogEncoder(new AnalogInput(turnAbsEncID));
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setOpenLoopRampRate(1);
    m_turningMotor.setSmartCurrentLimit(40);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    /************ SET PID VALUES HERE ******************/
    m_drivePIDController = new SparkPIDController(m_driveMotor);
    m_turningPIDController = new ProfiledPIDController(5.0, 0.0, 0.5,
        new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
    m_turningPIDController.setIntegratorRange(-0.5,0.5);

    m_driveFeedforward = new SimpleMotorFeedforward(0.0, 12.0);
    m_turnFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(kWheelRadius);
    m_driveEncoder.setVelocityConversionFactor(1/kDriveMaxRPM);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();

  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getRotation());
  }

  public Rotation2d getAbsOrentation() {
    return Rotation2d.fromRotations(turnAbsEncoder.getAbsolutePosition());
  }

  @Override
  public Rotation2d getRotation() {
    return Rotation2d.fromRotations(turnAbsEncoder.get());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getRotation());
  }

  @Override
  protected void reportCmd(SwerveModuleState targetState){
    velCmd = targetState.speedMetersPerSecond;
    rotCmd = targetState.angle.getRotations();
  }

  protected void applySwerveModuleState(double driveCmd, double turnCmd) {
    speedCmd = driveCmd;

    if (isArmed()) {
      m_driveMotor.setVoltage(driveCmd);
      m_turningMotor.setVoltage(turnCmd);
    }
  }

  private int decimate = 10;

  @Override
  public void periodic() {
    SmartDashboard.putNumber(this.getName() + " Cmd V", velCmd);
    SmartDashboard.putNumber(this.getName() + " Cmd S", speedCmd);
    SmartDashboard.putNumber(this.getName() + " Vel", m_driveEncoder.getVelocity());
    SmartDashboard.putNumber(this.getName() + " Pos", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(this.getName() + " Cmd R", rotCmd);
    SmartDashboard.putNumber(this.getName() + " Abs R", turnAbsEncoder.getAbsolutePosition());
  }

  @Override
  public void doArm() {
    resetEncoders();
  }

  @Override
  public void doDisarm() {
    applySwerveModuleState(0, 0);
  }

  public void setTurnOffset(double value) {
    turnAbsEncoder.setPositionOffset(value);
  }

  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_driveEncoder.getPosition();
    m_driveEncoder.getVelocity();
  }

  public double getDriveEnc() {
		return m_driveEncoder.getPosition();
	}

  // auto related routines
  
  public void setDriveMaxAccel(final int accel) {
    m_drivePIDController.setSmartMotionMaxAccel(accel, 0);
  }
  public void setDriveMaxVelocity(final int velocity) {
    m_drivePIDController.setSmartMotionMaxVelocity(velocity, 0);
  }

}
