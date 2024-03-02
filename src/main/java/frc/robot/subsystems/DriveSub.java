// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.controllers.PathFollowingController;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
public class DriveSub extends SubsystemBase {
  /** Creates a new DriveSub. */
  
  private final SwerveModule frontLeft = new SwerveModule(18, 19, 25,0.386475, true,"front left", 0.015, 2.4585);

  private final SwerveModule frontRight = new SwerveModule(16, 17, 23,-0.668701, true,"front right",0.015, 2.4691);
  AHRS gyro = new AHRS();
  
  int counter=0;
  double dimension = Units.inchesToMeters(27/2);
  private final SwerveModule backLeft = new SwerveModule(20, 21, 26, 0.268311, true, "back left", 0.015, 2.4978);
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(dimension,dimension), new Translation2d(dimension,-dimension), new Translation2d(-dimension,dimension), new Translation2d(-dimension,-dimension));
  private final SwerveModule backRight = new SwerveModule(14, 15, 22,0.133301, true, "back right", 0.015, 2.46);
  private final SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  boolean keepTurning = false;
  
  PIDController pidController = new PIDController(0.011, 0, 0);
  SwerveDriveOdometry odometry;
  public DriveSub() {
    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0), getModulePositions());
    //HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(0.005,0,0), new PIDConstants(5, 0, 0),Constants.TeleOp.maxSpeed, Math.sqrt(2*Constants.TeleOp.robotRadius*Constants.TeleOp.robotRadius), new ReplanningConfig());
    
    pidController.enableContinuousInput(-180, 180);
    
    robotRelative(0, 0, 0);
    //AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, this::getSpeeds, this::setModuleStates, pathFollowerConfig, ()->false, this);
  }

  @Override
  public void periodic() {
    counter+=1;
    ChassisSpeeds speeds = new ChassisSpeeds(1, -5,0);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SmartDashboard.putNumber("module angles", Math.toDegrees(moduleStates[0].angle.getRadians()));
    SmartDashboard.putNumber("yaw",getHeading());
    // This method will be called once per scheduler run
    
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    return modulePositions;
  }

  

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public ChassisSpeeds getSpeeds(){
    return kinematics.toChassisSpeeds(getModuleStates());

  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] moduleStates = {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    return moduleStates;
  }

  public void resetOdometry(Pose2d pose){
    SmartDashboard.putString("inside odomtry", "yes");
    SwerveModulePosition[] modulePositions = getModulePositions();
    odometry.resetPosition(getRotation2d(), modulePositions, pose);
  }


  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void setSpeed(double speed, double turningSpeed){
    for (int i=0;i<modules.length;i++)
    modules[i].setSpeed(speed, turningSpeed);
    //frontRight.setSpeed(speed, turningSpeed);
    //backRight.setSpeed(speed, turningSpeed);
    //backLeft.setSpeed(speed, turningSpeed);
  }

  public double getHeading(){
    //invert gyro yaw reading
    return gyro.getYaw()*-1;
  }

  public void setModuleStates(ChassisSpeeds speed){
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speed);
    //desaturate wheelspeeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.TeleOp.maxSpeed);
    SmartDashboard.putNumber("module angles2", Math.toDegrees(moduleStates[0].angle.getRadians()));
    SmartDashboard.putNumber("module velocity", (moduleStates[0].speedMetersPerSecond));
    publisher.set(moduleStates);
    for (int i=0; i<modules.length;i++){
      modules[i].setState(moduleStates[i]);
    }

     //backRight.setState(moduleStates[3]);
  }

  public void robotRelative(double xSpeed, double ySpeed, double turningSpeed){
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    SmartDashboard.putNumber("xSpeed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ySpeed", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("turning rad", speeds.omegaRadiansPerSecond);
    setModuleStates(speeds);
    
  }

  public void fieldRelative(double xSpeed, double ySpeed, double turningSpeed){
    
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, new Rotation2d(getHeading()*Math.PI/180));
    SmartDashboard.putNumber("xSpeed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ySpeed", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("turning rad", speeds.omegaRadiansPerSecond);
    setModuleStates(speeds);
  }



  public void setAngle(double angle){
    for (int i=0;i<modules.length;i++){
      modules[i].setAngle(angle);
      SmartDashboard.putNumber("number", counter);
    }
  } 

  public void stopMotors(){
    setSpeed(0, 0);
  }

  public boolean atSetpoint(){
    return frontLeft.atSetpoint();
  } 

  //
  public void turnToAngle(double goal){
    
    if (goal>180){
      goal = goal-360;
    }

    double turnSpeed = pidController.calculate(getHeading(), goal);
    turnSpeed*=Constants.TeleOp.maxTurningRad*0.8;

    
    robotRelative(0, 0, turnSpeed);
    
    
    
  }

  public void setRobotRelative(ChassisSpeeds speeds){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    
  }


  public boolean followPath(double target, double angle){
    boolean reachedDest = true;
    for (int i=0;i<modules.length;i++){
      modules[i].setAngle(angle);
      reachedDest= reachedDest && modules[i].followPath(target);
      
    } 
    return reachedDest;
    



  }
}
