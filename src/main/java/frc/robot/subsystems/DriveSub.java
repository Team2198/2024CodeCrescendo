// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Measure;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.controllers.PathFollowingController;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.StructArrayPublisher;

public class DriveSub extends SubsystemBase {
  /** Creates a new DriveSub. */
  
  //private final SwerveModule frontLeft = new SwerveModule(18, 19, 25,0.380615, true,"front left", 0.015, 2.4585);
  //private final SwerveModule frontLeft = new SwerveModule(18, 19, 25,0.380371, true,"front left", 0.015, 2.4585);
  private final SwerveModule frontLeft = new SwerveModule(18, 19, 25,0.376709, true,"front left", 0.015, 2.4585);
  private final PhotonCamera noteDetectCamera = new PhotonCamera("NoteDetect");
  //private final SwerveModule frontRight = new SwerveModule(16, 17, 23,-0.668701, true,"front right",0.015, 2.4691);
  private final SwerveModule frontRight = new SwerveModule(16, 17, 23,0.323242, true,"front right",0.015, 2.4691);
  AHRS gyro = new AHRS();
  PIDController positionPidController = new PIDController(4,0,0);
  int counter=0;
  double dimension = Units.inchesToMeters(27/2);
  //private final SwerveModule backLeft = new SwerveModule(20, 21, 26, 0.268311, true, "back left", 0.015, 2.4978);
  private final SwerveModule backLeft = new SwerveModule(20, 21, 26, 0.261963, true, "back left", 0.015, 2.4978);
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(dimension,dimension), new Translation2d(dimension,-dimension), new Translation2d(-dimension,dimension), new Translation2d(-dimension,-dimension));
  //private final SwerveModule backRight = new SwerveModule(14, 15, 22,0.133301, true, "back right", 0.015, 2.46);
  private final SwerveModule backRight = new SwerveModule(14, 15, 22,0.128418, true, "back right", 0.015, 2.46);
  private final SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
  
   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));        
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  boolean keepTurning = false;
  double robotOffset = 0;
 // PIDController pidController = new PIDController(0.007, 0, 0);
  PIDController pidController = new PIDController(0.007,0,0);
  SwerveDrivePoseEstimator odometry;
  
  PhotonCamera camera = new PhotonCamera("AprilTag");
  Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-17),0, Units.inchesToMeters( 25)), new Rotation3d(0,Units.degreesToRadians(-30),Units.degreesToRadians(180)));
  AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  PhotonPoseEstimator visionPoseEst = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
  Pose3d robotToField = new Pose3d(new Translation3d(),new Rotation3d());
  Pose2d blueSpeaker;
  Pose2d redSpeaker;
  Pose2d target;
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();
  /**ss
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */         
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  public DriveSub() {
    visionPoseEst.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(0), getModulePositions(), new Pose2d(), stateStdDevs,visionMeasurementStdDevs);
    gyro.zeroYaw();
    blueSpeaker = visionPoseEst.getFieldTags().getTagPose(8).get().toPose2d();
    redSpeaker = visionPoseEst.getFieldTags().getTagPose(4).get().toPose2d();
    //HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(0.005,0,0), new PIDConstants(5, 0, 0),Constants.TeleOp.maxSpeed, Math.sqrt(2*Constants.TeleOp.robotRadius*Constants.TeleOp.robotRadius), new ReplanningConfig());
    positionPidController.setTolerance(0.1);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(4);
    robotRelative(0, 0, 0);

    //AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, this::getSpeeds, this::setModuleStates, pathFollowerConfig, ()->false, this);
  }

  public void setTarget(){
    if (DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      target = blueSpeaker;
    }

    else if (DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      target = redSpeaker;
    }
  }

  @Override
  public void periodic() {
    //counter+=1;
    //ChassisSpeeds speeds = new ChassisSpeeds(1, -5,0);
    //SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    //SmartDashboard.putNumber("module angles", Math.toDegrees(moduleStates[0].angle.getRadians()));
    SmartDashboard.putNumber("yaw",getHeading());
    SmartDashboard.putNumber("robot offset", robotOffset);
    //setAngle(90);
    
    odometry.update(getRotation2d(), getModulePositions());
    arrayPublisher.set(new Pose2d[] {currPoseVision(), getPose()});
    //SmartDashboard.putNumber("limeligth data x", getLimelight());
    if (getnumTags()>0){
      odometry.addVisionMeasurement(currPoseVision(), pipelineTime(), getEstimationStdDevs(currPoseVision()));
    }
    
    // This method will be called once per scheduler run
    
  }

  public Optional<EstimatedRobotPose> getVisionPose(){
    return visionPoseEst.update();
  }

  public PhotonPipelineResult getLatestResult(){
    return camera.getLatestResult();
  }

  public double pipelineTime(){
    return getLatestResult().getTimestampSeconds();
  }


  public double distToTarget(){
    return PhotonUtils.getDistanceToPose(odometry.getEstimatedPosition(), target);
    
  }

  public double getPitchTarget(){
    return Math.atan(77/distToTarget());
  }

  public Pose2d currPoseVision(){
    Optional<EstimatedRobotPose> rawPose = getVisionPose();
    
    if (rawPose.isPresent()){
      EstimatedRobotPose pose = rawPose.get();
      robotToField = pose.estimatedPose;
      

      double dist = PhotonUtils.getDistanceToPose(robotToField.toPose2d(),target);
      Rotation2d yaw = PhotonUtils.getYawToPose(robotToField.toPose2d(),target);
    //SmartDashboard.putData("vision pose", pose2);\
    
    VisionHelper.setVisionPose(robotToField.toPose2d());
    SmartDashboard.putNumber("distance", dist);
    SmartDashboard.putNumber("yaw", yaw.getDegrees());
    SmartDashboard.putBoolean("can see", false);
    }
    return robotToField.toPose2d();
  }

  public void zeroYaw(){
    gyro.zeroYaw();
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    return modulePositions;
  }

  public double getDrivePosition(){
    double averageDrive = 0;
     for (int i=0;i<modules.length;i++){
      averageDrive+=modules[i].getDrivePosition()/4;
    }
    return averageDrive;
  }

  public Pose2d getPose(){
    VisionHelper.setRobotPose(odometry.getEstimatedPosition());
    return odometry.getEstimatedPosition();
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

  public void resetOdometryVision(){
    SwerveModulePosition[] modulePositions = getModulePositions();
    odometry.resetPosition(getRotation2d(), modulePositions, currPoseVision());
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
    double angle = (gyro.getYaw()*-1);
    SmartDashboard.putNumber("raw heading", angle);
    angle = angle+robotOffset;
    

    if (angle>180){
      angle = angle-360;
    }

    


   

    return angle;

  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = Constants.TeleOp.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = visionPoseEst.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.TeleOp.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

  public double getnumTags(){
    return getLatestResult().getTargets().size();
  }

  public double getRotHeading(){
    double angle = (gyro.getYaw()*-1)+robotOffset;
    if (angle>180){
      angle = angle-360;
    }

    else if (angle<180){
      angle = 360+angle;
    }

    return angle;

  }

  public void setModuleStates(ChassisSpeeds speed){
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speed);
    //desaturate wheelspeeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.TeleOp.maxSpeed);
    SmartDashboard.putNumber("module angles2", Math.toDegrees(moduleStates[0].angle.getRadians()));
    SmartDashboard.putNumber("module velocity", (moduleStates[0].speedMetersPerSecond));
   
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
   return pidController.atSetpoint();
   //return false;
  } 

  //pass the angle that you want the robot to turn to
  public double turnToAngle(double goal){
    
    if (goal>180){
      goal = goal-360;
    }

    double turnSpeed = pidController.calculate(getHeading(), goal);
    turnSpeed*=Constants.TeleOp.maxTurningRad*0.8;

    
    
    //returns true if auto aligned
    return turnSpeed;
    //return 0;
    
  }

  

  public void setRobotRelative(ChassisSpeeds speeds){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    
  }


  public boolean followPath(double target, double angle){
    //setAngle(angle);
    for (int i=0;i<modules.length;i++){
      modules[i].setAngle(angle);
      modules[i].followPath(positionPidController.calculate(getDrivePosition(), target));
      
      
    } 
    return positionPidController.atSetpoint();

  }

  public void zeroEncoders(){
    for (int i=0;i<modules.length;i++){
      modules[i].zeroDrive(); 
    } 
  }

  public double getLimelight(){
   
    PhotonPipelineResult result = noteDetectCamera.getLatestResult();

    double x = result.getBestTarget().getYaw();
    VisionHelper.updateNoteYaw(x);

    return x;
  }


  public void setOffset(double angle){
    robotOffset = angle;
  }

  public Command setRobotOffset(double angle){
    
    return this.runOnce(()->this.setOffset(angle));
  }

  public Command zeroEncodersCommand(){
    return this.runOnce(()->this.zeroEncoders());
  }

  public void setwheelAngles(double angle){
    for (int i=0;i<modules.length;i++){
      modules[i].setAngle(angle); 
    } 
  }

  public Command setWheelAngleCommand(double angle){
    return this.run(()->this.setwheelAngles(angle));
  }

  public Command visionReset(){
    return this.runOnce(()->this.resetOdometryVision());

  }

}
