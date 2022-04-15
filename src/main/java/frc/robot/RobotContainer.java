// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.commands.AutoAlign;
import frc.robot.commands.GetShooterReady;
import frc.robot.commands.TeleopDriveArcade;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Convoyeur;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static Joystick m_pilotJoystick;
  private static Joystick m_copilotJoystick;

  private final DriveTrain m_drivetrain;
  private final Intake m_intake;
  private final Vision m_vision;
  private final Shooter m_shooter;
  private final Climber m_climber;
  private final Convoyeur m_convoyeur;
  private final Turret m_turret;
  private final Leds m_leds;

  SendableChooser<Command> m_autoChooser;
  ComplexWidget autoChooserList;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){

    m_vision = new Vision();
    m_turret = new Turret();
    m_convoyeur = new Convoyeur(m_turret);
    m_drivetrain = new DriveTrain();
    m_intake = new Intake(m_convoyeur);
    m_shooter = new Shooter();
    m_climber = new Climber();
    
    m_leds = new Leds();

    m_pilotJoystick = new Joystick(0);
    m_copilotJoystick = new Joystick(1);

    //configureDefaultCommands();
    configureButtonBindings();

    m_autoChooser = new SendableChooser<>();

    // Add commands to the autonomous command chooser

  m_autoChooser.setDefaultOption("shit auto", getShitAuto());
  m_autoChooser.addOption("4 Balls", getAutonomousCommand4Balls());  
  m_autoChooser.addOption("2 Balls", getAutonomousCommand2Balls());
  m_autoChooser.addOption("2 Balls steal", getAutonomousCommand2BallsSteal());
  m_autoChooser.addOption("5 Balls", getAutonomousCommand5Balls());
  // Put the chooser on the dashboard
  autoChooserList = Shuffleboard
  .getTab("Pilot View")
  .add(m_autoChooser)
  .withWidget(BuiltInWidgets.kComboBoxChooser);
  
  //SmartDashboard.putData(m_autoChooser);
  }

  public void configureDefaultCommands(){

    m_drivetrain.setDefaultCommand(
      new TeleopDriveArcade(
        m_drivetrain, 
        () -> { return m_pilotJoystick.getRawAxis(3) - m_pilotJoystick.getRawAxis(2); },
        () -> { return m_pilotJoystick.getRawAxis(0); }
      )
    );

    m_shooter.setDefaultCommand(new RunCommand(m_shooter::stop, m_shooter));
    m_leds.setDefaultCommand(new RunCommand(m_leds::ledsOn, m_leds));
    m_climber.setDefaultCommand(new RunCommand(m_climber::stop, m_climber));
    m_turret.setDefaultCommand(new RunCommand(m_turret::aimManual, m_turret));

  }

  public void configureIntakeDefaultCommand(){
    m_intake.setDefaultCommand(new RunCommand(m_intake::handleIntakeWinch, m_intake));
  }

  public void providePilotFeedback() {
    if(m_intake.isIntaking()) {
      m_pilotJoystick.setRumble(RumbleType.kLeftRumble, 1);
      m_pilotJoystick.setRumble(RumbleType.kRightRumble, 1);
    } else {
      m_pilotJoystick.setRumble(RumbleType.kLeftRumble, 0);
      m_pilotJoystick.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(){
   
    new JoystickButton(m_pilotJoystick, XboxController.Button.kA.value).whenPressed(new ConditionalCommand(
      new InstantCommand(m_intake::runReversed, m_convoyeur)
        .andThen(new WaitCommand(0.15))
        .andThen(new InstantCommand(m_intake::stop, m_convoyeur)), 
      new InstantCommand(m_intake::run, m_intake, m_convoyeur)
        .andThen(new InstantCommand(m_intake::releaseIntake, m_intake)), 
      m_intake::isRunning));

    new JoystickButton(m_pilotJoystick, XboxController.Button.kB.value)
      .whileHeld(new StartEndCommand(m_intake::runReversed, m_intake::stop, m_convoyeur))
      .whenPressed(new InstantCommand(m_intake::releaseIntake));

    new JoystickButton(m_copilotJoystick, XboxController.Button.kY.value)
    .whileHeld(new AutoAlign(m_turret, m_vision));

    new JoystickButton(m_copilotJoystick, XboxController.Button.kLeftBumper.value)
    .whileHeld(new GetShooterReady(m_shooter, m_turret, m_vision));

    new JoystickButton(m_copilotJoystick, XboxController.Button.kRightBumper.value)
      .whileHeld(new StartEndCommand(m_convoyeur::feed, m_convoyeur::stop, m_convoyeur));

    new JoystickButton(m_copilotJoystick, XboxController.Button.kX.value)
      .whileHeld(new RunCommand(m_climber::climb, m_climber)
      .andThen(new InstantCommand(m_climber::stop, m_climber))); 
    
    new JoystickButton(m_copilotJoystick, XboxController.Button.kB.value)
      .whileHeld(new RunCommand(m_climber::extend, m_climber)
      .andThen(new InstantCommand(m_climber::stop, m_climber))); 

    new JoystickButton(m_copilotJoystick, XboxController.Button.kA.value)
      .whileHeld(new StartEndCommand(m_intake::runReversed, m_intake::stop, m_convoyeur))
      .whenPressed(new InstantCommand(m_intake::releaseIntake));

    new JoystickButton(m_copilotJoystick, XboxController.Button.kBack.value)
      .whenPressed(new InstantCommand(m_climber::releaseClimb, m_climber));

    new JoystickButton(m_copilotJoystick, XboxController.Button.kStart.value)
      .toggleWhenPressed(new RunCommand(m_turret::setTurretClimb, m_turret), false)
      .whenPressed(new InstantCommand(m_intake::retractIntake));

    new JoystickButton(m_pilotJoystick, XboxController.Button.kX.value)
      .whenPressed(new InstantCommand(m_intake::retractIntake));
  }

  public void resetSensors(){
    m_drivetrain.resetEncoders();
    m_drivetrain.zeroHeading();
  }

  public Command getAutoChooser(){
    return m_autoChooser.getSelected();
    //return getAutonomousCommand4Balls();
  }

  /*
  public Command cmd1(){
    return new PrintCommand("cmd1");
  }
  public Command cmd2(){
    return new PrintCommand("cmd2");
  }
  public Command cmd3(){
    return new PrintCommand("cmd3");
  }
  */
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public Command getShitAuto(){

    Command shootSeq =

      //1st attempt
      new InstantCommand(m_intake::runReversed)
      .andThen(new WaitCommand(0.15))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(1))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(2))
      //2nd attempt
      .andThen(m_intake::runReversed)
      .andThen(new WaitCommand(0.15))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(0.1))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(2))
      //3rd attempt
      .andThen(m_intake::runReversed)
      .andThen(new WaitCommand(0.15))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(0.1))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(2))
      //stop mechanisms
      .andThen(new InstantCommand(m_shooter::stop))
      .andThen(new InstantCommand(m_intake::stop));


    return new InstantCommand(m_intake::run)
    .andThen(new InstantCommand(m_intake::releaseIntake))
    .andThen(new InstantCommand(m_drivetrain::driveReverse))
    .andThen(new WaitCommand(2.5))
    .andThen(new InstantCommand(m_drivetrain::stop))
    .andThen(new InstantCommand(m_intake::stop))
    .andThen(shootSeq)
    .raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));
   }
  public Command getAutonomousCommand2Balls() {

    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.ksVolts, 
          Constants.ksVoltsSecondMeters, 
          Constants.ksVoltsSecondMetersSquared), 
          Constants.driveKinematics, 
          9);

    TrajectoryConfig configForward =
      new TrajectoryConfig(
        Constants.kMaxSpeedMetersSecondSlow,
        Constants.kMaxAccelerationMetersSecondSquaredSlow)
        .setKinematics(Constants.driveKinematics)
        .addConstraint(autoVoltageConstraint);

        TrajectoryConfig configReverse =
        new TrajectoryConfig(
          Constants.kMaxSpeedMetersSecondSlow,
          Constants.kMaxAccelerationMetersSecondSquaredSlow)
          .setKinematics(Constants.driveKinematics)
          .addConstraint(autoVoltageConstraint)
          .setReversed(true);

    Trajectory exampleTrajectory1 =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(0)),
        List.of(),
        new Pose2d(2,0,new Rotation2d(Math.toRadians(0))),
        configForward);

    Trajectory exampleTrajectory2 =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2,0, new Rotation2d(0)),
        List.of(),
        new Pose2d(1.5,0,new Rotation2d(0)),
        configReverse);

    Trajectory combo = exampleTrajectory1.concatenate(exampleTrajectory2);

    RamseteCommand ramseteCommand2 = 
      new RamseteCommand(
          combo,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity,
          m_drivetrain);

    // An ExampleCommand will run in autonomous
    return new InstantCommand(m_intake::run)
      .andThen(new InstantCommand(m_intake::releaseIntake))
      .andThen(ramseteCommand2)
      .andThen(new InstantCommand(m_intake::stop))
      .andThen((new GetShooterReady(m_shooter, m_turret, m_vision))
      .andThen(new WaitCommand(3)
      .andThen(new RunCommand(m_convoyeur::feed))))
      .raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));
  }

  public Command getAutonomousCommand4Balls() {

    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.ksVolts, 
          Constants.ksVoltsSecondMeters, 
          Constants.ksVoltsSecondMetersSquared), 
          Constants.driveKinematics, 
          9);

  TrajectoryConfig configForward =
    new TrajectoryConfig(
        Constants.kMaxSpeedMetersSecondSlow,
        Constants.kMaxAccelerationMetersSecondSquaredSlow)
      .setKinematics(Constants.driveKinematics)
      .addConstraint(autoVoltageConstraint);

    TrajectoryConfig configReverse =
      new TrajectoryConfig(
        Constants.kMaxSpeedMetersSecondSlow,
        Constants.kMaxAccelerationMetersSecondSquaredSlow)
      .setKinematics(Constants.driveKinematics)
      .addConstraint(autoVoltageConstraint)
      .setReversed(true);

    Trajectory getFirstBall =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(Math.toRadians(0))),
        List.of(),
        new Pose2d(1.5, 0.2,new Rotation2d(Math.toRadians(10))),
        configForward);

    Trajectory getBallsFeeder =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.5,0.2,new Rotation2d(Math.toRadians(10))),
        List.of(),
        new Pose2d(4.7, -0.70,new Rotation2d(Math.toRadians(30))),
        configForward);

    Trajectory goShoot =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(4.70,-0.70, new Rotation2d(Math.toRadians(30))),
        List.of(),
        new Pose2d(1.5,0,new Rotation2d(Math.toRadians(-10))),
        configReverse);

    RamseteCommand Move1 = 
      new RamseteCommand(
          getFirstBall,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity);

    RamseteCommand Move2 = 
      new RamseteCommand(
          getBallsFeeder,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity);
    

    RamseteCommand Move3 = 
      new RamseteCommand(
          goShoot,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity);

     Command shootSeq =
      new InstantCommand(m_intake::runReversed)
      .andThen(new WaitCommand(0.1))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(1.25))
      .andThen(new InstantCommand(m_shooter::stop))
      .andThen(new InstantCommand(m_intake::stop));
      //.raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));

      Command shootSeq2 =
      new InstantCommand(m_intake::runReversed)
      .andThen(new WaitCommand(0.1))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(2))
      .andThen(new InstantCommand(m_shooter::stop))
      .andThen(new InstantCommand(m_intake::stop));
      //.raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));

    m_drivetrain.resetOdometry(getFirstBall.getInitialPose());
    // An ExampleCommand will run in autonomous
    return 
      //move while intaking
      new InstantCommand(m_intake::run)
      .andThen(new InstantCommand(m_intake::releaseIntake))
      //.andThen(m_shooter::setAutoPreSpin)
      .andThen(Move1)
      .andThen(new InstantCommand(m_intake::stop))

      //shoot sequence
      .andThen(shootSeq)

      //move to feeder while intaking
      .andThen(new InstantCommand(m_intake::run))
      .andThen(Move2)
      .andThen(new WaitCommand(0.01))

      //go to shooting position
      //.andThen(m_shooter::setAutoPreSpin)
      .andThen(Move3)
      .andThen(new InstantCommand(m_intake::stop))
      
      //shoot sequence
      .andThen(shootSeq2)
      .raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));
      

      
  }

  public Command getAutonomousCommand5Balls() {

    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.ksVolts, 
          Constants.ksVoltsSecondMeters, 
          Constants.ksVoltsSecondMetersSquared), 
          Constants.driveKinematics, 
          10);

  TrajectoryConfig configForward =
    new TrajectoryConfig(
        Constants.kMaxSpeedMetersSecondFast,
        Constants.kMaxAccelerationMetersSecondSquaredFast)
      .setKinematics(Constants.driveKinematics)
      .addConstraint(autoVoltageConstraint);

    TrajectoryConfig configReverse =
      new TrajectoryConfig(
        Constants.kMaxSpeedMetersSecondFast,
        Constants.kMaxAccelerationMetersSecondSquaredFast)
      .setKinematics(Constants.driveKinematics)
      .addConstraint(autoVoltageConstraint)
      .setReversed(true);


      Trajectory getZeroBall =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(-1.15, 1.42, new Rotation2d(Math.toRadians(60))), 
      List.of(), 
      new Pose2d(-0.5, 2.5,new Rotation2d(Math.toRadians(-25))), 
      configForward);

    Trajectory getFirstBall =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(-0.5, 2.5, new Rotation2d(Math.toRadians(-25))), 
      List.of(), 
      new Pose2d(1.5, 0.2,new Rotation2d(Math.toRadians(-60))), 
      configForward);

    Trajectory getBallsFeeder =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.5,0.2,new Rotation2d(Math.toRadians(-60))),
        List.of(),
        new Pose2d(5.20, -0.70,new Rotation2d(Math.toRadians(30))),
        configForward);

    Trajectory goShoot =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.20, -0.70, new Rotation2d(Math.toRadians(30))),
        List.of(),
        new Pose2d(1.5, 0, new Rotation2d(Math.toRadians(-10))),
        configReverse);

    RamseteCommand Move0 =
      new RamseteCommand(
        getZeroBall,
        m_drivetrain::getPose,
        new RamseteController(),
        Constants.driveKinematics,
        m_drivetrain::setDriveVelocity);

    RamseteCommand Move1 = 
      new RamseteCommand(
          getFirstBall,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity);

    RamseteCommand Move2 = 
      new RamseteCommand(
          getBallsFeeder,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity);
    

    RamseteCommand Move3 = 
      new RamseteCommand(
          goShoot,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity);

     Command shootSeq =
      new InstantCommand(m_intake::runReversed)
      .andThen(new WaitCommand(0.1))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(1.25))
      .andThen(new InstantCommand(m_shooter::stop))
      .andThen(new InstantCommand(m_intake::stop));
      //.raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));

      Command shootSeq2 =
      new InstantCommand(m_intake::runReversed)
      .andThen(new WaitCommand(0.1))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(2))
      .andThen(new InstantCommand(m_shooter::stop))
      .andThen(new InstantCommand(m_intake::stop));
      //.raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));

      Command shootSeq3 =
      new InstantCommand(m_intake::runReversed)
      .andThen(new WaitCommand(0.1))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(1.25))
      .andThen(new InstantCommand(m_shooter::stop))
      .andThen(new InstantCommand(m_intake::stop));

    m_drivetrain.resetOdometry(getZeroBall.getInitialPose());
    // An ExampleCommand will run in autonomous
    return 
      //move while intaking
      new InstantCommand(m_intake::run)
      .andThen(new InstantCommand(m_intake::releaseIntake))
      //.andThen(m_shooter::setAutoPreSpin)
      .andThen(Move0)
      .andThen(new InstantCommand(m_intake::stop))

      //shoot sequence
      .andThen(shootSeq)

      .andThen(new InstantCommand(m_intake::run))
      .andThen(Move1)
      .andThen(new InstantCommand(m_intake::stop))

      //shoot sequence
      .andThen(shootSeq3)

      //move to feeder while intaking
      .andThen(new InstantCommand(m_intake::run))
      .andThen(Move2)
      .andThen(new WaitCommand(0.01))

      //go to shooting position
      //.andThen(m_shooter::setAutoPreSpin)
      .andThen(Move3)
      .andThen(new InstantCommand(m_intake::stop))
      
      //shoot sequence
      .andThen(shootSeq2)
      .raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));
  }

  public Command getAutonomousCommand2BallsSteal() {

    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.ksVolts, 
          Constants.ksVoltsSecondMeters, 
          Constants.ksVoltsSecondMetersSquared), 
          Constants.driveKinematics, 
          10);

  TrajectoryConfig configForward =
    new TrajectoryConfig(
        Constants.kMaxSpeedMetersSecondFast,
        Constants.kMaxAccelerationMetersSecondSquaredFast)
      .setKinematics(Constants.driveKinematics)
      .addConstraint(autoVoltageConstraint);

    TrajectoryConfig configReverse =
      new TrajectoryConfig(
        Constants.kMaxSpeedMetersSecondFast,
        Constants.kMaxAccelerationMetersSecondSquaredFast)
      .setKinematics(Constants.driveKinematics)
      .addConstraint(autoVoltageConstraint)
      .setReversed(true);

    Trajectory getFirstBall =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(Math.toRadians(0))),
        List.of(),
        new Pose2d(1.5, -0.2,new Rotation2d(Math.toRadians(-10))),
        configForward);

    Trajectory firstReverse =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.5,-0.2,new Rotation2d(Math.toRadians(-10))),
        List.of(),
        new Pose2d(0, -0.5,new Rotation2d(Math.toRadians(-10))),
        configReverse);
    
    Trajectory getFirstOppBall =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,-0.5,new Rotation2d(Math.toRadians(-10))),
        List.of(),
        new Pose2d(1, -1.2,new Rotation2d(Math.toRadians(-20))),
        configForward);

    Trajectory getSecondOppBall =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(1,-1.2, new Rotation2d(Math.toRadians(-20))),
        List.of(),
        new Pose2d(0.3,3,new Rotation2d(Math.toRadians(145))),
        configForward);


    Trajectory ballStealing = firstReverse.concatenate(getFirstOppBall.concatenate(getSecondOppBall));

    RamseteCommand Move1 = 
      new RamseteCommand(
          getFirstBall,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity);

    RamseteCommand Move2 = 
      new RamseteCommand(
          ballStealing,
          m_drivetrain::getPose,
          new RamseteController(),
          Constants.driveKinematics,
          m_drivetrain::setDriveVelocity);
    

     Command shootSeq =
      new InstantCommand(m_intake::runReversed)
      .andThen(new WaitCommand(0.1))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(1.75))
      .andThen(new InstantCommand(m_shooter::stop))
      .andThen(new InstantCommand(m_intake::stop))
      .raceWith(new GetShooterReady(m_shooter, m_turret, m_vision));


    m_drivetrain.resetOdometry(getFirstBall.getInitialPose());
    // An ExampleCommand will run in autonomous
    return 
      //move while intaking
      new InstantCommand(m_intake::run)
      //.andThen(m_shooter::setAutoPreSpin)
      .andThen(new InstantCommand(m_intake::releaseIntake))
      .andThen(Move1)
      .andThen(new InstantCommand(m_intake::stop))

      //shoot sequence
      .andThen(shootSeq)

      //move to steal 2 balls while intaking
      .andThen(new InstantCommand(m_intake::run))
      .andThen(Move2)
      .andThen(new WaitCommand(0.01))
      

      //vomit 2 balls in hangar
      .andThen(new InstantCommand(m_shooter::setAutoPreSpin))
      .andThen(new WaitCommand(0.25))
      .andThen(new InstantCommand(m_convoyeur::feed))
      .andThen(new WaitCommand(1.5))
      .andThen(new InstantCommand(m_intake::stop))
      .andThen(new InstantCommand(m_shooter::stop));
      
      
  }

  public static Joystick getPilotJoystick(){
    return m_pilotJoystick;
  }
  public static Joystick getCopilotJoystick(){
    return m_copilotJoystick;
  }
}
