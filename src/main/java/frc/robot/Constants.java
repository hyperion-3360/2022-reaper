// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int id_DriveL1 = 3;
    public static final int id_DriveL2 = 4;
    public static final int id_DriveR1 = 1;
    public static final int id_DriveR2 = 2;

    public static final int id_intakeRoller = 6;

    public static final int id_Shooter1 = 7;
    public static final int id_Shooter2 = 8;
    public static final int id_TurretTurner = 9;

    public static final int id_ClimberL = 10;
    public static final int id_ClimberR = 11;

    public static final int id_convoyeurLock = 12;
    public static final int id_convoyeurBelt = 13;

    public static final int id_intakeWinch = 15;

    public static final int id_servoRelease = 2;

    public static final int id_climberLeftRelease = 0;
    public static final int id_climberRightRelease = 1;

    public static final int id_ledR = 0;
    public static final int id_ledL = 1;
    public static final int id_ledF = 2;
    public static final int id_ledB = 3;

    public static final double ksVolts = 0.78593;
    public static final double ksVoltsSecondMeters = 2.7053;
    public static final double ksVoltsSecondMetersSquared = 0.38565;

    public static final double kMaxSpeedMetersSecondFast = 2.5;
    public static final double kMaxAccelerationMetersSecondSquaredFast = 2;

    public static final double kMaxSpeedMetersSecondSlow = 2;
    public static final double kMaxAccelerationMetersSecondSquaredSlow = 1.25;

    public static final double kTrackWidthMeters = 1.5;

    public static final DifferentialDriveKinematics driveKinematics = 
      new DifferentialDriveKinematics(kTrackWidthMeters);
}
