package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class AutoConstants {
        public static final int ampPipeline = 0;

        public static final double limelightAngle = 40.0;

        public static final double rotKP = 0.01;
        public static final double rotKI = 0.0;
        public static final double rotKD = 0.0;

        public static final double xKP = 0.5;
        public static final double xKI = 0.0;
        public static final double xKD = 0.01;

        public static final double yKP = 0.35;
        public static final double yKI = 0.0;
        public static final double yKD = 0.008;
    }

    /** 
     * Drivetrain and output control constants.
     * Most are copied from the <a href="https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java">MAXSwerve Java example</a>.
     */
    public static final class DriveConstants {
        /** 
         * Distance between centers of front and back wheels on drivetrain in meters. 
         * Explained in more detail in {@link frc.robot.subsystems.drive.Drive#Drive() <code>new frc.robot.subsystems.Drive()</code>}
         */
        public static final double wheelBase = Units.inchesToMeters(26.5);
        /** 
         * Distance between centers of right and left wheels on drivetrain in meters.
         * Explained in more detail in {@link frc.robot.subsystems.drive.Drive#Drive() <code>new frc.robot.subsystems.Drive()</code>}
         */
        public static final double trackWidth = Units.inchesToMeters(26.5); 
        /** Radius of the drive base (in meters) measured from the center of the base to the furthest module */
        public static final double baseRadius = Math.sqrt(wheelBase*wheelBase + trackWidth*trackWidth);

        /** Angular offset of module A (in rads). */
        public static final double aAngularOffset = -Math.PI/2;
        /** Angular offset of module B (in rads). */
        public static final double bAngularOffset = 0;
        /** Angular offset of module C (in rads). */
        public static final double cAngularOffset = Math.PI;
        /** Angular offset of module D (in rads). */
        public static final double dAngularOffset = Math.PI/2;
        /** Angular offsets of each module (in rads) */
        public static final double[] angularOffsets = {aAngularOffset, bAngularOffset, cAngularOffset, dAngularOffset};

        /** Max translaitonal speed (m/s) */
        public static final double maxTranslationalSpeed = 4.8;
        public static final double kiddieMaxTranslationalSpeed = 1.2; // TODO: Actually find this;
        /** Max angular speed (rads/s) */
        public static final double maxAngularSpeed = Units.degreesToRadians(360); // TODO: Actually find this;
        public static final double maxAngularAccel = Units.degreesToRadians(540);
        /** Max translation acceleration (m/s) */
        public static final double maxAcceleration = 4.0; // TODO: Actually find this.

        /** Maximum speed at which the translation vector direction ({@code Math.atan2(y, x)}) can change. Measured in rads/s. */
        public static final double magnitudeSlewRate = 1.0;
        /** Maximum speed at which the translation vector magnitude ({@code Math.sqrt(x*x + y*y)}) can change. Measured in percent/s (1 = 100%). */
        public static final double rotationalSlewRate = 2.0;

        /** Used to invert the gyroscope direction. */
        public static final boolean gyroReversed = true;
        /** Max input speed. */
        public static final double speed = 1.0;

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // TODO: Tune this
            new PIDConstants(5.0, 0.0, 0.0), // TODO: Tune this
            ModuleConstants.maxSpeed,
            baseRadius,
            new ReplanningConfig()
        );
    }

    /** 
     * Constants for each individual MAXSwerve module (wheel).
     * Includes precalculated gear ratios, PID constants, and motor info.
     * Most are copied from the <a href="https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java">MAXSwerve Java example</a>.
     */
    public static final class ModuleConstants {
        /** 
         * The MAXSwerve module can be configued with one of three pinion gears: 12T (Low), 13T (Mid), and 14T (High).
         * This changes the drive speed of the module.
         * A pinion gear with more teeth will result in a faster but weaker drivetrain.
         */
        public static final int drivePinionTeeth = 13;

        /**
         *  Invert the turning envoder since the output shaft rotates in the
         *  opposite direction of the steering motor in the MAXSwerve module. 
         */
        public static final boolean encoderReversed = true;

        /** MAXSwerve uses NEOs for drive motors, which are brushless. */
        public static final MotorType driveMotorType = MotorType.kBrushless;
        /** MAXSwerve uses NEO 550s for turn motors, which are brushless. */
        public static final MotorType turnMotorType = MotorType.kBrushless;

        // Calculations required for driving motor conversion factors and feed forward.
        /** Drive motor free speed (in RPS). */
        public static final double driveMotorFreeRps = MotorConstants.NeoFreeRpm / 60;
        /** Wheel diameter in meters. */
        public static final double wheelDiameterMeters = 0.0762;
        /** Wheel circumferance in meters. */
        public static final double wheelCircumferanceMeters = wheelDiameterMeters * Math.PI;
        
        /** 
         * Gear reduction on the drive motor.
         * 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion. 
         */
        public static final double driveMotorReduction = (45.0 * 22) / (drivePinionTeeth * 15);
        /** Drive wheel free speed (in RPS). */
        public static final double driveWheelFreeRps = (driveMotorFreeRps * wheelCircumferanceMeters) / driveMotorReduction;
        /** Distance traveled per rotation of the drive motor (in meters). */
        public static final double driveEncoderPositionFactor = wheelCircumferanceMeters / driveMotorReduction;
        /** Velocity factor for the drive motor (in m/s). */
        public static final double driveEncoderVelocityFactor = driveEncoderPositionFactor / 60;

        /** Radians travled per rotation of the turn encoder (absolute). In radians. */
        public static final double turnEncoderPositionFactor = 2 * Math.PI;
        /** Angular velocity factor for the turn motor (in rads/s). */
        public static final double turnEncoderVelocityFactor = turnEncoderPositionFactor / 60;

        /** Minimum value of the absolute turn encoder in radians. */
        public static final double turnEncoderPositionPIDMinInput = 0;
        /** Maximum value of the absolute turn encoder in radians. */
        public static final double turnEncoderPositionPIDMaxInput = turnEncoderPositionFactor;

        public static final double driveKP = 0.05;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveFF = 1 / driveWheelFreeRps;
        public static final double driveMinOutput = -1;
        public static final double driveMaxOutput = 1;

        public static final double turnKP = 1.0;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnFF = 0;
        public static final double turnMinOutput = -1;
        public static final double turnMaxOutput = 1;
        
        /** Drive motor idle mode for Spark Max. */
        public static final IdleMode driveMotorIdleMode = IdleMode.kCoast;
        /** Rotation motor idle mode for Spark Max. */
        public static final IdleMode turnMotorIdleMode = IdleMode.kBrake;

        /** Drive motor smart current limit (in Amps). */
        public static final int driveMotorCurrentLimit = 60; // amps (A)
        /** Rotation motor smart current limit (in Amps).  */
        public static final int turnMotorCurrentLimit = 20; //amps (A)

        public static final double maxSpeed = 4.5; // TODO: Find this
    }

    /** Various constants used for the intake subsystem and commands */
    public static final class IntakeConstants {
        public static final double distanceFactor = 1.0/4.0 * 1.5 * Math.PI; // gear ratio * wheel radius * pi
        // TODO: Tune target intake velocity for speaker
        /** Target intake velocity for shooting into the Speaker */
        public static final double speakerTarget = 4750;       // TODO: Tune target intake velocity for amp
        /** Target intake velocity for shooting into the Amp */
        public static final double ampTarget = 4000;
        public static final double idleTarget = 1800;
        public static final double intakeTarget = -1000;

        // TODO: Tune intake PID
        public static final double motorKP = 0.00001;
        public static final double motorKI = 2e-7;
        public static final double motorKD = 0.02;
        public static final double motorKF = 0.0002;
    }

    /** Various constants used for the intake shooter and commands */
    public static final class ShooterConstants {
        // TODO: Tune target shooter velocity for speaker
        public static final double speakerTarget = 4550;
        // TODO: Tune target shooter velocity for amp
        public static final double ampTarget = 2200;
        // 
        public static final double idleTarget = 1000;
        //
        public static final double intakeTarget = -1000;

        public static final double shootTime = 5.0;

        // TODO: Tune shooter PID
        public static final double motorKP = 0.0001;
        public static final double motorKI = 5e-7;
        public static final double motorKD = 0.03;
        public static final double motorKF = 0.00015;
    }

    /** Various constants used for the climber subsystem and commands */
    public static final class ClimberConstants {
        public static final float rightMinPosition = -200.0f;
        public static final float leftMinPosition = -200.0f;
        public static final float maxPosition = 0.0f;

        public static final double extendedPosition = maxPosition;
        // TODO: Tune the climber retract position
        // I figured make it a little more than `minPosition` to give the PID some room to stablize
        public static final double retractedPosition = -200.0;

        public static final double conversionFactor = 1.0;// 1.0/16.0 * 2*Math.PI ; // Gear ratio * radius * tau
        public static final double spikeCurrent = 40.0;

        public static final double leftKP = 1.0;
        public static final double leftKI = 0.0;
        public static final double leftKD = 0.0;

        public static final double rightKP = 1.0;
        public static final double rightKI = 0.0;
        public static final double rightKD = 0.0;
    }

    /** Various motor constants taken from datasheets. */
    public static final class MotorConstants {
        /** Free speed of a NEO motor (in RPM). */
        public static final double NeoFreeRpm = 5676;
        /** Resolution of the relative encoder in a NEO motor. */
        public static final double NeoEncoderResolution = 42;
        /** Free speed of a NEO 550 motor (in RPM). */
        public static final double Neo550FreeRpm = 11_000;
        /** Resolution of the relative encoder in a NEO 550 motor. */
        public static final double Neo550EncoderResolution = 42;
    }

    public static final class IOConstants {
        public static final int driveControllerPort = 0;
        public static final int mechControllerPort = 1;

        public static final boolean xyInverted = true;

        public static final boolean rotInverted = true;

        //---------- CAN IDs ------------//
        public static final int aPowerId = 1;
        public static final int aRotId = 2;
        
        public static final int bPowerId = 3;
        public static final int bRotId = 4;

        public static final int cPowerId = 5;
        public static final int cRotId = 6;

        public static final int dPowerId = 7;
        public static final int dRotId = 8;

        public static final int pdhId = 10;

        public static final int intakeId = 11;
        public static final int shooterId = 12;

        public static final int climbLeftId = 13;
        public static final int climbRightId = 14;

        public static final int detectorChannel = 4;
        public static final int noteBeambreakChannel = 3;
        public static final int ledPort = 1;
        public static final int ledLength = 20;
    }

    public static final class LEDConstants {
        public static final long flashLength = 1_000_000_000;
        public static final long fastFlashLength = 250_000_000;
        public static final double fadePeriod = 2e9;
    }

    public static final class GameConstants {
        public static final double noteDiameter = 14.0;
    }

    public static final class DriverConstants {
        public static final class Default {
            public static final boolean xyInverted = false;

            public static final boolean rotInverted = false;
    
            public static final double transDeadband = 0.05;
    
            public static final double rotDeadband = 0.05;
            /** Maximum speed at which the translation vector direction ({@code Math.atan2(y, x)}) can change. Measured in rads/s. */
            public static final double dirSlewRate = Math.PI;
            /** Maximum speed at which the translation vector magnitude ({@code Math.sqrt(x*x + y*y)}) can change. Measured in percent/s (1 = 100%). */
            public static final double magSlewRate = 3.0;
            /** Maximum speed at which the rotation vector magnitude can change. Measured in percent/s (1 = 100%). */
            public static final double rotSlewRate = 3.0;
        }

        public static final class Jett {
            public static final boolean xyInverted = false;

            public static final boolean rotInverted = false;
    
            public static final double transDeadband = 0.05;
    
            public static final double rotDeadband = 0.05;
            /** Maximum speed at which the translation vector direction ({@code Math.atan2(y, x)}) can change. Measured in rads/s. */
            public static final double dirSlewRate = Math.PI / 2;
            /** Maximum speed at which the translation vector magnitude ({@code Math.sqrt(x*x + y*y)}) can change. Measured in percent/s (1 = 100%). */
            public static final double magSlewRate = 1.0;
            /** Maximum speed at which the rotation vector magnitude can change. Measured in percent/s (1 = 100%). */
            public static final double rotSlewRate = 2.0;
        }

        public static final class Chris {
            public static final boolean xyInverted = false;

            public static final boolean rotInverted = false;
    
            public static final double transDeadband = 0.05;
    
            public static final double rotDeadband = 0.05;
            /** Maximum speed at which the translation vector direction ({@code Math.atan2(y, x)}) can change. Measured in rads/s. */
            public static final double dirSlewRate = Math.PI / 2;
            /** Maximum speed at which the translation vector magnitude ({@code Math.sqrt(x*x + y*y)}) can change. Measured in percent/s (1 = 100%). */
            public static final double magSlewRate = 1.0;
            /** Maximum speed at which the rotation vector magnitude can change. Measured in percent/s (1 = 100%). */
            public static final double rotSlewRate = 2.0;
        }

        public static final class Parker {
            public static final boolean xyInverted = false;

            public static final boolean rotInverted = false;
    
            public static final double transDeadband = 0.05;
    
            public static final double rotDeadband = 0.05;
            /** Maximum speed at which the translation vector direction ({@code Math.atan2(y, x)}) can change. Measured in rads/s. */
            public static final double dirSlewRate = Math.PI / 2;
            /** Maximum speed at which the translation vector magnitude ({@code Math.sqrt(x*x + y*y)}) can change. Measured in percent/s (1 = 100%). */
            public static final double magSlewRate = 1.0;
            /** Maximum speed at which the rotation vector magnitude can change. Measured in percent/s (1 = 100%). */
            public static final double rotSlewRate = 2.0;
        }
    }
}
