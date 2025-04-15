// This file is based on code from team 6328 Mechanical Advantage
// See here for the original source:
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/c0c6d11547769f6dc5f304d5c18c9b51086a691b/src/main/java/org/littletonrobotics/frc2024/commands/WheelRadiusCharacterization.java

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.FieldConstants.ReefHeight;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;

public class DriveCommands extends Command {

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(0.1);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    SwerveRequest.RobotCentric req =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withVelocityX(0)
            .withVelocityY(0);

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(6);
                  drive.setControl(req.withRotationalRate(speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getDrivePositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      Angle[] positions = drive.getDrivePositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < Constants.PP_CONFIG.numModules; i++) {
                        wheelDelta +=
                            Math.abs(
                                    positions[i].minus(state.positions[i]).baseUnitMagnitude()
                                        / Constants.SWERVE_MODULE_CONSTANTS.DriveMotorGearRatio)
                                / Constants.PP_CONFIG.numModules;
                      }
                      double wheelRadius =
                          (state.gyroDelta * Constants.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                      System.out.println(
                          "\tDrive base radius: "
                              + formatter.format(Constants.DRIVE_BASE_RADIUS)
                              + " meters");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    Angle[] positions = new Angle[Constants.PP_CONFIG.numModules];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public static Command autoAlignToClosestReefFace(Drive drivetrain) {
    Pose2d currentPose = drivetrain.getPose();

    // Find the reef face that's closest to the current robot pose
    Pose2d closestFace =
        Arrays.stream(FieldConstants.Reef.centerFaces)
            .min(
                (a, b) ->
                    Double.compare(
                        a.getTranslation().getDistance(currentPose.getTranslation()),
                        b.getTranslation().getDistance(currentPose.getTranslation())))
            .orElse(FieldConstants.Reef.centerFaces[0]); // fallback if empty

    // Return the pathfinding command
    return AutoBuilder.pathfindToPose(
        closestFace,
        new PathConstraints(3.0, 3, 540, 700), // Speed, accel, angular vel, angular accel
        0.0 // End velocity (stop at the pose)
        );
  }

  public static Command autoAlignToClosestReefBranch(Drive drivetrain, ReefHeight targetLevel) {
    Pose2d robotPose = drivetrain.getPose();

    int closestBranchIndex = -1;
    double minDistance = Double.MAX_VALUE;

    for (int i = 0; i < FieldConstants.Reef.branchPositions.size(); i++) {
      Pose2d branchPose = FieldConstants.Reef.branchPositions.get(i).get(targetLevel).toPose2d();

      double distance = robotPose.getTranslation().getDistance(branchPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestBranchIndex = i;
      }
    }

    Pose2d closestTargetPose =
        FieldConstants.Reef.branchPositions.get(closestBranchIndex).get(targetLevel).toPose2d();

    return AutoBuilder.pathfindToPose(
        closestTargetPose, new PathConstraints(3.0, 3.0, 3.0, 3.0), 0.0);
  }

  public static Command autoAlignToClosestBranch(Drive drivetrain, boolean goLeft) {
    Pose2d currentPose = drivetrain.getPose();

    // Find closest reef face
    Pose2d closestFace = FieldConstants.Reef.centerFaces[0];
    double minDistance = currentPose.getTranslation().getDistance(closestFace.getTranslation());

    for (Pose2d face : FieldConstants.Reef.centerFaces) {
      double distance = currentPose.getTranslation().getDistance(face.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestFace = face;
      }
    }

    // Determine face index
    int faceIndex = -1;
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      if (FieldConstants.Reef.centerFaces[i] == closestFace) {
        faceIndex = i;
        break;
      }
    }

    // Get the LEFT or RIGHT branch (pole) pose
    int branchIndex = goLeft ? faceIndex * 2 + 1 : faceIndex * 2;

    Pose2d branchPose =
        FieldConstants.Reef.branchPositions
            .get(branchIndex)
            .get(FieldConstants.ReefHeight.L1) // Just for X/Y
            .toPose2d();

    // Set rotation to match the reef face direction
    Pose2d targetPose = new Pose2d(branchPose.getTranslation(), closestFace.getRotation());

    return AutoBuilder.pathfindToPose(targetPose, new PathConstraints(3.0, 3.0, 3.0, 3.0), 0.0);
  }

  public static Pose2d getClosestBranchPose(Pose2d currentPose, boolean goLeft) {
    // Find closest face
    Pose2d closestFace = FieldConstants.Reef.centerFaces[0];
    double minDistance = currentPose.getTranslation().getDistance(closestFace.getTranslation());

    for (Pose2d face : FieldConstants.Reef.centerFaces) {
      double distance = currentPose.getTranslation().getDistance(face.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestFace = face;
      }
    }

    // Get index of that face
    int faceIndex = -1;
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      if (FieldConstants.Reef.centerFaces[i] == closestFace) {
        faceIndex = i;
        break;
      }
    }

    // Get branch index and return L1 pose2d for simplicity
    int branchIndex = goLeft ? faceIndex * 2 + 1 : faceIndex * 2;
    return FieldConstants.Reef.branchPositions
        .get(branchIndex)
        .get(FieldConstants.ReefHeight.L1)
        .toPose2d();
  }
}
