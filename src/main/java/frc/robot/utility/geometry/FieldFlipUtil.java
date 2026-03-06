package frc.robot.utility.geometry;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility functions for flipping field elements to the correct alliance side.
 *
 * <p>This differences from alliance flip util as it accounts for whether the field is mirrored or
 * rotationally symmetric.
 *
 * <p>This should be used for field elements, not basic robot movement which should use
 * AllianceMirrorUtil.
 */
public class FieldFlipUtil {

  public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

  private FieldFlipUtil() {}

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip() ? FlippingUtil.flipFieldPosition(translation) : translation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? FlippingUtil.flipFieldRotation(rotation) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    Translation2d flipped2d = apply(translation.toTranslation2d());
    return shouldFlip()
        ? new Translation3d(flipped2d.getX(), flipped2d.getY(), translation.getZ())
        : translation;
  }

  public static Rotation3d apply(Rotation3d rotation) {
    Rotation2d flipped2d = apply(rotation.toRotation2d());
    return shouldFlip()
        ? new Rotation3d(rotation.getX(), rotation.getY(), flipped2d.getRadians())
        : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  /** Get whether to flip. If alliance is blue or unknown don't flip, if it is red then flip. */
  public static boolean shouldFlip() {
    return !DriverStation.getAlliance().orElse(DEFAULT_ALLIANCE).equals(DEFAULT_ALLIANCE);
  }
}
