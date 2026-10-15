import 'dart:math' as math;

/// Rotates about world Z for yaw, then view X for pitch.
/// Returns (viewX, viewDepth, viewZ); callers negate viewZ for screen Y.
/// Angles are in radians. Zero pitch is an elevation view; +/-90 degrees is top-down.
(double viewX, double viewDepth, double viewZ) orbitProject(
  double x,
  double y,
  double z,
  double yaw,
  double pitch,
) {
  final cy = math.cos(yaw), sy = math.sin(yaw);
  final cp = math.cos(pitch), sp = math.sin(pitch);

  final x1 = x * cy - y * sy;
  final y1 = x * sy + y * cy;
  final viewDepth = y1 * cp - z * sp;
  final viewZ = y1 * sp + z * cp;

  return (x1, viewDepth, viewZ);
}
