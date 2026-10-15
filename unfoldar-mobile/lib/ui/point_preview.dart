import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../scan/orbit_projection.dart';
import '../scan/session_writer.dart';

/// Live point-cloud preview with orbit, pan and zoom.
/// Projection is implemented in orbit_projection.dart.
class PointPreview extends StatefulWidget {
  final List<PointRecord> points;
  const PointPreview({super.key, required this.points});

  @override
  State<PointPreview> createState() => _PointPreviewState();
}

class _PointPreviewState extends State<PointPreview> {
  static const double _defaultYaw = -math.pi / 4;
  static const double _defaultPitch = 0.45;

  double _yaw = _defaultYaw;
  double _pitch = _defaultPitch;
  double _zoom = 1.0;
  double _zoomBase = 1.0;
  Offset _pan = Offset.zero;

  void _resetView() {
    setState(() {
      _yaw = _defaultYaw;
      _pitch = _defaultPitch;
      _zoom = 1.0;
      _pan = Offset.zero;
    });
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onDoubleTap: _resetView,
      onScaleStart: (_) => _zoomBase = _zoom,
      onScaleUpdate: (details) {
        setState(() {
          if (details.pointerCount >= 2) {
            // Use cumulative scale for zoom and incremental focal-point motion for pan.
            _zoom = (_zoomBase * details.scale).clamp(0.35, 5.0);
            _pan += details.focalPointDelta;
          } else {
            // One finger orbits; multi-touch pans and zooms.
            _yaw += details.focalPointDelta.dx * 0.01;
            _pitch = (_pitch - details.focalPointDelta.dy * 0.01).clamp(
              -math.pi / 2 + 0.05,
              math.pi / 2 - 0.05,
            );
          }
        });
      },
      child: MouseRegion(
        cursor: SystemMouseCursors.grab,
        child: CustomPaint(
          painter: _PointPainter(widget.points, _yaw, _pitch, _zoom, _pan),
          child: const SizedBox.expand(),
        ),
      ),
    );
  }
}

class _PointPainter extends CustomPainter {
  final List<PointRecord> points;
  final double yaw;
  final double pitch;
  final double zoom;
  final Offset pan;
  _PointPainter(this.points, this.yaw, this.pitch, this.zoom, this.pan);

  @override
  void paint(Canvas canvas, Size size) {
    _drawBackground(canvas, size);

    if (points.isEmpty) {
      _drawHint(canvas, size);
      return;
    }

    // Limit preview paint cost; saved clouds retain every point.
    const maxDrawn = 8000;
    final step = points.length > maxDrawn
        ? (points.length / maxDrawn).ceil()
        : 1;

    final vx = <double>[], vz = <double>[], depth = <double>[];
    var minX = double.infinity, maxX = -double.infinity;
    var minZ = double.infinity, maxZ = -double.infinity;
    var minD = double.infinity, maxD = -double.infinity;

    for (var i = 0; i < points.length; i += step) {
      final p = points[i];
      final (x1, d, z1) = orbitProject(p.x, p.y, p.z, yaw, pitch);
      vx.add(x1);
      vz.add(z1);
      depth.add(d);
      if (x1 < minX) minX = x1;
      if (x1 > maxX) maxX = x1;
      if (z1 < minZ) minZ = z1;
      if (z1 > maxZ) maxZ = z1;
      if (d < minD) minD = d;
      if (d > maxD) maxD = d;
    }

    final spanX = (maxX - minX).abs() < 1e-6 ? 1.0 : maxX - minX;
    final spanZ = (maxZ - minZ).abs() < 1e-6 ? 1.0 : maxZ - minZ;
    final span = spanX > spanZ ? spanX : spanZ;
    final scale = (size.shortestSide * 0.85) / span * zoom;
    // Apply screen-space pan to points and axes.
    final cx = size.width / 2 + pan.dx, ccy = size.height / 2 + pan.dy;
    final midX = (minX + maxX) / 2, midZ = (minZ + maxZ) / 2;
    final depthSpan = (maxD - minD).abs() < 1e-6 ? 1.0 : maxD - minD;

    _drawAxes(canvas, scale, cx, ccy, midX, midZ);

    for (var i = 0; i < vx.length; i++) {
      final dx = cx + (vx[i] - midX) * scale;
      final dy = ccy - (vz[i] - midZ) * scale; // screen Y grows down
      final t = ((depth[i] - minD) / depthSpan).clamp(0.0, 1.0);
      final brightness = 0.3 + 0.7 * (1.0 - t);
      final intensity = points[i * step].intensity.clamp(0, 255);
      final color = _pointColor(brightness, intensity / 255.0);
      final radius = 0.8 + 1.1 * (1.0 - t);
      canvas.drawCircle(Offset(dx, dy), radius, Paint()..color = color);
    }
  }

  void _drawBackground(Canvas canvas, Size size) {
    final rect = Offset.zero & size;
    final bg = Paint()
      ..shader = RadialGradient(
        center: Alignment.center,
        radius: 1.1,
        colors: const [Color(0xFF1A2530), Color(0xFF0B0F14)],
      ).createShader(rect);
    canvas.drawRect(rect, bg);
  }

  /// Map depth and LiDAR intensity to the preview color ramp.
  Color _pointColor(double brightness, double intensityFrac) {
    final base = Color.lerp(
      const Color(0xFF1F5A6B),
      const Color(0xFF8FF3E8),
      brightness,
    )!;
    return Color.lerp(base, const Color(0xFFFFD8A0), intensityFrac * 0.35)!;
  }

  /// Draws origin-anchored axes with the same rotation as the points.
  void _drawAxes(
    Canvas canvas,
    double scale,
    double cx,
    double ccy,
    double midX,
    double midZ,
  ) {
    Offset project(double x, double y, double z) {
      final (vx, _, vz) = orbitProject(x, y, z, yaw, pitch);
      return Offset(cx + (vx - midX) * scale, ccy - (vz - midZ) * scale);
    }

    final origin = project(0, 0, 0);
    const axisLen = 0.5; // meters
    final axes = [
      (project(axisLen, 0, 0), const Color(0xFFE05B5B), 'X'),
      (project(0, axisLen, 0), const Color(0xFF5B9DE0), 'Y'),
      (project(0, 0, axisLen), const Color(0xFFE0C85B), 'Z'),
    ];
    for (final (end, color, label) in axes) {
      canvas.drawLine(
        origin,
        end,
        Paint()
          ..color = color
          ..strokeWidth = 2,
      );
      final tp = TextPainter(
        text: TextSpan(
          text: label,
          style: TextStyle(color: color, fontSize: 10, fontWeight: FontWeight.bold),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      tp.paint(canvas, end + const Offset(3, -3));
    }
    canvas.drawCircle(origin, 4, Paint()..color = Colors.orangeAccent);
  }

  void _drawHint(Canvas canvas, Size size) {
    final tp = TextPainter(
      text: const TextSpan(
        text: 'drag to orbit · two fingers to zoom/pan · double-tap to reset',
        style: TextStyle(color: Colors.white38, fontSize: 12),
      ),
      textDirection: TextDirection.ltr,
    )..layout(maxWidth: size.width - 24);
    tp.paint(
      canvas,
      Offset((size.width - tp.width) / 2, (size.height - tp.height) / 2),
    );
  }

  @override
  bool shouldRepaint(covariant _PointPainter oldDelegate) {
    // The controller mutates the same point list, so repaint independently
    // of list identity.
    return oldDelegate.points.length != points.length ||
        oldDelegate.yaw != yaw ||
        oldDelegate.pitch != pitch ||
        oldDelegate.zoom != zoom ||
        oldDelegate.pan != pan;
  }
}
