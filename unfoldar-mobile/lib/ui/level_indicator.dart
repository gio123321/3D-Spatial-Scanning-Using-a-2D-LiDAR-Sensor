import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../scan/scan_controller.dart';

/// Measured IMU pitch and roll with the rig standing level.
const double kLevelNeutralPitchDeg = 1.12;
const double kLevelNeutralRollDeg = -179.89;

/// Angular tolerance around the measured neutral position.
const double kLevelGoodDeg = 1.5;
const double kLevelOkDeg = 5.0;

/// Shortest signed angle difference in [-180, 180), including roll wraparound.
double _angleDiffDeg(double a, double b) {
  final d = (b - a + 540.0) % 360.0 - 180.0;
  return d;
}

/// Bubble level driven by the rig IMU pitch and roll.
class LevelIndicator extends StatelessWidget {
  final RigTelemetry telemetry;
  // Keeps the details sheet subscribed to live readings.
  final ScanController controller;
  const LevelIndicator({
    super.key,
    required this.telemetry,
    required this.controller,
  });

  @override
  Widget build(BuildContext context) {
    if (!telemetry.imuPresent) {
      return _disabledDial(context);
    }

    final pitchOff = telemetry.pitch - kLevelNeutralPitchDeg;
    final rollOff = _angleDiffDeg(kLevelNeutralRollDeg, telemetry.roll);
    final offDeg = math.sqrt(pitchOff * pitchOff + rollOff * rollOff);
    final status = offDeg <= kLevelGoodDeg
        ? _LevelStatus.good
        : offDeg <= kLevelOkDeg
        ? _LevelStatus.ok
        : _LevelStatus.bad;

    return InkWell(
      customBorder: const CircleBorder(),
      onTap: () => _showDetails(context),
      child: Padding(
        padding: const EdgeInsets.all(4),
        child: SizedBox(
          width: 88,
          height: 88,
          child: _Bubble(pitchOff: pitchOff, rollOff: rollOff, status: status),
        ),
      ),
    );
  }

  Widget _disabledDial(BuildContext context) => Padding(
    padding: const EdgeInsets.all(4),
    child: SizedBox(
      width: 88,
      height: 88,
      child: Container(
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          border: Border.all(color: Theme.of(context).colorScheme.outline),
        ),
        child: Icon(
          Icons.sensors_off,
          size: 20,
          color: Theme.of(context).colorScheme.outline,
        ),
      ),
    ),
  );

  void _showDetails(BuildContext context) {
    showModalBottomSheet(
      context: context,
      showDragHandle: true,
      // Rebuild the open sheet when live telemetry changes.
      builder: (context) => ListenableBuilder(
        listenable: controller,
        builder: (context, _) {
          final t = controller.telemetry;
          return Padding(
            padding: const EdgeInsets.fromLTRB(20, 0, 20, 24),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Orientation',
                  style: Theme.of(context).textTheme.titleLarge,
                ),
                const SizedBox(height: 12),
                _row(
                  context,
                  'Pitch',
                  '${t.pitch.toStringAsFixed(2)}° '
                  '(${_signed(t.pitch - kLevelNeutralPitchDeg)}° off level)',
                ),
                _row(
                  context,
                  'Roll',
                  '${t.roll.toStringAsFixed(2)}° '
                  '(${_signed(_angleDiffDeg(kLevelNeutralRollDeg, t.roll))}° off level)',
                ),
                const Divider(height: 24),
                Text(
                  'Encoders',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
                const SizedBox(height: 8),
                if (t.encAzPresent)
                  _row(
                    context,
                    'Azimuth encoder',
                    '${t.encAzDeg.toStringAsFixed(2)}°',
                  ),
                if (t.encTiltPresent)
                  _row(
                    context,
                    'Tilt encoder',
                    '${t.encTiltDeg.toStringAsFixed(2)}°',
                  ),
                if (!t.encAzPresent && !t.encTiltPresent)
                  Text(
                    'No encoders reporting.',
                    style: TextStyle(
                      color: Theme.of(context).colorScheme.outline,
                    ),
                  ),
                const Divider(height: 24),
                _row(
                  context,
                  'Commanded azimuth',
                  '${t.cmdAz.toStringAsFixed(1)}°',
                ),
                _row(
                  context,
                  'Commanded tilt',
                  '${t.cmdTilt.toStringAsFixed(1)}°',
                ),
              ],
            ),
          );
        },
      ),
    );
  }

  String _signed(double v) => v >= 0 ? '+${v.toStringAsFixed(2)}' : v.toStringAsFixed(2);

  // Right-align values without depending on their changing text width.
  Widget _row(BuildContext context, String label, String value) => Padding(
    padding: const EdgeInsets.symmetric(vertical: 4),
    child: Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: TextStyle(color: Theme.of(context).colorScheme.outline)),
        const SizedBox(width: 12),
        Expanded(
          child: Text(
            value,
            textAlign: TextAlign.right,
            style: const TextStyle(fontWeight: FontWeight.w600),
          ),
        ),
      ],
    ),
  );
}

enum _LevelStatus { good, ok, bad }

/// Animated bubble showing pitch and roll relative to neutral.
class _Bubble extends StatefulWidget {
  final double pitchOff;
  final double rollOff;
  final _LevelStatus status;
  const _Bubble({
    required this.pitchOff,
    required this.rollOff,
    required this.status,
  });

  @override
  State<_Bubble> createState() => _BubbleState();
}

class _BubbleState extends State<_Bubble> with SingleTickerProviderStateMixin {
  late final AnimationController _pulse = AnimationController(
    vsync: this,
    duration: const Duration(milliseconds: 1400),
  )..repeat(reverse: true);

  @override
  void dispose() {
    _pulse.dispose();
    super.dispose();
  }

  Color _color(BuildContext context) => switch (widget.status) {
    _LevelStatus.good => Colors.greenAccent.shade400,
    _LevelStatus.ok => Colors.amberAccent.shade400,
    _LevelStatus.bad => Colors.redAccent.shade200,
  };

  @override
  Widget build(BuildContext context) {
    final color = _color(context);
    // Map +/-30 degrees from neutral to the dial radius.
    const maxDeg = 30.0;
    final dx = (widget.rollOff / maxDeg).clamp(-1.0, 1.0);
    final dy = (-widget.pitchOff / maxDeg).clamp(-1.0, 1.0);

    return AnimatedBuilder(
      animation: _pulse,
      builder: (context, _) {
        return CustomPaint(
          painter: _BubblePainter(
            dx: dx,
            dy: dy,
            color: color,
            glow: widget.status == _LevelStatus.good ? _pulse.value : 0,
          ),
        );
      },
    );
  }
}

class _BubblePainter extends CustomPainter {
  final double dx, dy;
  final Color color;
  final double glow;
  _BubblePainter({
    required this.dx,
    required this.dy,
    required this.color,
    required this.glow,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = size.center(Offset.zero);
    final r = size.shortestSide / 2;

    final track = Paint()
      ..color = color.withValues(alpha: 0.15)
      ..style = PaintingStyle.fill;
    canvas.drawCircle(center, r, track);

    final ring = Paint()
      ..color = color.withValues(alpha: 0.6)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;
    canvas.drawCircle(center, r - 1, ring);

    final cross = Paint()
      ..color = color.withValues(alpha: 0.35)
      ..strokeWidth = 1;
    canvas.drawLine(
      Offset(center.dx - r * 0.5, center.dy),
      Offset(center.dx + r * 0.5, center.dy),
      cross,
    );
    canvas.drawLine(
      Offset(center.dx, center.dy - r * 0.5),
      Offset(center.dx, center.dy + r * 0.5),
      cross,
    );

    final dotCenter = Offset(center.dx + dx * (r - 8), center.dy + dy * (r - 8));

    if (glow > 0) {
      final glowPaint = Paint()
        ..color = color.withValues(alpha: 0.25 * (0.4 + 0.6 * glow))
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 6);
      canvas.drawCircle(dotCenter, 10 + 3 * glow, glowPaint);
    }

    canvas.drawCircle(dotCenter, 6, Paint()..color = color);
    canvas.drawCircle(
      dotCenter,
      6,
      Paint()
        ..color = Colors.black.withValues(alpha: 0.25)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1,
    );
  }

  @override
  bool shouldRepaint(covariant _BubblePainter oldDelegate) =>
      oldDelegate.dx != dx ||
      oldDelegate.dy != dy ||
      oldDelegate.color != color ||
      oldDelegate.glow != glow;
}
