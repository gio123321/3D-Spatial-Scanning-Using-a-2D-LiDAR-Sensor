import 'package:flutter/material.dart';

import '../scan/scan_controller.dart';
import 'status_pill.dart';

/// Approximate SOC fallback from bus-side INA226 voltage. VBUS can differ
/// from pack voltage because of load and the charging path; prefer the dedicated
/// BatteryStatus reading when available. Uses the firmware generic 2S curve.
const _batteryFallbackCurve = <({double volts, double fraction})>[
  (volts: 8.40, fraction: 1.00),
  (volts: 8.12, fraction: 0.90),
  (volts: 7.96, fraction: 0.80),
  (volts: 7.84, fraction: 0.70),
  (volts: 7.74, fraction: 0.60),
  (volts: 7.64, fraction: 0.50),
  (volts: 7.58, fraction: 0.40),
  (volts: 7.54, fraction: 0.30),
  (volts: 7.46, fraction: 0.20),
  (volts: 7.10, fraction: 0.10),
  (volts: 6.80, fraction: 0.05),
  (volts: 6.00, fraction: 0.00),
];

double batteryFraction(double busV) {
  if (busV >= _batteryFallbackCurve.first.volts) return 1;
  if (busV <= _batteryFallbackCurve.last.volts) return 0;

  for (var i = 0; i < _batteryFallbackCurve.length - 1; i++) {
    final high = _batteryFallbackCurve[i];
    final low = _batteryFallbackCurve[i + 1];
    if (busV <= high.volts && busV >= low.volts) {
      final position = (busV - low.volts) / (high.volts - low.volts);
      return low.fraction + position * (high.fraction - low.fraction);
    }
  }
  return 0;
}

/// Battery percentage with voltage and current in the details sheet.
class BatteryGauge extends StatelessWidget {
  final BatteryStatus? battery;
  final RigTelemetry telemetry;
  // Keeps the details sheet subscribed to live readings.
  final ScanController controller;
  const BatteryGauge({
    super.key,
    this.battery,
    required this.telemetry,
    required this.controller,
  });

  bool get _hasRealSoc => battery != null;

  double get _percent =>
      battery?.socPercent ?? batteryFraction(telemetry.busV) * 100;

  @override
  Widget build(BuildContext context) {
    if (!_hasRealSoc && !telemetry.inaPresent) {
      return StatusPill(
        icon: Icons.battery_unknown,
        color: Theme.of(context).colorScheme.outline,
        label: 'No power data',
      );
    }

    final percent = _percent;
    final Color color;
    final IconData icon;
    if (percent > 55) {
      color = Colors.greenAccent.shade400;
      icon = Icons.battery_full;
    } else if (percent > 25) {
      color = Colors.amberAccent.shade400;
      icon = Icons.battery_4_bar;
    } else {
      color = Colors.redAccent.shade200;
      icon = Icons.battery_alert;
    }

    final label = _hasRealSoc ? '${percent.round()}%' : '~${percent.round()}%';

    return StatusPill(
      icon: icon,
      color: color,
      label: label,
      onTap: () => _showDetails(context),
    );
  }

  void _showDetails(BuildContext context) {
    showModalBottomSheet(
      context: context,
      showDragHandle: true,
      // Rebuild the open sheet when live battery readings change.
      builder: (context) => ListenableBuilder(
        listenable: controller,
        builder: (context, _) {
          final b = controller.battery;
          final t = controller.telemetry;
          final percent = b?.socPercent ?? batteryFraction(t.busV) * 100;
          return Padding(
            padding: const EdgeInsets.fromLTRB(20, 0, 20, 24),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Battery', style: Theme.of(context).textTheme.titleLarge),
                const SizedBox(height: 4),
                if (b != null)
                  _detailRow(
                    context,
                    Icons.battery_std,
                    'State of charge',
                    '${b.socPercent.toStringAsFixed(1)}%',
                  )
                else
                  Text(
                    '~${percent.round()}% estimated from bus voltage - the '
                    'rig has no dedicated pack SOC sensor wired up yet.',
                    style: Theme.of(context).textTheme.bodySmall?.copyWith(
                      color: Theme.of(context).colorScheme.outline,
                    ),
                  ),
                const SizedBox(height: 12),
                _detailRow(
                  context,
                  Icons.bolt,
                  b != null ? 'Pack voltage' : 'Bus voltage',
                  '${(b?.packVoltage ?? t.busV).toStringAsFixed(3)} V',
                ),
                _detailRow(
                  context,
                  Icons.speed,
                  'Current',
                  b != null
                      ? '${b.currentMilliA.toStringAsFixed(0)} mA'
                      : '${t.currentA.toStringAsFixed(3)} A',
                ),
                if (b == null)
                  _detailRow(
                    context,
                    Icons.flash_on,
                    'Power draw',
                    '${t.powerW.toStringAsFixed(2)} W',
                  ),
              ],
            ),
          );
        },
      ),
    );
  }

  Widget _detailRow(
    BuildContext context,
    IconData icon,
    String label,
    String value,
  ) => Padding(
    padding: const EdgeInsets.symmetric(vertical: 6),
    child: Row(
      children: [
        Icon(icon, size: 18, color: Theme.of(context).colorScheme.outline),
        const SizedBox(width: 10),
        Expanded(child: Text(label)),
        Text(value, style: const TextStyle(fontWeight: FontWeight.w600)),
      ],
    ),
  );
}
