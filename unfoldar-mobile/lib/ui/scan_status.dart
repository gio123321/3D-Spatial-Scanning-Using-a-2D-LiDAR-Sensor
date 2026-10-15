import 'package:flutter/material.dart';

import '../scan/scan_controller.dart';
import 'status_pill.dart';

/// Displays connection and scan phase below the battery gauge.
class ScanPhaseChip extends StatelessWidget {
  final ScanController controller;
  const ScanPhaseChip({super.key, required this.controller});

  @override
  Widget build(BuildContext context) {
    final c = controller;
    final IconData icon;
    final Color color;
    final String label;

    // Connection status takes priority over the default idle phase.
    if (c.scanPhase == ScanPhase.failed ||
        c.scanPhase == ScanPhase.interrupted) {
      icon = Icons.warning_amber;
      color = Colors.orange;
      label = c.saveError != null
          ? 'Save failed — Retry available'
          : 'Capture interrupted';
    } else if (c.scanPhase == ScanPhase.preparing ||
        c.scanPhase == ScanPhase.stopping ||
        c.scanPhase == ScanPhase.saving) {
      icon = Icons.hourglass_top;
      color = Colors.teal;
      label = switch (c.scanPhase) {
        ScanPhase.preparing => 'Preparing scan',
        ScanPhase.stopping => 'Stopping and parking',
        _ => 'Saving scan',
      };
    } else if (c.scanPhase == ScanPhase.saved) {
      icon = Icons.check_circle;
      color = Colors.teal;
      label = c.interruptionReason == null
          ? 'Scan saved'
          : 'Partial scan saved';
    } else if (c.linkPhase != LinkPhase.connected) {
      icon = Icons.bluetooth_disabled;
      color = Theme.of(context).colorScheme.outline;
      label = 'Not connected';
    } else if (c.scanPhase == ScanPhase.draining) {
      icon = Icons.cloud_download;
      color = Colors.cyanAccent.shade400;
      label = 'Draining photos';
    } else if (c.scanPhase == ScanPhase.saved) {
      icon = Icons.check_circle;
      color = Colors.greenAccent.shade400;
      label = 'Scan saved';
    } else if (c.scanPhase == ScanPhase.idle) {
      icon = Icons.pause_circle_outline;
      color = Theme.of(context).colorScheme.outline;
      label = 'Idle';
    } else {
      switch (c.telemetry.state) {
        case 'WARMING':
          icon = Icons.hourglass_top;
          color = Colors.amberAccent.shade400;
          label = 'Warming up';
        case 'SCANNING':
          icon = Icons.radar;
          color = Colors.tealAccent.shade400;
          label = 'Scanning';
        case 'RETURNING':
          icon = Icons.replay;
          color = Colors.amberAccent.shade400;
          label = 'Returning to start';
        default:
          icon = Icons.settings_input_antenna;
          color = Colors.tealAccent.shade400;
          label = 'Working';
      }
    }

    return StatusPill(icon: icon, color: color, label: label);
  }
}

/// Scan and photo-transfer progress; hidden while idle or saved.
class ScanProgressBar extends StatelessWidget {
  final ScanController controller;
  const ScanProgressBar({super.key, required this.controller});

  @override
  Widget build(BuildContext context) {
    final c = controller;
    if (c.scanPhase != ScanPhase.running && c.scanPhase != ScanPhase.draining) {
      return const SizedBox.shrink();
    }

    final progress = _progress();
    final label = _progressLabel();

    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        ClipRRect(
          borderRadius: BorderRadius.circular(4),
          child: LinearProgressIndicator(
            value: progress,
            minHeight: 6,
            backgroundColor: Theme.of(
              context,
            ).colorScheme.surfaceContainerHighest,
          ),
        ),
        if (label != null) ...[
          const SizedBox(height: 2),
          Text(
            label,
            style: TextStyle(
              fontSize: 11,
              color: Theme.of(context).colorScheme.outline,
            ),
          ),
        ],
      ],
    );
  }

  /// Warmup and return progress are indeterminate. Photo transfer progress
  /// uses the known trigger count rather than the growing metadata count.
  double? _progress() {
    final c = controller;
    if (c.scanPhase == ScanPhase.draining) {
      if (c.triggeredCount <= 0) return null;
      return ((c.drainedCount + c.drainFailedCount) / c.triggeredCount).clamp(
        0.0,
        1.0,
      );
    }
    if (c.scanPhase == ScanPhase.running && c.telemetry.state == 'SCANNING') {
      // Only capture has determinate azimuth progress; return motion rewinds.
      return (c.telemetry.cmdAz / 360.0).clamp(0.0, 1.0);
    }
    return null;
  }

  String? _progressLabel() {
    final c = controller;
    if (c.scanPhase == ScanPhase.draining) {
      final expected = c.triggeredCount;
      final failedSuffix = c.drainFailedCount > 0
          ? ' · ${c.drainFailedCount} failed'
          : '';
      return expected > 0
          ? '${c.drainedCount}/$expected photo(s)$failedSuffix'
          : 'waiting for photo count…';
    }
    if (c.scanPhase == ScanPhase.running && c.telemetry.state == 'SCANNING') {
      return '${c.telemetry.cmdAz.clamp(0, 360).toStringAsFixed(0)}° / 360°';
    }
    return null;
  }
}
