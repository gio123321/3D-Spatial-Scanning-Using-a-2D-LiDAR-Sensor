import 'package:flutter/material.dart';

import '../scan/scan_controller.dart';
import 'status_pill.dart';

/// Connection status pill, optionally tappable via [onTap].
class ConnectionStatus extends StatelessWidget {
  final LinkPhase phase;
  final VoidCallback? onTap;
  const ConnectionStatus({super.key, required this.phase, this.onTap});

  @override
  Widget build(BuildContext context) {
    final Color color;
    final IconData icon;
    final String label;
    switch (phase) {
      case LinkPhase.connected:
        color = Colors.greenAccent.shade700;
        icon = Icons.bluetooth_connected;
        label = 'Connected';
      case LinkPhase.connecting:
        color = Colors.amberAccent.shade700;
        icon = Icons.bluetooth_searching;
        label = 'Connecting…';
      case LinkPhase.disconnected:
        color = Colors.redAccent;
        icon = Icons.bluetooth_disabled;
        label = 'Not connected';
    }

    return StatusPill(
      icon: icon,
      color: color,
      label: label,
      onTap: onTap,
      pulse: phase == LinkPhase.connected,
      trailing: onTap != null
          ? Icon(Icons.chevron_right, size: 16, color: color)
          : null,
    );
  }
}
