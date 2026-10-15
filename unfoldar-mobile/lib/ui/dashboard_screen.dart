import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../scan/scan_controller.dart';
import 'battery_gauge.dart';
import 'connect_screen.dart';
import 'connection_status.dart';
import 'level_indicator.dart';
import 'point_cloud_fullscreen_screen.dart';
import 'point_preview.dart';
import 'scan_status.dart';
import 'scans_screen.dart';

class DashboardScreen extends StatefulWidget {
  final ScanController controller;
  const DashboardScreen({super.key, required this.controller});

  @override
  State<DashboardScreen> createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen>
    with WidgetsBindingObserver {
  static const _screenChannel = MethodChannel('unfoldar/screen');
  bool _awake = false;

  void _syncAwake() {
    final awake = widget.controller.isBusy;
    if (_awake == awake) return;
    _awake = awake;
    _screenChannel.invokeMethod<void>('keepAwake', awake).catchError((_) {});
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state == AppLifecycleState.paused ||
        state == AppLifecycleState.detached) {
      unawaited(widget.controller.interrupt('Application moved to background'));
    }
  }

  Timer? _previewTimer;
  final _cmdController = TextEditingController();
  LinkPhase? _lastLinkPhase;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _lastLinkPhase = widget.controller.linkPhase;
    widget.controller.addListener(_onChanged);
    _syncPreviewTimer();
    _syncAwake();
  }

  void _onChanged() {
    if (!mounted) return;
    final c = widget.controller;

    // Show connection loss even when no scan is active.
    if (_lastLinkPhase == LinkPhase.connected &&
        c.linkPhase == LinkPhase.disconnected) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (!mounted) return;
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(
            content: Text('Rig disconnected'),
            backgroundColor: Colors.redAccent,
          ),
        );
      });
    }
    _lastLinkPhase = c.linkPhase;

    setState(() {});
    _syncPreviewTimer();
    _syncAwake();
  }

  // Refresh the point preview on a timer only while scanning.
  void _syncPreviewTimer() {
    final running = widget.controller.scanPhase == ScanPhase.running;
    if (running && _previewTimer == null) {
      _previewTimer = Timer.periodic(const Duration(milliseconds: 150), (_) {
        if (mounted) setState(() {});
      });
    } else if (!running && _previewTimer != null) {
      _previewTimer!.cancel();
      _previewTimer = null;
    }
  }

  void _sendCommand() {
    final cmd = _cmdController.text.trim();
    if (cmd.isEmpty) return;
    widget.controller.sendRaw(cmd);
    _cmdController.clear();
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _screenChannel.invokeMethod<void>('keepAwake', false).catchError((_) {});
    _previewTimer?.cancel();
    widget.controller.removeListener(_onChanged);
    _cmdController.dispose();
    super.dispose();
  }

  /// Start a scan with an optional name; blank uses a timestamp.
  Future<void> _startScan(ScanController c) async {
    final name = await showDialog<String>(
      context: context,
      builder: (context) => const _ScanNameDialog(),
    );
    if (name == null) return; // cancelled
    c.startScan(name: name);
  }

  void _openConsole(BuildContext context) {
    showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      builder: (sheetContext) => DraggableScrollableSheet(
        initialChildSize: 0.6,
        minChildSize: 0.3,
        maxChildSize: 0.9,
        expand: false,
        builder: (context, scrollController) => ListenableBuilder(
          listenable: widget.controller,
          builder: (context, _) {
            final c = widget.controller;
            return Padding(
              padding: const EdgeInsets.fromLTRB(16, 12, 16, 12),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Row(
                    children: [
                      const Icon(Icons.terminal, size: 18),
                      const SizedBox(width: 8),
                      Expanded(
                        child: Text(
                          'Diagnostics · 1.0.0-rc.1',
                          maxLines: 1,
                          overflow: TextOverflow.ellipsis,
                          style: Theme.of(context).textTheme.titleMedium,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  Expanded(
                    child: ListView.builder(
                      controller: scrollController,
                      reverse: true,
                      itemCount: c.log.length,
                      itemBuilder: (context, i) {
                        final line = c.log[c.log.length - 1 - i];
                        return Text(
                          line,
                          style: const TextStyle(
                            fontFamily: 'monospace',
                            fontSize: 11,
                          ),
                        );
                      },
                    ),
                  ),
                  const SizedBox(height: 8),
                  _debugConsole(c),
                ],
              ),
            );
          },
        ),
      ),
    );
  }

  void _openConnectScreen() {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => ConnectScreen(controller: widget.controller),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final c = widget.controller;
    final connected = c.linkPhase == LinkPhase.connected;
    final canStart = connected && !c.isBusy && c.saveError == null;

    return Scaffold(
      appBar: AppBar(
        title: const Text('UnfoLDAR'),
        actions: [
          // Saved scans remain available without a rig connection.
          Opacity(
            opacity: connected ? 1.0 : 0.35,
            child: IconButton(
              icon: const Icon(Icons.terminal),
              tooltip: 'Console',
              onPressed: connected ? () => _openConsole(context) : null,
            ),
          ),
          IconButton(
            icon: const Icon(Icons.folder_open),
            tooltip: 'Saved scans',
            onPressed: () => Navigator.of(
              context,
            ).push(MaterialPageRoute(builder: (_) => const ScansScreen())),
          ),
          if (connected)
            IconButton(
              icon: const Icon(Icons.link_off),
              tooltip: 'Disconnect',
              onPressed: c.disconnect,
            ),
        ],
      ),
      body: SafeArea(
        // AbsorbPointer keeps the inline preview out of the scroll gesture
        // arena. Tapping opens the separate interactive fullscreen viewer.
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(12),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              _statusCard(c, connected),
              if (c.saveError != null) ...[
                Text('Save failed. Your capture is retained. ${c.saveError}'),
                FilledButton.icon(
                  onPressed: c.retrySave,
                  icon: const Icon(Icons.refresh),
                  label: const Text('Retry save'),
                ),
              ],
              if (c.interruptionReason != null)
                Text('Partial capture: ${c.interruptionReason}'),
              const SizedBox(height: 8),
              AnimatedOpacity(
                duration: const Duration(milliseconds: 250),
                opacity: connected ? 1.0 : 0.35,
                child: IgnorePointer(
                  ignoring: !connected,
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      _scanControls(c, canStart),
                      const SizedBox(height: 8),
                      ScanProgressBar(controller: c),
                      const SizedBox(height: 8),
                      if (c.latestImage != null) ...[
                        ClipRRect(
                          borderRadius: BorderRadius.circular(8),
                          child: Image.memory(
                            c.latestImage!,
                            fit: BoxFit.fitWidth,
                            gaplessPlayback: true,
                          ),
                        ),
                        const SizedBox(height: 8),
                      ],
                      _pointCloudPreview(c),
                    ],
                  ),
                ),
              ),
              const SizedBox(height: 4),
              Text(
                '${c.points.length} points'
                '${c.triggeredCount > 0 ? " · ${c.triggeredCount} shot(s) triggered" : ""}'
                // Use triggeredCount as the stable photo-transfer total.
                '${c.scanPhase == ScanPhase.draining ? " · received ${c.drainedCount}/${c.triggeredCount == 0 ? "?" : c.triggeredCount} photos" : ""}'
                '${c.drainFailedCount > 0 ? " · ${c.drainFailedCount} failed" : ""}'
                '${c.useCamera && c.scanPhase != ScanPhase.idle && !c.cameraArmedThisScan ? " · camera did not arm - lidar only" : ""}',
                style: TextStyle(
                  color:
                      (c.useCamera &&
                          c.scanPhase != ScanPhase.idle &&
                          !c.cameraArmedThisScan)
                      ? Colors.orangeAccent
                      : Colors.grey,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  /// Non-interactive cloud preview; open the full-screen view for gestures.
  Widget _pointCloudPreview(ScanController c) {
    return GestureDetector(
      onTap: () => Navigator.of(context).push(
        MaterialPageRoute(
          builder: (_) => PointCloudFullscreenScreen(
            points: c.points,
            live: c.scanPhase == ScanPhase.running,
          ),
        ),
      ),
      child: SizedBox(
        height: 260,
        child: Stack(
          children: [
            Positioned.fill(
              child: Container(
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.black26),
                  borderRadius: BorderRadius.circular(8),
                ),
                clipBehavior: Clip.antiAlias,
                child: AbsorbPointer(child: PointPreview(points: c.points)),
              ),
            ),
            Positioned(
              right: 8,
              bottom: 8,
              child: DecoratedBox(
                decoration: BoxDecoration(
                  color: Colors.black.withValues(alpha: 0.55),
                  borderRadius: BorderRadius.circular(6),
                ),
                child: const Padding(
                  padding: EdgeInsets.symmetric(horizontal: 6, vertical: 3),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.fullscreen, size: 14, color: Colors.white70),
                      SizedBox(width: 3),
                      Text(
                        'Tap to view',
                        style: TextStyle(color: Colors.white70, fontSize: 11),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  /// Status pills and live level indicator.
  Widget _statusCard(ScanController c, bool connected) {
    return Card(
      margin: EdgeInsets.zero,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                mainAxisSize: MainAxisSize.min,
                children: [
                  // Keep the connection control available while disconnected.
                  ConnectionStatus(
                    phase: c.linkPhase,
                    onTap: c.linkPhase == LinkPhase.disconnected
                        ? _openConnectScreen
                        : null,
                  ),
                  const SizedBox(height: 6),
                  BatteryGauge(
                    battery: c.battery,
                    telemetry: c.telemetry,
                    controller: c,
                  ),
                  const SizedBox(height: 6),
                  ScanPhaseChip(controller: c),
                ],
              ),
            ),
            const SizedBox(width: 12),
            LevelIndicator(telemetry: c.telemetry, controller: c),
          ],
        ),
      ),
    );
  }

  /// Sends a raw rig command and displays the reply in the console.
  Widget _debugConsole(ScanController c) {
    final connected = c.linkPhase == LinkPhase.connected;
    return Row(
      children: [
        Expanded(
          child: TextField(
            controller: _cmdController,
            enabled: connected,
            decoration: const InputDecoration(
              isDense: true,
              border: OutlineInputBorder(),
              hintText: 'PING / CAMPING / STATUS / CAMTEST / ...',
              contentPadding: EdgeInsets.symmetric(horizontal: 8, vertical: 8),
            ),
            style: const TextStyle(fontFamily: 'monospace', fontSize: 12),
            textInputAction: TextInputAction.send,
            onSubmitted: (_) => _sendCommand(),
          ),
        ),
        IconButton(
          icon: const Icon(Icons.send),
          tooltip: 'Send',
          onPressed: connected ? _sendCommand : null,
        ),
      ],
    );
  }

  /// Camera status, capture toggle and Start/Stop controls.
  Widget _scanControls(ScanController c, bool canStart) {
    return Row(
      children: [
        _cameraStatusIcon(c),
        const SizedBox(width: 4),
        Switch(
          value: c.useCamera,
          onChanged: canStart ? (v) => setState(() => c.useCamera = v) : null,
        ),
        const SizedBox(width: 8),
        Expanded(
          child: c.isBusy
              ? FilledButton.tonalIcon(
                  onPressed:
                      {
                        ScanPhase.running,
                        ScanPhase.preparing,
                        ScanPhase.draining,
                      }.contains(c.scanPhase)
                      ? c.stopScan
                      : null,
                  icon: const Icon(Icons.stop),
                  label: Text(
                    c.scanPhase == ScanPhase.draining ? 'Draining…' : 'Stop',
                  ),
                )
              : FilledButton.icon(
                  onPressed: canStart ? () => _startScan(c) : null,
                  icon: const Icon(Icons.play_arrow),
                  label: const Text('Start Scan'),
                ),
        ),
      ],
    );
  }

  /// Camera indicator: orange when disabled, red when unreachable, green when ready.
  Widget _cameraStatusIcon(ScanController c) {
    final Color color;
    final IconData icon;
    final String label;
    if (!c.useCamera) {
      color = Colors.orange;
      icon = Icons.photo_camera_outlined;
      label = 'Capture photos is off';
    } else if (c.cameraLinkStatus == CameraLinkStatus.ok) {
      color = Colors.green;
      icon = Icons.photo_camera;
      label = 'Camera connected';
    } else {
      color = Colors.red;
      icon = Icons.no_photography;
      label = c.cameraLinkStatus == CameraLinkStatus.unreachable
          ? 'Camera unreachable'
          : 'Camera not checked yet';
    }
    return Tooltip(
      message: label,
      child: CircleAvatar(
        radius: 16,
        backgroundColor: color.withValues(alpha: 0.15),
        child: Icon(icon, size: 18, color: color),
      ),
    );
  }
}

/// Keeps the name controller alive through the dialog's exit animation.
class _ScanNameDialog extends StatefulWidget {
  const _ScanNameDialog();

  @override
  State<_ScanNameDialog> createState() => _ScanNameDialogState();
}

class _ScanNameDialogState extends State<_ScanNameDialog> {
  final _controller = TextEditingController();

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  void _submit() => Navigator.of(context).pop(_controller.text);

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('Name this scan'),
      content: TextField(
        controller: _controller,
        autofocus: true,
        textCapitalization: TextCapitalization.words,
        decoration: const InputDecoration(
          hintText: 'e.g. living room (optional)',
        ),
        onSubmitted: (_) => _submit(),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.of(context).pop(),
          child: const Text('Cancel'),
        ),
        FilledButton(onPressed: _submit, child: const Text('Start')),
      ],
    );
  }
}
