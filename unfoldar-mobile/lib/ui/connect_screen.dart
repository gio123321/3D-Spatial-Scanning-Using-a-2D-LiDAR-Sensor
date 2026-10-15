import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:permission_handler/permission_handler.dart';

import '../scan/scan_controller.dart';

/// Match HOST_BT_NAME, allowing suffixes added by Android to bonded names.
const String _rigNameNeedle = 'unfoldar';

bool _isRig(BluetoothDevice d) =>
    (d.name ?? '').toLowerCase().contains(_rigNameNeedle);

/// Lists bonded Classic Bluetooth devices. Pair with UnfoLDAR in Android
/// settings before connecting here.
class ConnectScreen extends StatefulWidget {
  final ScanController controller;
  const ConnectScreen({super.key, required this.controller});

  @override
  State<ConnectScreen> createState() => _ConnectScreenState();
}

class _ConnectScreenState extends State<ConnectScreen> {
  List<BluetoothDevice> _allDevices = [];
  bool _loading = false;
  String? _error;
  BluetoothDevice? _connecting;

  @override
  void initState() {
    super.initState();
    _refresh();
  }

  List<BluetoothDevice> get _rigDevices =>
      _allDevices.where(_isRig).toList(growable: false);

  Future<void> _refresh() async {
    setState(() {
      _loading = true;
      _error = null;
    });
    try {
      // The vendored SPP plugin cancels discovery when connecting, which also
      // requires SCAN on Android 12+, even though this picker lists paired devices.
      final permissions = await [
        Permission.bluetoothConnect,
        Permission.bluetoothScan,
      ].request();
      if (permissions.values.any((status) => !status.isGranted)) {
        throw StateError(
          'Bluetooth permission denied. Enable Nearby devices in Android settings.',
        );
      }
      if (await FlutterBluetoothSerial.instance.isEnabled != true) {
        if (await FlutterBluetoothSerial.instance.requestEnable() != true) {
          throw StateError(
            'Bluetooth is disabled. Turn it on to connect to UnfoLDAR.',
          );
        }
      }
      final devices = await widget.controller.bondedDevices();
      if (!mounted) return;
      setState(() => _allDevices = devices);
    } catch (e) {
      if (mounted) setState(() => _error = '$e');
    } finally {
      if (mounted) setState(() => _loading = false);
    }
  }

  Future<void> _connect(BluetoothDevice d) async {
    setState(() => _connecting = d);
    await widget.controller.connect(d);
    if (!mounted) return;
    setState(() => _connecting = null);
    if (widget.controller.linkPhase == LinkPhase.connected) {
      Navigator.of(context).pop();
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Could not connect: ${widget.controller.connectError}'),
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final rigDevices = _rigDevices;
    // Show all bonded devices if none match the rig name.
    final showingAll = rigDevices.isEmpty && _allDevices.isNotEmpty;
    final shown = showingAll ? _allDevices : rigDevices;

    return Scaffold(
      appBar: AppBar(
        title: const Text('Connect to rig'),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _loading ? null : _refresh,
          ),
        ],
      ),
      // Keep controls clear of system bars in edge-to-edge mode.
      body: SafeArea(
        child: _loading
            ? const Center(child: CircularProgressIndicator())
            : _error != null
            ? Center(child: Text('Error: $_error'))
            : _allDevices.isEmpty
            ? _emptyState()
            : Column(
                children: [
                  if (showingAll) _fallbackBanner(),
                  Expanded(
                    child: ListView.separated(
                      padding: const EdgeInsets.symmetric(vertical: 8),
                      itemCount: shown.length,
                      separatorBuilder: (_, _) => const Divider(height: 1),
                      itemBuilder: (context, i) {
                        final d = shown[i];
                        final isRig = _isRig(d);
                        final busy = _connecting == d;
                        return ListTile(
                          leading: CircleAvatar(
                            backgroundColor: isRig
                                ? Theme.of(context).colorScheme.primaryContainer
                                : Theme.of(
                                    context,
                                  ).colorScheme.surfaceContainerHighest,
                            child: Icon(
                              isRig ? Icons.settings_remote : Icons.bluetooth,
                              color: isRig
                                  ? Theme.of(
                                      context,
                                    ).colorScheme.onPrimaryContainer
                                  : Theme.of(
                                      context,
                                    ).colorScheme.onSurfaceVariant,
                            ),
                          ),
                          title: Text(d.name ?? d.address),
                          subtitle: Text(d.address),
                          trailing: busy
                              ? const SizedBox(
                                  width: 20,
                                  height: 20,
                                  child: CircularProgressIndicator(
                                    strokeWidth: 2,
                                  ),
                                )
                              : const Icon(Icons.chevron_right),
                          onTap: _connecting == null ? () => _connect(d) : null,
                        );
                      },
                    ),
                  ),
                ],
              ),
      ),
    );
  }

  Widget _fallbackBanner() => Container(
    width: double.infinity,
    color: Theme.of(context).colorScheme.errorContainer,
    padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
    child: Row(
      children: [
        Icon(
          Icons.info_outline,
          size: 18,
          color: Theme.of(context).colorScheme.onErrorContainer,
        ),
        const SizedBox(width: 8),
        Expanded(
          child: Text(
            'No device named "UnfoLDAR" among your paired devices - '
            'showing everything paired instead.',
            style: TextStyle(
              color: Theme.of(context).colorScheme.onErrorContainer,
              fontSize: 12,
            ),
          ),
        ),
      ],
    ),
  );

  Widget _emptyState() => Center(
    child: Padding(
      padding: const EdgeInsets.all(24),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            Icons.bluetooth_disabled,
            size: 48,
            color: Theme.of(context).colorScheme.outline,
          ),
          const SizedBox(height: 12),
          const Text(
            'No paired devices found.\n\nPair with the rig '
            '("UnfoLDAR") in your phone\'s Bluetooth settings '
            'first, then come back and refresh.',
            textAlign: TextAlign.center,
          ),
        ],
      ),
    ),
  );
}
