import 'dart:async';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:path_provider/path_provider.dart';

import '../bt/rig_link.dart';
import 'geometry.dart';
import 'session_writer.dart';

enum LinkPhase { disconnected, connecting, connected }

/// Camera preflight result; unknown also covers scans without camera capture.
enum CameraLinkStatus { unknown, ok, unreachable }

/// Capture lifecycle, including phone-side transfer and saving phases.
enum ScanPhase {
  idle,
  preparing,
  running,
  stopping,
  draining,
  saving,
  saved,
  interrupted,
  failed,
}

class _ImageAssembly {
  final int totalLength;
  final int expectedCrc;
  final Uint8List bytes;
  int nextOffset = 0;

  _ImageAssembly(this.totalLength, this.expectedCrc)
    : bytes = Uint8List(totalLength);
}

/// Current EVT TELEM fields from the main firmware. Encoder angles are
/// monitoring readings; they do not replace commanded capture geometry.
class RigTelemetry {
  final String state;
  final double cmdAz, cmdTilt;
  final bool imuPresent;
  final double pitch, roll;
  final bool encAzPresent, encTiltPresent;
  final double encAzDeg, encTiltDeg;
  final bool inaPresent;
  final double busV, currentA, powerW;

  const RigTelemetry({
    this.state = 'UNKNOWN',
    this.cmdAz = 0,
    this.cmdTilt = 0,
    this.imuPresent = false,
    this.pitch = 0,
    this.roll = 0,
    this.encAzPresent = false,
    this.encTiltPresent = false,
    this.encAzDeg = 0,
    this.encTiltDeg = 0,
    this.inaPresent = false,
    this.busV = 0,
    this.currentA = 0,
    this.powerW = 0,
  });

  factory RigTelemetry.fromKv(Map<String, String> kv) => RigTelemetry(
    state: kv['state'] ?? 'UNKNOWN',
    cmdAz: double.tryParse(kv['cmd_az'] ?? '') ?? 0,
    cmdTilt: double.tryParse(kv['cmd_tilt'] ?? '') ?? 0,
    imuPresent: kv['imu'] == '1',
    pitch: double.tryParse(kv['pitch'] ?? '') ?? 0,
    roll: double.tryParse(kv['roll'] ?? '') ?? 0,
    encAzPresent: kv['encaz'] == '1',
    encTiltPresent: kv['enctl'] == '1',
    encAzDeg: double.tryParse(kv['encaz_deg'] ?? '') ?? 0,
    encTiltDeg: double.tryParse(kv['enctl_deg'] ?? '') ?? 0,
    inaPresent: kv['ina'] == '1',
    busV: double.tryParse(kv['vbus'] ?? '') ?? 0,
    currentA: double.tryParse(kv['cur'] ?? '') ?? 0,
    powerW: double.tryParse(kv['pwr'] ?? '') ?? 0,
  );
}

/// Pack readings from RigBatteryFrame.
class BatteryStatus {
  final double socPercent;
  final double packVoltage; // Volts
  final double currentMilliA;
  const BatteryStatus({
    required this.socPercent,
    required this.packVoltage,
    required this.currentMilliA,
  });
}

/// Camera sweep limits. Firmware derives speed from the span, cycle count,
/// and azimuth-pass duration, pausing tilt for each photograph.
const double defaultCamTiltFrom = -40.0;
const double defaultCamTiltTo = 40.0;
const int defaultCamTiltCycles = 8;

/// Converts a scan name into a safe filesystem name.
String _sanitizeFileName(String name) {
  final stripped = name.replaceAll(RegExp(r'[<>:"/\\|?*\x00-\x1F]'), '');
  return stripped.trim().replaceAll(RegExp(r'\s+'), '_');
}

/// Manages the rig connection, capture, telemetry, and session storage.
class ScanController extends ChangeNotifier {
  final RigLink _link;
  final Future<Directory> Function()? storageRoot;
  final Duration inactivityTimeout;
  ScanController({
    RigLink? link,
    this.storageRoot,
    this.inactivityTimeout = const Duration(seconds: 20),
  }) : _link = link ?? RigLink();
  StreamSubscription<RigEvent>? _eventSub;
  int _startupGeneration = 0;
  int _linkGeneration = 0;
  bool _disposed = false;
  bool _completed = false;
  Timer? _inactivityTimer;
  String? interruptionReason;
  String? saveError;
  String? firmwareVersion;

  bool get isBusy => {
    ScanPhase.preparing,
    ScanPhase.running,
    ScanPhase.stopping,
    ScanPhase.draining,
    ScanPhase.saving,
  }.contains(scanPhase);

  @override
  void notifyListeners() {
    if (!_disposed) super.notifyListeners();
  }

  void _watchInactivity() {
    _inactivityTimer?.cancel();
    if (scanPhase == ScanPhase.draining || scanPhase == ScanPhase.stopping) {
      _inactivityTimer = Timer(
        scanPhase == ScanPhase.stopping
            ? const Duration(seconds: 120)
            : inactivityTimeout,
        () => interrupt('Transfer or parking timed out'),
      );
    }
  }

  Future<void> interrupt(String reason) async {
    if (!isBusy || scanPhase == ScanPhase.saving) return;
    ++_startupGeneration;
    interruptionReason = reason;
    _link.send('STOP');
    _inactivityTimer?.cancel();
    scanPhase = ScanPhase.interrupted;
    _appendLog('EVT INTERRUPTED $reason');
    notifyListeners();
    if (points.isNotEmpty || shots.values.any((s) => s.jpeg != null)) {
      await _finishAndSave();
    }
  }

  LinkPhase linkPhase = LinkPhase.disconnected;
  BluetoothDevice? device;
  String? connectError;

  ScanPhase scanPhase = ScanPhase.idle;
  RigTelemetry telemetry = const RigTelemetry();
  BatteryStatus? battery;
  bool useCamera = true;

  /// Optional scan name for the folder and session.json; blank uses a timestamp.
  String? scanName;

  final List<PointRecord> points = [];
  final Map<int, ShotRecord> shots = {};
  int triggeredCount = 0;
  int drainedCount = 0;
  int drainExpectedCount = 0;
  int drainFailedCount = 0;
  int drainReceivedBytes = 0;
  Uint8List? latestImage;

  final Map<int, _ImageAssembly> _imageAssemblies = {};
  bool _saving = false;

  final List<String> log = [];
  Directory? lastSavedDir;

  TminiParser _lidarParser = TminiParser();

  void _appendLog(String line) {
    log.add(line);
    if (log.length > 300) log.removeAt(0);
  }

  Future<List<BluetoothDevice>> bondedDevices() =>
      FlutterBluetoothSerial.instance.getBondedDevices();

  Future<void> connect(BluetoothDevice d) async {
    if (_disposed ||
        linkPhase == LinkPhase.connecting ||
        linkPhase == LinkPhase.connected) {
      return;
    }
    final generation = ++_linkGeneration;
    linkPhase = LinkPhase.connecting;
    connectError = null;
    // Reset sensor readings when connecting to another device or session.
    battery = null;
    telemetry = const RigTelemetry();
    notifyListeners();
    try {
      await _link.connect(d);
      if (_disposed || generation != _linkGeneration) {
        await _link.disconnect();
        return;
      }
      device = d;
      linkPhase = LinkPhase.connected;
      await _eventSub?.cancel();
      _eventSub = _link.events.listen(_onEvent);
      // Check camera reachability asynchronously after connecting.
      unawaited(_pingCamera());
    } catch (e) {
      if (_disposed || generation != _linkGeneration) return;
      connectError = '$e';
      linkPhase = LinkPhase.disconnected;
    }
    notifyListeners();
  }

  Future<void> disconnect() async {
    ++_linkGeneration;
    await interrupt('Bluetooth disconnected');
    ++_startupGeneration;
    _cameraPingInFlight = null;
    await _eventSub?.cancel();
    _eventSub = null;
    await _link.disconnect();
    linkPhase = LinkPhase.disconnected;
    device = null;
    // Clear readings from the disconnected rig.
    battery = null;
    telemetry = const RigTelemetry();
    notifyListeners();
  }

  /// Whether the rig confirmed CAMTILT for this scan.
  bool cameraArmedThisScan = false;

  /// Camera preflight result for the current scan.
  CameraLinkStatus cameraLinkStatus = CameraLinkStatus.unknown;

  Future<({bool alive, String? reply})>? _cameraPingInFlight;

  /// Updates camera reachability, sharing any ping already in flight.
  Future<({bool alive, String? reply})> _pingCamera() {
    return _cameraPingInFlight ??= _doPingCamera().whenComplete(
      () => _cameraPingInFlight = null,
    );
  }

  Future<({bool alive, String? reply})> _doPingCamera() async {
    final generation = _linkGeneration;
    final reply = await _link.sendAndWait('CAMPING', [
      'OK CAMPING',
      'ERR CAMPING',
    ]);
    final alive =
        reply != null &&
        reply.toUpperCase().startsWith('OK CAMPING') &&
        reply.contains('protocol=2');
    if (!_disposed && generation == _linkGeneration) {
      cameraLinkStatus = alive
          ? CameraLinkStatus.ok
          : CameraLinkStatus.unreachable;
    }
    notifyListeners();
    return (alive: alive, reply: reply);
  }

  Future<void> startScan({String? name}) async {
    try {
      await _startScan(name: name);
    } catch (error) {
      if (!_disposed && scanPhase == ScanPhase.preparing) {
        ++_startupGeneration;
        interruptionReason = 'Startup failed: $error';
        scanPhase = ScanPhase.failed;
        _appendLog('ERR START_FAILED $error');
        try {
          _link.send('STOP');
        } catch (_) {
          /* Link may already be gone. */
        }
        notifyListeners();
      }
    }
  }

  Future<void> _startScan({String? name}) async {
    if (_disposed ||
        isBusy ||
        saveError != null ||
        linkPhase != LinkPhase.connected ||
        !_link.isConnected) {
      return;
    }
    final generation = ++_startupGeneration;
    bool active() =>
        !_disposed &&
        generation == _startupGeneration &&
        linkPhase == LinkPhase.connected &&
        _link.isConnected;
    _completed = false;
    interruptionReason = null;
    saveError = null;
    lastSavedDir = null;
    scanName = (name == null || name.trim().isEmpty) ? null : name.trim();
    points.clear();
    shots.clear();
    triggeredCount = 0;
    drainedCount = 0;
    drainExpectedCount = 0;
    drainFailedCount = 0;
    drainReceivedBytes = 0;
    latestImage = null;
    _imageAssemblies.clear();
    _saving = false;
    cameraArmedThisScan = false;
    scanPhase = ScanPhase.preparing;
    // Discard any partial packet from the previous scan.
    _lidarParser = TminiParser();
    notifyListeners();

    var wantCamera = useCamera;

    // Check camera reachability before arming capture; trigger events alone
    // do not confirm that photographs were saved.
    if (wantCamera) {
      final result = await _pingCamera();
      if (!active()) return;
      if (!result.alive) {
        wantCamera = false;
        _appendLog(
          result.reply == null
              ? 'ERR CAMPING no reply - cam module unreachable, starting lidar-only scan'
              : '${result.reply} - starting lidar-only scan',
        );
      }
    } else {
      cameraLinkStatus = CameraLinkStatus.unknown;
    }

    // Confirm the tilt sweep is armed before starting the scan.
    if (wantCamera) {
      final reply = await _link.sendAndWait(
        'CAMTILT $defaultCamTiltFrom $defaultCamTiltTo $defaultCamTiltCycles',
        ['OK CAMTILT', 'ERR'],
      );
      if (!active()) return;
      cameraArmedThisScan =
          reply != null && reply.toUpperCase().startsWith('OK CAMTILT');
      if (!cameraArmedThisScan) {
        _appendLog(
          reply == null
              ? 'ERR CAMTILT never confirmed - starting a lidar-only scan instead'
              : 'ERR CAMTILT rejected ($reply) - starting a lidar-only scan instead',
        );
        _link.send('CAMTILT OFF'); // best effort: make sure it is not left
      }
    } else {
      await _link.sendAndWait('CAMTILT OFF', ['OK CAMTILT']);
      if (!active()) return;
    }

    if (!active()) return;
    final reply = await _link.sendAndWait('SCAN', [
      'OK SCAN_STARTED',
      'ERR',
    ], retries: 0);
    if (!active()) return;
    var started = reply?.startsWith('OK SCAN_STARTED') ?? false;
    if (reply == null) {
      final status = await _link.sendAndWait('STATUS', [
        'OK STATUS',
      ], retries: 0);
      if (!active()) return;
      final state = parseKv(status ?? '')['state']?.toUpperCase();
      started = {
        'WARMING',
        'WARMING_UP',
        'SCANNING',
        'FORWARD',
        'RETURNING',
      }.contains(state);
    }
    scanPhase = started ? ScanPhase.running : ScanPhase.failed;
    if (!started) {
      interruptionReason = 'Scan startup was not confirmed';
      _link.send('STOP');
      _appendLog(
        'ERR START_FAILED ${reply ?? "no acknowledgement or active STATUS"}',
      );
    }
    notifyListeners();
  }

  /// Sends a raw command to the rig for the dashboard console.
  void sendRaw(String command) {
    _appendLog('> $command');
    _link.send(command);
  }

  void stopScan() {
    if (scanPhase == ScanPhase.preparing) {
      unawaited(interrupt('Stopped during preparation'));
      return;
    }
    if (scanPhase == ScanPhase.draining) {
      unawaited(interrupt('Photo transfer stopped'));
      return;
    }
    if (scanPhase != ScanPhase.running) return;
    ++_startupGeneration;
    interruptionReason = 'Stopped by user';
    scanPhase = ScanPhase.stopping;
    _link.send('STOP');
    _watchInactivity();
    notifyListeners();
  }

  void _onEvent(RigEvent ev) {
    if (_disposed) return;
    if (ev is RigImageChunk ||
        (ev is RigTextLine && ev.line.startsWith('EVT IMAGE_'))) {
      _watchInactivity();
    }
    switch (ev) {
      case RigTextLine(:final line):
        _onTextLine(line);
      case RigLidarFrame(:final thetaDeg, :final raw):
        _onLidarFrame(thetaDeg, raw);
      case RigImageChunk(
        :final idx,
        :final offset,
        :final totalLength,
        :final data,
      ):
        _onImageChunk(idx, offset, totalLength, data);
      case RigProtocolError(:final message):
        _appendLog('ERR PROTOCOL $message');
        notifyListeners();
      case RigBatteryFrame(
        :final socPercent,
        :final packVoltageV,
        :final currentMilliA,
      ):
        battery = BatteryStatus(
          socPercent: socPercent,
          packVoltage: packVoltageV,
          currentMilliA: currentMilliA,
        );
        notifyListeners();
    }
  }


  void _onTextLine(String line) {
    // Keep routine telemetry and photo details out of the console.
    if (!line.startsWith('EVT TELEM') &&
        !line.startsWith('EVT CAMTILT_AT') &&
        !line.startsWith('EVT CAM_ACK OK ') &&
        !line.startsWith('EVT IMAGE_META ')) {
      _appendLog(line);
    }

    // Update connection state when RigLink reports a lost connection.
    if (line == 'EVT WARN link closed' || line == 'EVT WARN link error') {
      linkPhase = LinkPhase.disconnected;
      unawaited(interrupt('Bluetooth connection lost'));
    }

    if (line.startsWith('EVT TELEM')) {
      telemetry = RigTelemetry.fromKv(parseKv(line));
    } else if (line.startsWith('EVT SHOT')) {
      final kv = parseKv(line);
      final idx = int.tryParse(kv['idx'] ?? '');
      if (idx == null) return;
      final cmdAz = double.tryParse(kv['cmd_az'] ?? '') ?? 0;
      final cmdTilt = double.tryParse(kv['cmd_tilt'] ?? '') ?? 0;
      final measAz = kv['encaz'] == '1'
          ? (double.tryParse(kv['encaz_deg'] ?? '') ?? cmdAz)
          : cmdAz;
      final measTilt = kv['enctl'] == '1'
          ? (double.tryParse(kv['enctl_deg'] ?? '') ?? cmdTilt)
          : cmdTilt;
      shots[idx] = ShotRecord(
        index: idx,
        cmdAz: cmdAz,
        cmdTilt: cmdTilt,
        measAz: measAz,
        measTilt: measTilt,
      );
      triggeredCount = triggeredCount < idx + 1 ? idx + 1 : triggeredCount;
    } else if (line.startsWith('EVT IMAGE_META')) {
      _onImageMeta(parseKv(line));
    } else if (line.startsWith('EVT IMAGE_END')) {
      _onImageEnd(parseKv(line));
    } else if (line.startsWith('EVT DONE')) {
      final kv = parseKv(line);
      firmwareVersion = kv['firmware'] ?? firmwareVersion;
      if (kv['outcome'] == 'partial') {
        interruptionReason ??=
            kv['reason'] ?? 'Firmware reported an interrupted capture';
      }
      _onScanDone();
    } else if (line.startsWith('OK DRAINCAM') ||
        line.startsWith('ERR DRAINCAM')) {
      final kv = parseKv(line);
      drainExpectedCount =
          int.tryParse(kv['expected'] ?? '') ?? drainExpectedCount;
      final rigFailed = int.tryParse(kv['failed'] ?? '');
      if (rigFailed != null && rigFailed > drainFailedCount) {
        drainFailedCount = rigFailed;
      }
      triggeredCount = int.tryParse(kv['triggered'] ?? '') ?? triggeredCount;
      if (scanPhase == ScanPhase.draining) {
        if (line.startsWith('ERR') || drainFailedCount > 0) {
          interruptionReason ??= 'Some photographs could not be transferred';
        }
        unawaited(_finishAndSave());
      }
    }
    notifyListeners();
  }

  void _onScanDone() {
    if (scanPhase != ScanPhase.running &&
        scanPhase != ScanPhase.stopping &&
        scanPhase != ScanPhase.preparing) {
      return;
    }
    // Invalidate an outstanding startup acknowledgement before publishing completion.
    ++_startupGeneration;
    if (cameraArmedThisScan) {
      scanPhase = ScanPhase.draining;
      _link.send('DRAINCAM');
      _watchInactivity();
    } else {
      _finishAndSave();
    }
  }

  void _onLidarFrame(double thetaDeg, Uint8List raw) {
    if (scanPhase != ScanPhase.running &&
        scanPhase != ScanPhase.stopping &&
        scanPhase != ScanPhase.preparing) {
      return;
    }
    for (final p in _lidarParser.feed(raw)) {
      final (x, y, z) = toCartesian(thetaDeg, p.angleDeg, p.distanceMm);
      if (x * x + y * y < 0.01) {
        continue; // matches scan_session.run_scan's own filter
      }
      points.add(PointRecord(x, y, z, p.intensity, thetaDeg));
    }
    // Screens sample points on a timer; avoid notifying on every LiDAR chunk.
  }

  void _onImageMeta(Map<String, String> kv) {
    final idx = int.tryParse(kv['idx'] ?? '');
    final total = int.tryParse(kv['len'] ?? '');
    final crc = int.tryParse(kv['crc32'] ?? '', radix: 16);
    if (idx == null ||
        total == null ||
        crc == null ||
        total <= 0 ||
        total > 262144) {
      _appendLog('ERR IMAGE_META invalid');
      return;
    }
    double number(String key) => double.tryParse(kv[key] ?? '') ?? 0;
    shots[idx] = ShotRecord(
      index: idx,
      cmdAz: number('cmd_az'),
      cmdTilt: number('cmd_tilt'),
      measAz: number('meas_az'),
      measTilt: number('meas_tilt'),
    );
    _imageAssemblies[idx] = _ImageAssembly(total, crc);
    drainExpectedCount++;
  }

  void _onImageChunk(int idx, int offset, int totalLength, Uint8List data) {
    final assembly = _imageAssemblies[idx];
    if (assembly == null || assembly.totalLength != totalLength) {
      _appendLog('ERR IMAGE_CHUNK idx=$idx has no matching metadata');
      return;
    }
    if (offset < assembly.nextOffset) return; // harmless retry/duplicate
    if (offset != assembly.nextOffset ||
        offset + data.length > assembly.totalLength) {
      _appendLog(
        'ERR IMAGE_CHUNK idx=$idx gap at ${assembly.nextOffset}, got $offset',
      );
      return;
    }
    assembly.bytes.setRange(offset, offset + data.length, data);
    assembly.nextOffset += data.length;
    drainReceivedBytes += data.length;
  }

  void _onImageEnd(Map<String, String> kv) {
    final idx = int.tryParse(kv['idx'] ?? '');
    if (idx == null) return;
    final assembly = _imageAssemblies.remove(idx);
    final shot = shots[idx];
    final senderOk = kv['ok'] == '1';
    final complete =
        assembly != null && assembly.nextOffset == assembly.totalLength;
    final jpeg = assembly?.bytes;
    final markersOk =
        jpeg != null &&
        jpeg.length >= 4 &&
        jpeg[0] == 0xFF &&
        jpeg[1] == 0xD8 &&
        jpeg[jpeg.length - 2] == 0xFF &&
        jpeg[jpeg.length - 1] == 0xD9;
    final crcOk =
        jpeg != null &&
        assembly != null &&
        crc32Bytes(jpeg) == assembly.expectedCrc;
    if (senderOk && complete && markersOk && crcOk && shot != null) {
      shot.jpeg = jpeg;
      latestImage = jpeg;
      drainedCount++;
    } else {
      drainFailedCount++;
      _appendLog(
        'ERR IMAGE_INVALID idx=$idx sender=$senderOk complete=$complete '
        'jpeg=$markersOk crc=$crcOk pose=${shot != null}',
      );
    }
  }

  Future<void> _finishAndSave() async {
    if (_saving || _completed || _disposed) return;
    _saving = true;
    _inactivityTimer?.cancel();
    scanPhase = ScanPhase.saving;
    saveError = null;
    notifyListeners();
    try {
      final root = storageRoot != null
          ? await storageRoot!()
          : await getExternalStorageDirectory();
      final base = root ?? await getApplicationDocumentsDirectory();
      final scansRoot = Directory('${base.path}/UnfoLDAR/scans');
      final safeName = scanName == null ? null : _sanitizeFileName(scanName!);
      String dirName;
      if (safeName != null && safeName.isNotEmpty) {
        // Preserve the chosen name; add a numeric suffix on a collision.
        dirName = safeName;
        var n = 2;
        while (await Directory('${scansRoot.path}/$dirName').exists()) {
          dirName = '$safeName ($n)';
          n++;
        }
      } else {
        final ts = DateTime.now().toIso8601String().replaceAll(
          RegExp(r'[:.]'),
          '-',
        );
        dirName = 'scan_$ts';
      }
      final dir = Directory('${scansRoot.path}/$dirName');
      await writeSession(
        sessionDir: dir,
        points: List.of(points),
        shots: shots.values.toList()
          ..sort((a, b) => a.index.compareTo(b.index)),
        camWidth: 640,
        camHeight: 480,
        triggeredCount: triggeredCount,
        capturedCount: drainExpectedCount,
        failedCount: drainFailedCount,
        name: scanName,
        captureOutcome: interruptionReason == null ? 'complete' : 'partial',
        interruptionReason: interruptionReason,
        firmwareVersion: firmwareVersion,
      );
      lastSavedDir = dir;
      _completed = true;
      scanPhase = ScanPhase.saved;
      _appendLog('EVT SAVED ${dir.path}');
    } catch (e) {
      saveError = '$e';
      scanPhase = ScanPhase.failed;
      _appendLog('ERR SAVE_FAILED $e');
    }
    _saving = false;
    notifyListeners();
  }

  Future<void> retrySave() async {
    if (scanPhase == ScanPhase.failed && saveError != null) {
      await _finishAndSave();
    }
  }

  void resetToIdle() {
    scanPhase = ScanPhase.idle;
    notifyListeners();
  }

  @override
  void dispose() {
    _disposed = true;
    ++_linkGeneration;
    ++_startupGeneration;
    _inactivityTimer?.cancel();
    _eventSub?.cancel();
    _link.disconnect();
    super.dispose();
  }
}
