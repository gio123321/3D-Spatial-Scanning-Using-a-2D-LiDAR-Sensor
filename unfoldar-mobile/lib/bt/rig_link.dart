import 'dart:async';
import 'dart:typed_data';

import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

/// A text line, LiDAR frame, image chunk, battery frame, or protocol error
/// decoded from Bluetooth SPP. Wire layouts are defined in UnfoLDAR/Config.h.
sealed class RigEvent {
  const RigEvent();
}

class RigTextLine extends RigEvent {
  final String line;
  const RigTextLine(this.line);
}

class RigLidarFrame extends RigEvent {
  final double thetaDeg;
  final Uint8List raw;
  const RigLidarFrame(this.thetaDeg, this.raw);
}

class RigImageChunk extends RigEvent {
  final int idx;
  final int offset;
  final int totalLength;
  final Uint8List data;
  const RigImageChunk(this.idx, this.offset, this.totalLength, this.data);
}

class RigProtocolError extends RigEvent {
  final String message;
  const RigProtocolError(this.message);
}

/// Pack SOC, voltage and current from the dedicated INA226 battery frame.
class RigBatteryFrame extends RigEvent {
  final double socPercent;
  final double packVoltageV;
  final double currentMilliA;
  const RigBatteryFrame(this.socPercent, this.packVoltageV, this.currentMilliA);
}

int crc32Bytes(Iterable<int> bytes, [int crc = 0]) {
  var value = (~crc) & 0xFFFFFFFF;
  for (final byte in bytes) {
    value ^= byte;
    for (var bit = 0; bit < 8; bit++) {
      value = (value >> 1) ^ ((value & 1) != 0 ? 0xEDB88320 : 0);
    }
  }
  return (~value) & 0xFFFFFFFF;
}

/// Reassembles partial SPP frames and newline-terminated EVT/OK/ERR lines.
/// Wire layouts and checksums are defined in UnfoLDAR/Config.h.
class RigFrameDecoder {
  static const _lidarSync0 = 0x5A, _lidarSync1 = 0xA5;
  static const _imgSync0 = 0x49, _imgSync1 = 0x43;
  static const _battSync0 = 0x42, _battSync1 = 0x53;
  static const _battFrameLen = 12;
  static const _maxImageLength = 262144;
  static const _maxChunkLength = 256;

  final List<int> _buf = [];

  Iterable<RigEvent> feed(Uint8List chunk) sync* {
    _buf.addAll(chunk);
    yield* _drain();
  }

  Iterable<RigEvent> _drain() sync* {
    while (true) {
      if (_buf.isEmpty) return;

      if (_buf[0] == _lidarSync0 &&
          _buf.length >= 2 &&
          _buf[1] == _lidarSync1) {
        if (_buf.length < 4) return; // wait for the length field
        final len = _buf[2] | (_buf[3] << 8);
        if (len < 5 || len > 100) {
          _buf.removeRange(0, 4);
          yield const RigProtocolError('invalid LiDAR payload length');
          continue;
        }
        final total = 4 + len + 1;
        if (_buf.length < total) return; // wait for the rest of the frame
        var checksum = 0;
        for (var i = 4; i < 4 + len; i++) {
          checksum ^= _buf[i];
        }
        if (checksum != _buf[4 + len]) {
          _buf.removeRange(0, total);
          yield const RigProtocolError('LiDAR checksum failed');
          continue;
        }
        final thetaMilli = _readInt32LE(4);
        final raw = Uint8List.fromList(_buf.sublist(8, 4 + len));
        yield RigLidarFrame(thetaMilli / 1000.0, raw);
        _buf.removeRange(0, total);
        continue;
      }

      if (_buf[0] == _imgSync0 && _buf.length >= 2 && _buf[1] == _imgSync1) {
        if (_buf.length < 14) return;
        final idx = _buf[2] | (_buf[3] << 8);
        final offset = _readUint32LE(4);
        final imageLength = _readUint32LE(8);
        final chunkLength = _buf[12] | (_buf[13] << 8);
        if (imageLength <= 0 ||
            imageLength > _maxImageLength ||
            chunkLength <= 0 ||
            chunkLength > _maxChunkLength ||
            offset < 0 ||
            offset + chunkLength > imageLength) {
          _buf.removeAt(0);
          yield const RigProtocolError('invalid image chunk header');
          continue;
        }
        final frameLength = 14 + chunkLength + 4;
        if (_buf.length < frameLength) return;
        final expectedCrc = _readUint32LE(14 + chunkLength);
        final actualCrc = crc32Bytes(_buf.sublist(2, 14 + chunkLength));
        if (actualCrc != expectedCrc) {
          _buf.removeRange(0, frameLength);
          yield RigProtocolError(
            'image chunk CRC failed for idx=$idx offset=$offset',
          );
          continue;
        }
        final data = Uint8List.fromList(_buf.sublist(14, 14 + chunkLength));
        yield RigImageChunk(idx, offset, imageLength, data);
        _buf.removeRange(0, frameLength);
        continue;
      }

      if (_buf[0] == _battSync0 && _buf.length >= 2 && _buf[1] == _battSync1) {
        if (_buf.length < _battFrameLen) {
          return; // wait for the rest of the frame
        }
        final socX10 = _buf[2] | (_buf[3] << 8);
        final voltageMv = _buf[4] | (_buf[5] << 8);
        final currentMa = (_buf[6] | (_buf[7] << 8)).toSigned(16);
        final expectedCrc = _readUint32LE(8);
        final actualCrc = crc32Bytes(_buf.sublist(2, 8));
        if (actualCrc != expectedCrc) {
          _buf.removeRange(0, _battFrameLen);
          yield const RigProtocolError('battery frame CRC failed');
          continue;
        }
        yield RigBatteryFrame(
          socX10 / 10.0,
          voltageMv / 1000.0,
          currentMa.toDouble(),
        );
        _buf.removeRange(0, _battFrameLen);
        continue;
      }

      // A corrupt header can leave junk before the next valid binary frame.
      // Resynchronize before treating that frame's bytes as an ASCII line.
      final nl = _buf.indexOf(0x0A);
      var sync = -1;
      for (var i = 1; i + 1 < _buf.length; i++) {
        if ((_buf[i] == _lidarSync0 && _buf[i + 1] == _lidarSync1) ||
            (_buf[i] == _imgSync0 && _buf[i + 1] == _imgSync1) ||
            (_buf[i] == _battSync0 && _buf[i + 1] == _battSync1)) {
          sync = i;
          break;
        }
      }
      if (sync >= 0 && (nl < 0 || sync < nl)) {
        _buf.removeRange(0, sync);
        continue;
      }
      if (nl < 0) {
        // Bound the buffer while waiting for a valid sync or newline.
        if (_buf.length > 4096) _buf.removeRange(0, _buf.length - 256);
        return;
      }
      final line = String.fromCharCodes(
        _buf.sublist(0, nl),
      ).replaceAll('\r', '').trim();
      _buf.removeRange(0, nl + 1);
      if (line.isNotEmpty) yield RigTextLine(line);
    }
  }

  int _readInt32LE(int off) {
    final v =
        _buf[off] |
        (_buf[off + 1] << 8) |
        (_buf[off + 2] << 16) |
        (_buf[off + 3] << 24);
    return v.toSigned(32);
  }

  int _readUint32LE(int off) {
    return _buf[off] |
        (_buf[off + 1] << 8) |
        (_buf[off + 2] << 16) |
        (_buf[off + 3] << 24);
  }
}

/// Bluetooth SPP connection feeding a RigFrameDecoder.
class RigLink {
  BluetoothConnection? _conn;
  StreamSubscription<Uint8List>? _sub;
  final _events = StreamController<RigEvent>.broadcast();
  RigFrameDecoder _decoder = RigFrameDecoder();

  Stream<RigEvent> get events => _events.stream;
  bool get isConnected => _conn?.isConnected ?? false;

  Future<void> connect(BluetoothDevice device) async {
    _decoder = RigFrameDecoder();
    final conn = await BluetoothConnection.toAddress(device.address);
    _conn = conn;
    _sub = conn.input?.listen(
      _onData,
      onDone: () => _events.add(const RigTextLine('EVT WARN link closed')),
      onError: (_) => _events.add(const RigTextLine('EVT WARN link error')),
    );
  }

  Future<void> disconnect() async {
    await _sub?.cancel();
    _sub = null;
    await _conn?.close();
    _conn = null;
    _decoder = RigFrameDecoder();
  }

  /// Sends a command; replies arrive through [events].
  void send(String command) {
    final conn = _conn;
    if (conn == null || !conn.isConnected) return;
    conn.output.add(Uint8List.fromList('$command\n'.codeUnits));
  }

  /// Sends [command] and waits for a matching reply.
  /// The reply listener is installed before sending.
  Future<String?> sendAndWait(
    String command,
    List<String> expectPrefixes, {
    Duration timeout = const Duration(seconds: 2),
    int retries = 2,
  }) {
    return waitForReply(
      events,
      () => send(command),
      expectPrefixes,
      timeout: timeout,
      retries: retries,
    );
  }

  void _onData(Uint8List chunk) {
    for (final ev in _decoder.feed(chunk)) {
      _events.add(ev);
    }
  }
}

/// Extracts key=value fields from an EVT/OK line.
Map<String, String> parseKv(String line) {
  final out = <String, String>{};
  for (final tok in line.split(' ')) {
    final eq = tok.indexOf('=');
    if (eq > 0) out[tok.substring(0, eq)] = tok.substring(eq + 1);
  }
  return out;
}

/// Waits for a matching text reply, with timeout and optional retries.
/// Subscribe before sending so immediate replies are not lost.
Future<String?> waitForReply(
  Stream<RigEvent> events,
  void Function() sendOnce,
  List<String> expectPrefixes, {
  Duration timeout = const Duration(seconds: 2),
  int retries = 2,
}) async {
  for (var attempt = 0; attempt <= retries; attempt++) {
    final completer = Completer<String?>();
    final sub = events.listen((ev) {
      if (ev is RigTextLine) {
        final upper = ev.line.toUpperCase();
        if (expectPrefixes.any((p) => upper.startsWith(p))) {
          if (!completer.isCompleted) completer.complete(ev.line);
        }
      }
    });
    String? result;
    try {
      sendOnce();
      result = await completer.future.timeout(timeout, onTimeout: () => null);
    } finally {
      await sub.cancel();
    }
    if (result != null) return result;
  }
  return null;
}
