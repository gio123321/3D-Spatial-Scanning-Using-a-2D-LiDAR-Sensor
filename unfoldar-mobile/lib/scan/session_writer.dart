import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

/// One PLY vertex: XYZ in metres, intensity, and commanded theta_deg.
class PointRecord {
  final double x, y, z;
  final int intensity;
  final double thetaDeg;
  const PointRecord(this.x, this.y, this.z, this.intensity, this.thetaDeg);
}

/// Camera pose metadata and the JPEG received during DRAINCAM.
/// Field names match SessionRecord/ShotRecord in desktop models.py.
class ShotRecord {
  final int index;
  final double cmdAz;
  final double cmdTilt;
  final double measAz;
  final double measTilt;
  Uint8List? jpeg; // filled in once DRAINCAM relays it back

  ShotRecord({
    required this.index,
    required this.cmdAz,
    required this.cmdTilt,
    required this.measAz,
    required this.measTilt,
  });

  String get fileName => 'shot_${index.toString().padLeft(3, '0')}.jpg';
}

/// Publishes cloud.ply, received shot_NNN.jpg files, and session.json.
/// Desktop io/scan.py reads this layout. Shots without JPEG bytes are omitted
/// from both the files and manifest. Failed writes retain recovery files.
Future<void> writeSession({
  required Directory sessionDir,
  required List<PointRecord> points,
  required List<ShotRecord> shots,
  required int camWidth,
  required int camHeight,
  int? triggeredCount,
  int? capturedCount,
  int? failedCount,
  String? name,
  String captureOutcome = 'complete',
  String? interruptionReason,
  String? firmwareVersion,
}) async {
  if (await sessionDir.exists()) {
    throw FileSystemException(
      'Refusing to overwrite a saved scan',
      sessionDir.path,
    );
  }
  final staging = Directory(
    '${sessionDir.path}.partial-${DateTime.now().microsecondsSinceEpoch}',
  );
  await staging.create(recursive: true);
  try {
    await _writeSessionContents(
      sessionDir: staging,
      points: points,
      shots: shots,
      camWidth: camWidth,
      camHeight: camHeight,
      triggeredCount: triggeredCount,
      capturedCount: capturedCount,
      failedCount: failedCount,
      name: name,
      captureOutcome: captureOutcome,
      interruptionReason: interruptionReason,
      firmwareVersion: firmwareVersion,
    );
    await staging.rename(sessionDir.path);
  } catch (error) {
    // Keep both collected data in memory and successfully written files for recovery.
    throw FileSystemException(
      'Save failed; recovery files at ${staging.path}: $error',
      staging.path,
    );
  }
}

Future<void> _writeSessionContents({
  required Directory sessionDir,
  required List<PointRecord> points,
  required List<ShotRecord> shots,
  required int camWidth,
  required int camHeight,
  int? triggeredCount,
  int? capturedCount,
  int? failedCount,
  String? name,
  required String captureOutcome,
  String? interruptionReason,
  String? firmwareVersion,
}) async {
  await _writePly(File('${sessionDir.path}/cloud.ply'), points);

  final writtenShots = <Map<String, dynamic>>[];
  for (final s in shots) {
    final jpeg = s.jpeg;
    if (jpeg == null) continue;
    await File(
      '${sessionDir.path}/${s.fileName}',
    ).writeAsBytes(jpeg, flush: true);
    writtenShots.add({
      'index': s.index,
      'file': s.fileName,
      'cmd_az': s.cmdAz,
      'cmd_tilt': s.cmdTilt,
      'meas_az': s.measAz,
      'meas_tilt': s.measTilt,
      'usable': true,
      'note': '',
    });
  }

  final manifest = {
    'version': 2,
    'mode': 'phone_bluetooth',
    'created': DateTime.now().toIso8601String(),
    'name': ?name,
    'capture': {'outcome': captureOutcome, 'reason': ?interruptionReason},
    'firmware_version': ?firmwareVersion,
    'camera': {
      'description': 'ESP32-CAM OV2640 (rig-mounted)',
      'width': camWidth,
      'height': camHeight,
      // The camera discards its buffered frame before exposure, so image and
      // pose indices match. Declare the offset explicitly for the desktop reader.
      'pose_index_offset': 0,
      'pose_timing': 'commanded_midpoint_az_tilt_stationary',
    },
    'transfer': {
      'triggered': triggeredCount ?? shots.length,
      'captured': capturedCount ?? shots.length,
      'received': writtenShots.length,
      'failed': failedCount ?? 0,
    },
    'shots': writtenShots,
  };
  await File(
    '${sessionDir.path}/session.json',
  ).writeAsString(jsonEncode(manifest), flush: true);
}

/// Reads the fixed binary PLY layout produced by this app's _writePly.
Future<List<PointRecord>> readPly(File file) async {
  final bytes = await file.readAsBytes();
  final headerEnd = _indexOfEndHeader(bytes);
  if (headerEnd < 0) throw const FormatException('Missing PLY header');

  const recordSize =
      17; // x,y,z float32 (4 each) + intensity u8 + theta_deg float32
  final body = ByteData.sublistView(bytes, headerEnd);
  final count = body.lengthInBytes ~/ recordSize;
  final header = ascii.decode(bytes.sublist(0, headerEnd));
  final declared = RegExp(r'element vertex (\d+)').firstMatch(header);
  if (!header.contains('format binary_little_endian 1.0') ||
      declared == null ||
      int.parse(declared.group(1)!) != count ||
      body.lengthInBytes % recordSize != 0) {
    throw const FormatException(
      'Unsupported or truncated UnfoLDAR point cloud',
    );
  }

  final points = <PointRecord>[];
  for (var i = 0; i < count; i++) {
    final off = i * recordSize;
    points.add(
      PointRecord(
        body.getFloat32(off, Endian.little),
        body.getFloat32(off + 4, Endian.little),
        body.getFloat32(off + 8, Endian.little),
        body.getUint8(off + 12),
        body.getFloat32(off + 13, Endian.little),
      ),
    );
  }
  return points;
}

int _indexOfEndHeader(Uint8List bytes) {
  final marker = ascii.encode('end_header\n');
  for (var i = 0; i + marker.length <= bytes.length; i++) {
    var match = true;
    for (var j = 0; j < marker.length; j++) {
      if (bytes[i + j] != marker[j]) {
        match = false;
        break;
      }
    }
    if (match) return i + marker.length;
  }
  return -1;
}

/// Binary little-endian XYZ float32, intensity uchar, theta_deg float32,
/// in that order. Desktop io/scan.py preserves these vertex properties.
Future<void> _writePly(File file, List<PointRecord> points) async {
  final header =
      'ply\n'
      'format binary_little_endian 1.0\n'
      'element vertex ${points.length}\n'
      'property float x\n'
      'property float y\n'
      'property float z\n'
      'property uchar intensity\n'
      'property float theta_deg\n'
      'end_header\n';

  final sink = file.openWrite();
  sink.add(ascii.encode(header));

  // Each block owns up to 4096 vertex records. IOSink queues its bytes,
  // so a block must not be reused while its asynchronous write is pending.
  try {
    for (var start = 0; start < points.length; start += 4096) {
      final end = (start + 4096).clamp(0, points.length);
      final block = ByteData((end - start) * 17);
      for (var i = start; i < end; i++) {
        final p = points[i];
        final offset = (i - start) * 17;
        block.setFloat32(offset, p.x, Endian.little);
        block.setFloat32(offset + 4, p.y, Endian.little);
        block.setFloat32(offset + 8, p.z, Endian.little);
        block.setUint8(offset + 12, p.intensity.clamp(0, 255));
        block.setFloat32(offset + 13, p.thetaDeg, Endian.little);
      }
      sink.add(block.buffer.asUint8List());
      await sink.flush(); // Bound queued buffers while saving large captures.
    }
  } finally {
    await sink.close();
  }
}
