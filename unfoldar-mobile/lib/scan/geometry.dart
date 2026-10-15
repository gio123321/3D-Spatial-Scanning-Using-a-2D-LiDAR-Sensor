import 'dart:math' as math;
import 'dart:typed_data';

/// One decoded lidar sample, before it becomes a 3D point.
class LidarPoint {
  final double angleDeg;
  final int distanceMm;
  final int intensity;
  const LidarPoint(this.angleDeg, this.distanceMm, this.intensity);
}

/// Incremental YDLIDAR T-mini Plus parser. Packets contain AA 55, CT,
/// LSN, start/end angles, an XOR checksum, and LSN three-byte samples.
/// Preserves partial packets between feed() calls.
class TminiParser {
  static const _header0 = 0xAA, _header1 = 0x55;
  final List<int> _buf = [];
  int badChecksums = 0;

  Iterable<LidarPoint> feed(Uint8List data) sync* {
    _buf.addAll(data);
    yield* _parse();
  }

  Iterable<LidarPoint> _parse() sync* {
    while (true) {
      final h = _findHeader();
      if (h < 0) {
        if (_buf.length > 1) {
          _buf.removeRange(
            0,
            _buf.length - 1,
          ); // keep a possible split header byte
        }
        return;
      }
      if (h > 0) _buf.removeRange(0, h);
      if (_buf.length < 10) return;

      final ct = _buf[2], lsn = _buf[3];
      if (lsn == 0 || lsn > 200) {
        _buf.removeRange(0, 2); // bogus header, resync
        continue;
      }
      final total = 10 + 3 * lsn;
      if (_buf.length < total) return; // wait for the rest

      final pkt = Uint8List.fromList(_buf.sublist(0, total));
      _buf.removeRange(0, total);
      yield* _decode(pkt, ct, lsn);
    }
  }

  int _findHeader() {
    for (var i = 0; i + 1 < _buf.length; i++) {
      if (_buf[i] == _header0 && _buf[i + 1] == _header1) return i;
    }
    return -1;
  }

  Iterable<LidarPoint> _decode(Uint8List pkt, int ct, int lsn) sync* {
    final fsa = pkt[4] | (pkt[5] << 8);
    final lsa = pkt[6] | (pkt[7] << 8);
    final cs = pkt[8] | (pkt[9] << 8);

    if (!_checksumOk(pkt, lsn, ct, fsa, lsa, cs)) {
      badChecksums++;
      return;
    }

    final angFsa = (fsa >> 1) / 64.0;
    final angLsa = (lsa >> 1) / 64.0;
    final diff = (angLsa - angFsa) % 360.0;
    final denom = lsn > 1 ? lsn - 1 : 1;

    for (var k = 0; k < lsn; k++) {
      final off = 10 + 3 * k;
      final b1 = pkt[off], b2 = pkt[off + 1], b3 = pkt[off + 2];
      final dist = (b3 << 6) + (b2 >> 2);
      final flag = b2 & 0x03;
      if (dist == 0) continue;
      if (flag == 2 || flag == 3) continue; // interference, dropped
      final ang = (angFsa + diff * k / denom) % 360.0;
      yield LidarPoint(ang, dist, b1);
    }
  }

  static bool _checksumOk(
    Uint8List pkt,
    int lsn,
    int ct,
    int fsa,
    int lsa,
    int cs,
  ) {
    var x = 0x55AA ^ ((lsn << 8) | ct) ^ fsa ^ lsa;
    for (var k = 0; k < lsn; k++) {
      final off = 10 + 3 * k;
      x ^= pkt[off]; // 0x00 . Si[1]
      x ^= (pkt[off + 2] << 8) | pkt[off + 1]; // Si[3] . Si[2]
    }
    return (x & 0xFFFF) == cs;
  }
}

// LiDAR mounting convention established from captured scans. These signs
// define the saved PLY frame and must stay consistent with desktop camera poses.
const double lidarMountAlphaDeg = 90.0;
const int lidarAlphaDir = -1;
const int panDir = -1;

/// Converts commanded azimuth and LiDAR angle/range to PLY XYZ in metres.
/// The LiDAR rotates with azimuth; only the camera tilts.
(double, double, double) toCartesian(
  double thetaDeg,
  double angleDeg,
  int distanceMm,
) {
  final r = distanceMm / 1000.0;
  final a = (lidarAlphaDir * angleDeg + lidarMountAlphaDeg) * math.pi / 180.0;
  final t = (panDir * thetaDeg) * math.pi / 180.0;
  final radial = r * math.sin(a);
  final z = -r * math.cos(a);
  final x = radial * math.cos(t);
  final y = radial * math.sin(t);
  return (x, y, z);
}
