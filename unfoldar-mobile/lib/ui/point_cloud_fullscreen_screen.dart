import 'dart:async';

import 'package:flutter/material.dart';

import '../scan/session_writer.dart';
import 'point_preview.dart';

/// Full-screen cloud view with orbit, pan and zoom gestures.
class PointCloudFullscreenScreen extends StatefulWidget {
  final List<PointRecord> points;

  /// Repaint periodically while the point list is growing.
  final bool live;

  const PointCloudFullscreenScreen({
    super.key,
    required this.points,
    this.live = false,
  });

  @override
  State<PointCloudFullscreenScreen> createState() =>
      _PointCloudFullscreenScreenState();
}

class _PointCloudFullscreenScreenState
    extends State<PointCloudFullscreenScreen> {
  Timer? _timer;

  @override
  void initState() {
    super.initState();
    if (widget.live) {
      // The caller appends to the same list without emitting change events.
      _timer = Timer.periodic(const Duration(milliseconds: 150), (_) {
        if (mounted) setState(() {});
      });
    }
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        backgroundColor: Colors.black,
        foregroundColor: Colors.white,
        title: Text('${widget.points.length} points'),
      ),
      body: PointPreview(points: widget.points),
    );
  }
}
