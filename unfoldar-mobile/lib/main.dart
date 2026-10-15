import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'scan/scan_controller.dart';
import 'ui/dashboard_screen.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  // Lock orientation before the first frame.
  await SystemChrome.setPreferredOrientations(const [
    DeviceOrientation.portraitUp,
  ]);
  await SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
  runApp(UnfoldarApp(controller: ScanController()));
}

class UnfoldarApp extends StatelessWidget {
  final ScanController controller;
  const UnfoldarApp({super.key, required this.controller});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'UnfoLDAR',
      theme: ThemeData(
        colorSchemeSeed: Colors.teal,
        useMaterial3: true,
        brightness: Brightness.light,
      ),
      themeMode: ThemeMode.light,
      home: DashboardScreen(controller: controller),
    );
  }
}
