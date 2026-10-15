import 'dart:io';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:archive/archive_io.dart';
import 'package:path_provider/path_provider.dart';
import 'package:share_plus/share_plus.dart';

import '../scan/session_writer.dart';
import 'point_cloud_fullscreen_screen.dart';
import 'point_preview.dart';

/// Saved scan metadata, loaded once per refresh.
class _ScanEntry {
  final Directory dir;
  final String name;
  final DateTime modified;
  const _ScanEntry({
    required this.dir,
    required this.name,
    required this.modified,
  });
}

enum _SortMode { dateNewest, dateOldest, nameAsc, nameDesc }

int _compareEntries(_ScanEntry a, _ScanEntry b, _SortMode mode) {
  switch (mode) {
    case _SortMode.dateNewest:
      return b.modified.compareTo(a.modified);
    case _SortMode.dateOldest:
      return a.modified.compareTo(b.modified);
    case _SortMode.nameAsc:
      return a.name.toLowerCase().compareTo(b.name.toLowerCase());
    case _SortMode.nameDesc:
      return b.name.toLowerCase().compareTo(a.name.toLowerCase());
  }
}

String _formatDate(DateTime d) {
  String two(int n) => n.toString().padLeft(2, '0');
  return '${d.year}-${two(d.month)}-${two(d.day)} ${two(d.hour)}:${two(d.minute)}';
}

/// Confirms deletion and attempts each directory independently.
Future<int> _confirmAndDeleteAll(
  BuildContext context,
  List<Directory> dirs,
) async {
  if (dirs.isEmpty) return 0;
  final single = dirs.length == 1
      ? dirs.first.path.split(Platform.pathSeparator).last
      : null;
  final confirmed = await showDialog<bool>(
    context: context,
    builder: (context) => AlertDialog(
      title: Text(
        single != null ? 'Delete this scan?' : 'Delete ${dirs.length} scans?',
      ),
      content: Text(
        single != null
            ? '"$single" and everything in it - cloud.ply, every photo, '
                  'session.json - will be permanently deleted from this '
                  'phone. This cannot be undone.'
            : 'All ${dirs.length} selected scans and everything in them - '
                  'cloud.ply, every photo, session.json - will be '
                  'permanently deleted from this phone. This cannot be '
                  'undone.',
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.of(context).pop(false),
          child: const Text('Cancel'),
        ),
        FilledButton.tonalIcon(
          style: FilledButton.styleFrom(
            backgroundColor: Theme.of(context).colorScheme.errorContainer,
            foregroundColor: Theme.of(context).colorScheme.onErrorContainer,
          ),
          onPressed: () => Navigator.of(context).pop(true),
          icon: const Icon(Icons.delete_forever),
          label: const Text('Delete forever'),
        ),
      ],
    ),
  );
  if (confirmed != true) return 0;

  var deleted = 0;
  var failed = 0;
  for (final dir in dirs) {
    try {
      await dir.delete(recursive: true);
      deleted++;
    } catch (_) {
      failed++;
    }
  }

  if (context.mounted) {
    final failedSuffix = failed > 0 ? ' · $failed failed' : '';
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Deleted $deleted scan(s)$failedSuffix')),
    );
  }
  return deleted;
}

/// Saved scans with preview, sorting, export and deletion.
class ScansScreen extends StatefulWidget {
  const ScansScreen({super.key});

  @override
  State<ScansScreen> createState() => _ScansScreenState();
}

class _ScansScreenState extends State<ScansScreen> {
  List<_ScanEntry> _entries = [];
  bool _loading = true;
  _SortMode _sortMode = _SortMode.dateNewest;

  // Track selection by path because reloading replaces entry objects.
  final Set<String> _selectedPaths = {};
  bool get _selecting => _selectedPaths.isNotEmpty;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    if (!mounted) return;
    setState(() => _loading = true);
    final entries = <_ScanEntry>[];
    try {
      final root = await getExternalStorageDirectory();
      final base = root ?? await getApplicationDocumentsDirectory();
      final scansDir = Directory('${base.path}/UnfoLDAR/scans');
      if (await scansDir.exists()) {
        final dirs = <Directory>[];
        await for (final entry in scansDir.list()) {
          if (entry is Directory &&
              !entry.path.contains('.partial-') &&
              await File('${entry.path}/session.json').exists() &&
              await File('${entry.path}/cloud.ply').exists()) {
            dirs.add(entry);
          }
        }
        // Load directory statistics once rather than on each list-item rebuild.
        for (final d in dirs) {
          entries.add(await _statsFor(d));
        }
      }
    } catch (_) {
      // leave entries empty - the screen just shows "no scans yet"
    }
    entries.sort((a, b) => _compareEntries(a, b, _sortMode));
    if (!mounted) return; // screen navigated away during the awaits above
    setState(() {
      _entries = entries;
      // Drop selected paths that no longer exist.
      _selectedPaths.retainAll(entries.map((e) => e.dir.path));
      _loading = false;
    });
  }

  Future<_ScanEntry> _statsFor(Directory d) async {
    // Use folder mtime; custom names need not contain a timestamp.
    DateTime modified;
    try {
      modified = (await d.stat()).modified;
    } catch (_) {
      modified = DateTime.fromMillisecondsSinceEpoch(0);
    }
    return _ScanEntry(
      dir: d,
      name: d.path.split(Platform.pathSeparator).last,
      modified: modified,
    );
  }

  void _setSortMode(_SortMode mode) {
    setState(() {
      _sortMode = mode;
      _entries.sort((a, b) => _compareEntries(a, b, mode));
    });
  }

  void _toggleSelected(_ScanEntry e) {
    setState(() {
      if (!_selectedPaths.remove(e.dir.path)) _selectedPaths.add(e.dir.path);
    });
  }

  Future<void> _deleteSelected() async {
    final dirs = _entries
        .where((e) => _selectedPaths.contains(e.dir.path))
        .map((e) => e.dir)
        .toList();
    _selectedPaths.clear();
    await _confirmAndDeleteAll(context, dirs);
    _load();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: _selecting ? _selectionAppBar() : _normalAppBar(),
      body: SafeArea(
        child: _loading
            ? const Center(child: CircularProgressIndicator())
            : _entries.isEmpty
            ? const Center(child: Text('No scans saved yet.'))
            : ListView.builder(
                itemCount: _entries.length,
                itemBuilder: (context, i) {
                  final e = _entries[i];
                  final selected = _selectedPaths.contains(e.dir.path);
                  return ListTile(
                    selected: selected,
                    leading: _selecting
                        ? Icon(
                            selected
                                ? Icons.check_circle
                                : Icons.radio_button_unchecked,
                            color: selected
                                ? Theme.of(context).colorScheme.primary
                                : null,
                          )
                        : const Icon(Icons.folder),
                    title: Text(e.name),
                    subtitle: Text(_formatDate(e.modified)),
                    trailing: _selecting
                        ? null
                        : Row(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              IconButton(
                                icon: const Icon(Icons.delete_outline),
                                tooltip: 'Delete scan',
                                onPressed: () async {
                                  await _confirmAndDeleteAll(context, [e.dir]);
                                  _load();
                                },
                              ),
                              const Icon(Icons.chevron_right),
                            ],
                          ),
                    // Long-press begins selection; taps toggle items while selecting.
                    onLongPress: () => _toggleSelected(e),
                    onTap: _selecting
                        ? () => _toggleSelected(e)
                        : () async {
                            // Reload after returning because the detail screen can delete a scan.
                            await Navigator.of(context).push(
                              MaterialPageRoute(
                                builder: (_) => _ScanDetailScreen(entry: e),
                              ),
                            );
                            _load();
                          },
                  );
                },
              ),
      ),
    );
  }

  AppBar _normalAppBar() {
    return AppBar(
      title: const Text('Saved Scans'),
      actions: [
        PopupMenuButton<_SortMode>(
          icon: const Icon(Icons.sort),
          tooltip: 'Sort',
          onSelected: _setSortMode,
          itemBuilder: (context) => [
            _sortMenuItem(_SortMode.dateNewest, 'Date (newest first)'),
            _sortMenuItem(_SortMode.dateOldest, 'Date (oldest first)'),
            _sortMenuItem(_SortMode.nameAsc, 'Name (A-Z)'),
            _sortMenuItem(_SortMode.nameDesc, 'Name (Z-A)'),
          ],
        ),
        IconButton(
          icon: const Icon(Icons.refresh),
          tooltip: 'Refresh',
          onPressed: _load,
        ),
      ],
    );
  }

  PopupMenuItem<_SortMode> _sortMenuItem(_SortMode mode, String label) {
    final active = mode == _sortMode;
    return PopupMenuItem(
      value: mode,
      child: Row(
        children: [
          SizedBox(
            width: 28,
            child: active ? const Icon(Icons.check, size: 18) : null,
          ),
          Text(label),
        ],
      ),
    );
  }

  AppBar _selectionAppBar() {
    return AppBar(
      leading: IconButton(
        icon: const Icon(Icons.close),
        tooltip: 'Cancel selection',
        onPressed: () => setState(_selectedPaths.clear),
      ),
      title: Text('${_selectedPaths.length} selected'),
      actions: [
        IconButton(
          icon: const Icon(Icons.select_all),
          tooltip: 'Select all',
          onPressed: () => setState(
            () => _selectedPaths.addAll(_entries.map((e) => e.dir.path)),
          ),
        ),
        IconButton(
          icon: const Icon(Icons.delete_forever),
          tooltip: 'Delete selected',
          onPressed: _deleteSelected,
        ),
      ],
    );
  }
}

/// Saved cloud and photo data for the detail screen. Either may be empty.
class _ScanContents {
  final List<PointRecord> points;
  final List<File> photos;
  final String? captureReason;
  const _ScanContents({
    required this.points,
    required this.photos,
    this.captureReason,
  });
}

class _ScanDetailScreen extends StatelessWidget {
  final _ScanEntry entry;
  const _ScanDetailScreen({required this.entry});

  Future<_ScanContents> _load() async {
    final cloudFile = File('${entry.dir.path}/cloud.ply');
    final points = await cloudFile.exists()
        ? await readPly(cloudFile)
        : <PointRecord>[];

    final photos = <File>[];
    await for (final item in entry.dir.list()) {
      if (item is File && item.path.toLowerCase().endsWith('.jpg')) {
        photos.add(item);
      }
    }
    photos.sort((a, b) => a.path.compareTo(b.path));

    final manifest =
        jsonDecode(await File('${entry.dir.path}/session.json').readAsString())
            as Map<String, dynamic>;
    final capture = manifest['capture'] as Map<String, dynamic>?;
    return _ScanContents(
      points: points,
      photos: photos,
      captureReason: capture?['outcome'] == 'partial'
          ? '${capture?['reason'] ?? "Capture interrupted"}'
          : null,
    );
  }

  Future<void> _shareScan(BuildContext context) async {
    try {
      final tmp = await getTemporaryDirectory();
      final zip = File('${tmp.path}/${entry.name}.zip');
      await ZipFileEncoder().zipDirectory(entry.dir, filename: zip.path);
      await SharePlus.instance.share(
        ShareParams(
          files: [XFile(zip.path, mimeType: 'application/zip')],
          subject: 'UnfoLDAR scan ${entry.name}',
        ),
      );
    } catch (e) {
      if (context.mounted) {
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(SnackBar(content: Text('Could not export scan: $e')));
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(entry.name),
        actions: [
          IconButton(
            icon: const Icon(Icons.share),
            tooltip: 'Export scan ZIP',
            onPressed: () => _shareScan(context),
          ),
          IconButton(
            icon: const Icon(Icons.delete_outline),
            tooltip: 'Delete scan',
            onPressed: () async {
              // Return to the list after deleting this scan.
              final deleted = await _confirmAndDeleteAll(context, [entry.dir]);
              if (deleted > 0 && context.mounted) {
                Navigator.of(context).pop();
              }
            },
          ),
        ],
      ),
      body: SafeArea(
        child: FutureBuilder<_ScanContents>(
          future: _load(),
          builder: (context, snapshot) {
            if (snapshot.hasError) {
              return Center(
                child: Text('Cannot read this capture: ${snapshot.error}'),
              );
            }
            if (!snapshot.hasData) {
              return const Center(child: CircularProgressIndicator());
            }
            final contents = snapshot.data!;
            if (contents.points.isEmpty && contents.photos.isEmpty) {
              return const Center(
                child: Text('Nothing was saved for this scan.'),
              );
            }
            // Keep the embedded cloud preview non-interactive so the page scrolls.
            // Open the full-screen view for cloud gestures.
            return CustomScrollView(
              slivers: [
                if (contents.captureReason != null)
                  SliverToBoxAdapter(
                    child: Padding(
                      padding: const EdgeInsets.all(12),
                      child: Text('Partial capture: ${contents.captureReason}'),
                    ),
                  ),
                if (contents.points.isNotEmpty) ...[
                  SliverPadding(
                    padding: const EdgeInsets.fromLTRB(12, 12, 12, 0),
                    sliver: SliverToBoxAdapter(
                      child: Text(
                        '3D point cloud (${contents.points.length} points)',
                        style: Theme.of(context).textTheme.titleMedium,
                      ),
                    ),
                  ),
                  SliverPadding(
                    padding: const EdgeInsets.fromLTRB(12, 8, 12, 12),
                    sliver: SliverToBoxAdapter(
                      child: _pointCloudPreview(context, contents.points),
                    ),
                  ),
                ],
                SliverPadding(
                  padding: const EdgeInsets.fromLTRB(12, 0, 12, 8),
                  sliver: SliverToBoxAdapter(
                    child: Text(
                      'Photos (${contents.photos.length})',
                      style: Theme.of(context).textTheme.titleMedium,
                    ),
                  ),
                ),
                if (contents.photos.isEmpty)
                  SliverPadding(
                    padding: const EdgeInsets.symmetric(horizontal: 12),
                    sliver: SliverToBoxAdapter(
                      child: Text(
                        'No valid photos were saved for this scan.',
                        style: TextStyle(
                          color: Theme.of(context).colorScheme.outline,
                        ),
                      ),
                    ),
                  )
                else
                  SliverPadding(
                    padding: const EdgeInsets.fromLTRB(12, 0, 12, 12),
                    sliver: SliverGrid(
                      gridDelegate:
                          const SliverGridDelegateWithFixedCrossAxisCount(
                            crossAxisCount: 2,
                            crossAxisSpacing: 8,
                            mainAxisSpacing: 8,
                            childAspectRatio: 4 / 3,
                          ),
                      delegate: SliverChildBuilderDelegate(
                        (context, i) => ClipRRect(
                          borderRadius: BorderRadius.circular(8),
                          child: Image.file(
                            contents.photos[i],
                            fit: BoxFit.cover,
                          ),
                        ),
                        childCount: contents.photos.length,
                      ),
                    ),
                  ),
              ],
            );
          },
        ),
      ),
    );
  }

  /// Non-interactive cloud preview with access to the full-screen view.
  Widget _pointCloudPreview(BuildContext context, List<PointRecord> points) {
    return GestureDetector(
      onTap: () => Navigator.of(context).push(
        MaterialPageRoute(
          builder: (_) => PointCloudFullscreenScreen(points: points),
        ),
      ),
      child: SizedBox(
        height: 320,
        child: Stack(
          children: [
            Positioned.fill(
              child: Container(
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.black26),
                  borderRadius: BorderRadius.circular(8),
                ),
                clipBehavior: Clip.antiAlias,
                child: AbsorbPointer(child: PointPreview(points: points)),
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
}
