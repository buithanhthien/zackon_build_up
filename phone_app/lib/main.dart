import 'dart:convert';
import 'dart:math';
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';

// ── Config ────────────────────────────────────────────────────────────────────
const String kVPS = 'YOUR_VPS_IP:8000'; // ← change this

void main() => runApp(const ZackonApp());

class ZackonApp extends StatelessWidget {
  const ZackonApp({super.key});
  @override
  Widget build(BuildContext context) => MaterialApp(
        title: 'Zackon Map',
        theme: ThemeData(colorSchemeSeed: const Color(0xFF214196), useMaterial3: true),
        home: const MapScreen(),
      );
}

// ── Models ────────────────────────────────────────────────────────────────────
class MapMeta {
  final double resolution, originX, originY;
  final int width, height;
  const MapMeta(this.resolution, this.originX, this.originY, this.width, this.height);
  factory MapMeta.fromJson(Map<String, dynamic> j) => MapMeta(
        (j['resolution'] as num).toDouble(),
        (j['origin_x'] as num).toDouble(),
        (j['origin_y'] as num).toDouble(),
        j['width'] as int,
        j['height'] as int,
      );
}

class RobotPose {
  final double x, y, yaw;
  const RobotPose(this.x, this.y, this.yaw);
  factory RobotPose.fromJson(Map<String, dynamic> j) => RobotPose(
        (j['x'] as num).toDouble(),
        (j['y'] as num).toDouble(),
        (j['yaw'] as num).toDouble(),
      );
}

// ── Screen ────────────────────────────────────────────────────────────────────
class MapScreen extends StatefulWidget {
  const MapScreen({super.key});
  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  ui.Image? _mapImage;
  MapMeta? _meta;
  RobotPose? _pose;
  WebSocketChannel? _channel;
  String _status = 'Connecting…';

  @override
  void initState() {
    super.initState();
    _loadMap();
    _connectWs();
  }

  Future<void> _loadMap() async {
    setState(() => _status = 'Loading map…');
    try {
      final imgRes  = await http.get(Uri.parse('http://$kVPS/map'));
      final metaRes = await http.get(Uri.parse('http://$kVPS/map/meta'));
      if (imgRes.statusCode != 200) throw Exception('map not ready');

      final codec = await ui.instantiateImageCodec(imgRes.bodyBytes);
      final frame = await codec.getNextFrame();

      setState(() {
        _mapImage = frame.image;
        _meta = MapMeta.fromJson(jsonDecode(metaRes.body));
        _status = 'Map loaded';
      });
    } catch (e) {
      setState(() => _status = 'Map error: $e');
    }
  }

  void _connectWs() {
    _channel?.sink.close();
    _channel = WebSocketChannel.connect(Uri.parse('ws://$kVPS/ws/phone'));
    _channel!.stream.listen(
      (data) {
        final p = RobotPose.fromJson(jsonDecode(data as String));
        setState(() {
          _pose = p;
          _status = 'Live  x:${p.x.toStringAsFixed(2)}  y:${p.y.toStringAsFixed(2)}'
              '  yaw:${(p.yaw * 180 / pi).toStringAsFixed(1)}°';
        });
      },
      onError: (_) => Future.delayed(const Duration(seconds: 3), _connectWs),
      onDone:  ()  => Future.delayed(const Duration(seconds: 3), _connectWs),
    );
  }

  @override
  void dispose() {
    _channel?.sink.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFFF0F4FF),
      appBar: AppBar(
        backgroundColor: const Color(0xFF214196),
        foregroundColor: Colors.white,
        title: const Text('Zackon Robot Map'),
        actions: [
          IconButton(icon: const Icon(Icons.refresh), onPressed: _loadMap),
        ],
      ),
      body: Column(
        children: [
          // Status bar
          Container(
            width: double.infinity,
            color: const Color(0xFF214196).withOpacity(0.08),
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
            child: Text(_status,
                style: const TextStyle(fontSize: 13, color: Color(0xFF1a2a5e))),
          ),
          // Map canvas
          Expanded(
            child: _mapImage == null
                ? const Center(child: CircularProgressIndicator())
                : InteractiveViewer(
                    minScale: 0.5,
                    maxScale: 8.0,
                    child: CustomPaint(
                      painter: MapPainter(
                          mapImage: _mapImage!, meta: _meta, pose: _pose),
                      child: const SizedBox.expand(),
                    ),
                  ),
          ),
        ],
      ),
    );
  }
}

// ── Painter ───────────────────────────────────────────────────────────────────
class MapPainter extends CustomPainter {
  final ui.Image mapImage;
  final MapMeta? meta;
  final RobotPose? pose;

  const MapPainter({required this.mapImage, this.meta, this.pose});

  @override
  void paint(Canvas canvas, Size size) {
    // Draw map scaled to fill canvas
    final src = Rect.fromLTWH(
        0, 0, mapImage.width.toDouble(), mapImage.height.toDouble());
    final dst = Rect.fromLTWH(0, 0, size.width, size.height);
    canvas.drawImageRect(mapImage, src, dst, Paint());

    if (meta == null || pose == null) return;

    final scaleX = size.width  / meta!.width;
    final scaleY = size.height / meta!.height;

    // World → canvas (map is already flipped in bridge node)
    final cx = (pose!.x - meta!.originX) / meta!.resolution * scaleX;
    final cy = size.height - (pose!.y - meta!.originY) / meta!.resolution * scaleY;

    _drawArrow(canvas, cx, cy, pose!.yaw);
  }

  void _drawArrow(Canvas canvas, double cx, double cy, double yaw) {
    const len = 20.0;
    final tx = cx + len * cos(yaw);
    final ty = cy - len * sin(yaw); // canvas Y inverted

    // Body
    canvas.drawLine(Offset(cx, cy), Offset(tx, ty),
        Paint()..color = Colors.red..strokeWidth = 3);

    // Arrowhead
    const hl = 10.0, ha = 0.45;
    final path = Path()
      ..moveTo(tx, ty)
      ..lineTo(tx - hl * cos(yaw - ha), ty + hl * sin(yaw - ha))
      ..lineTo(tx - hl * cos(yaw + ha), ty + hl * sin(yaw + ha))
      ..close();
    canvas.drawPath(path, Paint()..color = Colors.red);

    // Robot dot
    canvas.drawCircle(
        Offset(cx, cy), 7, Paint()..color = Colors.red.withOpacity(0.35));
  }

  @override
  bool shouldRepaint(MapPainter old) =>
      old.pose?.x != pose?.x ||
      old.pose?.y != pose?.y ||
      old.pose?.yaw != pose?.yaw ||
      old.mapImage != mapImage;
}
