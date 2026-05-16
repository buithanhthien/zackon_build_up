"""
VPS FastAPI server — middleman between robot and phone.

Install:  pip install fastapi uvicorn python-multipart
Run:      uvicorn vps_server:app --host 0.0.0.0 --port 8000
"""
import asyncio, json
from pathlib import Path
from fastapi import FastAPI, File, Form, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

app = FastAPI()
MAP_FILE = Path('/tmp/robot_map.png')
MAP_META: dict = {}
_phone_clients: set[WebSocket] = set()
_latest_pose: str | None = None


@app.post('/map')
async def receive_map(file: UploadFile = File(...), meta: str = Form(...)):
    global MAP_META
    MAP_FILE.write_bytes(await file.read())
    MAP_META = json.loads(meta)
    return {'status': 'ok'}


@app.get('/map')
async def serve_map():
    if not MAP_FILE.exists():
        return JSONResponse({'error': 'no map yet'}, status_code=404)
    return FileResponse(MAP_FILE, media_type='image/png')


@app.get('/map/meta')
async def serve_meta():
    return MAP_META


@app.websocket('/ws/pose')
async def robot_ws(ws: WebSocket):
    """Robot connects here to push pose."""
    await ws.accept()
    try:
        while True:
            data = await ws.receive_text()
            global _latest_pose
            _latest_pose = data
            dead = []
            for client in list(_phone_clients):
                try:
                    await client.send_text(data)
                except Exception:
                    dead.append(client)
            for d in dead:
                _phone_clients.discard(d)
    except WebSocketDisconnect:
        pass


@app.websocket('/ws/phone')
async def phone_ws(ws: WebSocket):
    """Phone connects here to receive pose."""
    await ws.accept()
    _phone_clients.add(ws)
    if _latest_pose:
        await ws.send_text(_latest_pose)
    try:
        while True:
            await asyncio.sleep(30)
    except WebSocketDisconnect:
        pass
    finally:
        _phone_clients.discard(ws)
