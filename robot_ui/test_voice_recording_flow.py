import ast
import contextlib
from pathlib import Path
from types import SimpleNamespace
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from motion_commands import parse_motion, validate_motion


ROOT = Path(__file__).resolve().parent
CHAT_PANEL = ROOT / "chat_panel_widget.py"
VOICE_ENGINE = ROOT / "voice_engine.py"


def _extract_method(path, class_name, method_name, extra_globals=None):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    class_node = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    method = next(
        node for node in class_node.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == method_name
    )
    method.decorator_list = []
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {}
    if extra_globals:
        namespace.update(extra_globals)
    exec(compile(module, str(path), "exec"), namespace)
    return namespace[method_name]


class DummyButton:
    def __init__(self):
        self.enabled = True
        self.checked = False
        self.text = ""

    def setEnabled(self, value):
        self.enabled = value

    def setChecked(self, value):
        self.checked = value

    def setText(self, value):
        self.text = value


class DummyLabel:
    def __init__(self):
        self.visible = False
        self.text = ""
        self.style = ""

    def show(self):
        self.visible = True

    def hide(self):
        self.visible = False

    def setText(self, value):
        self.text = value

    def setStyleSheet(self, value):
        self.style = value


class DummyVoiceEngine:
    def __init__(self):
        self.listen_calls = 0
        self.stop_listen_calls = 0
        self.stop_speaking_calls = 0
        self.listen_result = True

    def listen_once(self):
        self.listen_calls += 1
        return self.listen_result

    def stop_listening(self):
        self.stop_listen_calls += 1
        return True

    def stop_speaking(self):
        self.stop_speaking_calls += 1


class DummySignal:
    def __init__(self):
        self.values = []

    def emit(self, value):
        self.values.append(value)


def test_mic_button_first_click_starts_second_click_stops_recording():
    click = _extract_method(CHAT_PANEL, "ChatPanel", "_on_listen_btn_clicked")
    reset = _extract_method(CHAT_PANEL, "ChatPanel", "_reset_recording_controls")

    panel = SimpleNamespace(
        _recording_active=False,
        _voice_enabled=False,
        _voice_engine=DummyVoiceEngine(),
        voice_btn=DummyButton(),
        voice_status_label=DummyLabel(),
        _reset_recording_controls=lambda: reset(panel),
    )

    click(panel)

    assert panel._voice_engine.listen_calls == 1
    assert panel._recording_active is True
    assert panel.voice_btn.checked is True
    assert "DỪNG" in panel.voice_btn.text

    click(panel)

    assert panel._voice_engine.stop_listen_calls == 1
    assert panel._voice_engine.listen_calls == 1
    assert panel._recording_active is False
    assert panel.voice_btn.enabled is False
    assert "XỬ LÝ" in panel.voice_btn.text


def test_second_mic_click_does_not_use_stop_speaking():
    click = _extract_method(CHAT_PANEL, "ChatPanel", "_on_listen_btn_clicked")
    reset = _extract_method(CHAT_PANEL, "ChatPanel", "_reset_recording_controls")

    engine = DummyVoiceEngine()
    panel = SimpleNamespace(
        _recording_active=False,
        _voice_enabled=False,
        _voice_engine=engine,
        voice_btn=DummyButton(),
        voice_status_label=DummyLabel(),
        _reset_recording_controls=lambda: reset(panel),
    )

    click(panel)
    click(panel)

    assert engine.stop_speaking_calls == 0
    assert engine.stop_listen_calls == 1


def test_empty_transcript_is_not_sent_and_nonempty_is_sent_once():
    handler = _extract_method(
        CHAT_PANEL,
        "ChatPanel",
        "_on_voice_transcript",
        extra_globals={"_normalize_room_names": lambda text: text.lower(),
                       "parse_motion": parse_motion, "validate_motion": validate_motion},
    )

    asked = []
    panel = SimpleNamespace(
        _language="vi",
        _waypoints_provider=None,
        _voice_engine=DummyVoiceEngine(),
        navigation_stop=SimpleNamespace(emit=lambda: None),
        log_signal=SimpleNamespace(emit=lambda _msg: None),
        _answer_current_location=lambda: None,
        _classify_intent=lambda *_args: None,
        _ask_ai=lambda text: asked.append(text),
    )

    handler(panel, "   ")
    assert asked == []

    handler(panel, "Xin chao")
    assert asked == ["Xin chao"]


def test_interrupt_button_remains_bound_to_stop_speaking():
    source = CHAT_PANEL.read_text(encoding="utf-8")
    assert "self.interrupt_btn.clicked.connect(self._voice_engine.stop_speaking)" in source


class FakeEvent:
    def __init__(self):
        self.value = False

    def is_set(self):
        return self.value

    def set(self):
        self.value = True

    def clear(self):
        self.value = False


class FakeAudioData:
    def __init__(self, frame_data, sample_rate, sample_width):
        self.frame_data = frame_data
        self.sample_rate = sample_rate
        self.sample_width = sample_width


class FakeStream:
    def __init__(self, stop_event, stop_after_reads=3):
        self.stop_event = stop_event
        self.stop_after_reads = stop_after_reads
        self.reads = 0

    def read(self, _chunk_size):
        self.reads += 1
        if self.reads >= self.stop_after_reads:
            self.stop_event.set()
        # Loud 16-bit PCM, enough to cross the test energy threshold.
        return (1000).to_bytes(2, "little", signed=True) * 8


class FakeSource:
    CHUNK = 8
    SAMPLE_RATE = 16000
    SAMPLE_WIDTH = 2

    def __init__(self, stream):
        self.stream = stream


def test_early_stop_returns_the_audio_recorded_so_far():
    capture = _extract_method(
        VOICE_ENGINE,
        "VoiceEngine",
        "_capture_audio_until_stopped",
        extra_globals={
            "time": __import__("time"),
            "_suppress_stderr": contextlib.nullcontext,
            "sr": SimpleNamespace(AudioData=FakeAudioData),
        },
    )
    rms = _extract_method(VOICE_ENGINE, "VoiceEngine", "_pcm16_rms")

    stop_event = FakeEvent()
    stream = FakeStream(stop_event, stop_after_reads=3)
    engine = SimpleNamespace(
        _listen_stop_event=stop_event,
        recognizer=SimpleNamespace(energy_threshold=100),
        _pcm16_rms=rms,
    )

    audio = capture(engine, FakeSource(stream), timeout=5.0, phrase_time_limit=15.0)

    assert audio is not None
    assert stream.reads == 3
    assert len(audio.frame_data) == 3 * 16
    assert audio.sample_rate == 16000
    assert audio.sample_width == 2


def test_stop_listening_sets_only_recording_event():
    stop = _extract_method(VOICE_ENGINE, "VoiceEngine", "stop_listening")

    class Locked:
        def locked(self):
            return True

    event = FakeEvent()
    engine = SimpleNamespace(_listen_lock=Locked(), _listen_stop_event=event)

    assert stop(engine) is True
    assert event.is_set() is True


def test_listen_thread_has_single_transcript_emission_point():
    tree = ast.parse(VOICE_ENGINE.read_text(encoding="utf-8"))
    class_node = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "VoiceEngine"
    )
    method = next(
        node for node in class_node.body
        if isinstance(node, ast.FunctionDef) and node.name == "_listen_thread"
    )

    count = 0
    for node in ast.walk(method):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Attribute) or func.attr != "emit":
            continue
        owner = func.value
        if (
            isinstance(owner, ast.Attribute)
            and isinstance(owner.value, ast.Name)
            and owner.value.id == "self"
            and owner.attr == "transcript_ready"
        ):
            count += 1

    assert count == 1


def test_voice_session_cleanup_resets_event_lock_and_state():
    finish = _extract_method(
        VOICE_ENGINE,
        "VoiceEngine",
        "_finish_listening_session",
    )

    class FakeLock:
        def __init__(self):
            self.released = False

        def locked(self):
            return not self.released

        def release(self):
            self.released = True

    event = FakeEvent()
    event.set()
    lock = FakeLock()
    state = DummySignal()
    engine = SimpleNamespace(
        _listen_stop_event=event,
        _listen_lock=lock,
        state_changed=state,
    )

    finish(engine)

    assert event.is_set() is False
    assert lock.released is True
    assert state.values == [""]


def test_ui_end_state_resets_recording_button():
    state_changed = _extract_method(
        CHAT_PANEL,
        "ChatPanel",
        "_on_voice_state_changed",
    )
    reset = _extract_method(CHAT_PANEL, "ChatPanel", "_reset_recording_controls")

    button = DummyButton()
    button.setChecked(True)
    button.setEnabled(False)
    button.setText("ĐANG XỬ LÝ")
    label = DummyLabel()

    panel = SimpleNamespace(
        _pending_reply=None,
        _did_speak=False,
        _recording_active=True,
        _voice_enabled=True,
        voice_btn=button,
        voice_status_label=label,
        _ai_thread=None,
        _intent_thread=None,
        _reset_recording_controls=lambda: reset(panel),
        _finish_turn=lambda: None,
    )

    state_changed(panel, "")

    assert panel._recording_active is False
    assert panel._voice_enabled is False
    assert button.enabled is True
    assert button.checked is False
    assert button.text == "CLICK\n TO SPEAK"
    assert label.visible is False
