"""Shared microphone state, independent of button presentation."""


def microphone_state(panel):
    state = getattr(panel, "_voice_state", "")
    if "SPEAKING" in state:
        return "speaking"
    if getattr(panel, "_recording_active", False):
        return "listening"
    timer = getattr(panel, "_typing_timer", None)
    if timer is not None and timer.isActive():
        return "thinking"
    engine = getattr(panel, "_voice_engine", None)
    capture_lock = getattr(engine, "_listen_lock", None)
    if capture_lock is not None and capture_lock.locked():
        return "processing"
    threads = (getattr(panel, name, None) for name in ("_ai_thread", "_intent_thread"))
    if ("THINKING" in state
            or any(thread is not None and thread.isRunning() for thread in threads)):
        return "processing"
    return "idle"


def refresh_microphone_enabled(panel):
    panel.voice_btn.setEnabled(microphone_state(panel) in ("idle", "listening"))
