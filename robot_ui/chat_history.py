#!/usr/bin/env python3
"""Shared chat history manager for persistent conversations across layouts"""
import json
import os

_HISTORY_FILE = os.path.join(os.path.dirname(__file__), '.chat_history.json')
_shared_history = None


def get_chat_history():
    """Get the shared chat history, loading from disk if needed"""
    global _shared_history
    if _shared_history is None:
        try:
            if os.path.exists(_HISTORY_FILE):
                with open(_HISTORY_FILE, 'r', encoding='utf-8') as f:
                    _shared_history = json.load(f)
            else:
                _shared_history = []
        except Exception:
            _shared_history = []
    return _shared_history


def save_chat_history(history):
    """Save chat history to disk and update shared state"""
    global _shared_history
    _shared_history = history
    try:
        with open(_HISTORY_FILE, 'w', encoding='utf-8') as f:
            json.dump(history, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[ChatHistory] Failed to save: {e}")


def clear_chat_history():
    """Clear the chat history"""
    global _shared_history
    _shared_history = []
    try:
        if os.path.exists(_HISTORY_FILE):
            os.remove(_HISTORY_FILE)
    except Exception:
        pass
