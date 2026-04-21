#!/usr/bin/env python3
"""Centralized stylesheet definitions for robot UI"""

MAIN_STYLESHEET = """
    QMainWindow, QWidget {
        background-color: #f0f4ff;
        color: #1a2a5e;
        border: none;
    }
    QWidget#left-panel {
        background-color: #214196;
        border-right: none;
    }
    QWidget#header-bar {
        background-color: transparent;
    }
    QWidget#status-card {
        background-color: #ffffff;
        border: 1px solid #c8d4f0;
        border-radius: 8px;
    }
    QWidget#log-panel {
        background-color: #ffffff;
        border-top: 1px solid #c8d4f0;
    }
    QWidget#chat-panel {
        background-color: #ffffff;
        border-top: 1px solid #c8d4f0;
    }
    QWidget#content-panel {
        background-color: #f0f4ff;
    }
    QPushButton#mode-btn, QPushButton#action-btn {
        background-color: transparent;
        color: #a8bce8;
        border: none;
        border-left: 4px solid transparent;
        border-radius: 0px;
        padding: 20px 20px 20px 24px;
        text-align: left;
        font-size: 18px;
    }
    QPushButton#mode-btn:hover, QPushButton#action-btn:hover {
        background-color: #1a3278;
        color: #ffffff;
        border-left: 4px solid #fcb525;
    }
    QPushButton#mode-btn:checked, QPushButton#action-btn:checked {
        background-color: #1a3278;
        color: #fcb525;
        border-left: 4px solid #fcb525;
    }
    QPushButton#mode-btn:disabled, QPushButton#action-btn:disabled {
        color: #4a6aaa;
        border-left: 4px solid transparent;
    }
    QPushButton#primary-btn {
        background-color: #214196;
        color: #ffffff;
        border: none;
        border-radius: 8px;
        font-size: 18px;
        min-height: 56px;
        padding: 0px 24px;
    }
    QPushButton#primary-btn:hover {
        background-color: #1a3278;
    }
    QPushButton#primary-btn:disabled {
        color: #a8bce8;
        background-color: #e0e8f8;
    }
    QPushButton#apply-btn {
        background-color: #fcb525;
        color: #1a2a5e;
        border: none;
        border-radius: 8px;
        font-size: 16px;
        min-height: 48px;
        padding: 0px 20px;
    }
    QPushButton#apply-btn:hover {
        background-color: #e8a510;
    }
    QPushButton#wp-btn {
        background-color: #ffffff;
        color: #5a7abf;
        border: 1px solid #c8d4f0;
        border-radius: 8px;
        font-size: 16px;
        min-height: 48px;
    }
    QPushButton#wp-btn:hover {
        background-color: #e8f0ff;
        color: #214196;
        border: 1px solid #214196;
    }
    QPushButton#wp-btn[filled="true"] {
        color: #22c55e;
        border: 2px solid #22c55e;
        background-color: #f0fff4;
    }
    QPushButton#wp-btn[selected="true"] {
        color: #ffffff;
        border: 2px solid #fcb525;
        background-color: #214196;
    }
    QTextEdit#log-text {
        background-color: #f8faff;
        color: #1a2a5e;
        border: none;
        font-size: 13px;
    }
    QLabel#log-title {
        color: #5a7abf;
        font-size: 11px;
        letter-spacing: 2px;
    }
    QLabel#clock {
        color: #5a7abf;
        font-size: 15px;
    }
    QLabel#mode-title, QLabel#header-title {
        color: #1a2a5e;
        font-size: 15px;
    }
    QLabel#device-name, QLabel#section-title, QLabel#section-label {
        color: #5a7abf;
        font-size: 11px;
        letter-spacing: 2px;
    }
    QLabel#section-title, QLabel#section-label {
        padding: 12px 24px 4px 24px;
    }
    QLabel#pos-value, QLabel#info-text {
        color: #5a7abf;
        font-size: 13px;
        padding: 0px 24px;
    }
    QLabel#status-ok {
        color: #22c55e;
        font-size: 14px;
    }
    QLabel#status-error {
        color: #ef4444;
        font-size: 14px;
    }
    QLabel#status-checking {
        color: #5a7abf;
        font-size: 14px;
    }
    QPushButton#panel-tab {
        background-color: transparent;
        color: #5a7abf;
        border: none;
        border-bottom: 2px solid transparent;
        border-radius: 0px;
        padding: 6px 16px;
        font-size: 11px;
        letter-spacing: 2px;
    }
    QPushButton#panel-tab:checked {
        color: #214196;
    }
    QPushButton#panel-tab:hover {
        color: #214196;
    }
    QLineEdit#chat-input, QLineEdit#map-input {
        background-color: #f0f4ff;
        color: #1a2a5e;
        border: 1px solid #c8d4f0;
        border-radius: 6px;
        padding: 10px 14px;
        font-size: 13px;
        selection-background-color: #fcb52544;
    }
    QLineEdit#chat-input:focus, QLineEdit#map-input:focus {
        border: 1px solid #214196;
    }
    QLineEdit#map-input {
        background-color: #ffffff;
        font-size: 16px;
        min-height: 48px;
        padding: 0px 12px;
    }
    QPushButton#send-btn {
        background-color: #214196;
        color: #ffffff;
        border: none;
        border-radius: 6px;
        padding: 10px 20px;
        font-size: 13px;
    }
    QPushButton#send-btn:hover {
        background-color: #1a3278;
    }
    QPushButton#send-btn:pressed {
        background-color: #fcb525;
        color: #214196;
    }
    QPushButton#send-btn:disabled {
        color: #a8bce8;
        background-color: #e0e8f8;
        border: none;
    }
    QPushButton#voice-btn {
        background-color: #f0f4ff;
        color: #5a7abf;
        border: 1px solid #c8d4f0;
        border-radius: 6px;
        font-size: 26px;
        padding: 10px;
    }
    QPushButton#voice-btn:checked {
        background-color: #fff0f0;
        color: #ef4444;
        border: 1px solid #ef444466;
    }
    QPushButton#clear-btn {
        background-color: transparent;
        color: #a8bce8;
        border: none;
        font-size: 11px;
        padding: 4px 8px;
    }
    QPushButton#clear-btn:hover {
        color: #5a7abf;
    }
    QScrollArea {
        background-color: #ffffff;
        border: none;
    }
    QScrollBar:vertical {
        background-color: #f0f4ff;
        width: 6px;
        border-radius: 3px;
    }
    QScrollBar::handle:vertical {
        background-color: #c8d4f0;
        border-radius: 3px;
        min-height: 20px;
    }
    QScrollBar::handle:vertical:hover {
        background-color: #214196;
    }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
        height: 0px;
    }
"""

DIALOG_STYLESHEET = """
    QDialog {
        background-color: #f0f4ff;
        color: #1a2a5e;
    }
    QLabel#title, QLabel#seq-label {
        color: #5a7abf;
        font-size: 11px;
        letter-spacing: 2px;
        padding-bottom: 4px;
    }
    QListWidget {
        background-color: #ffffff;
        color: #1a2a5e;
        border: 1px solid #c8d4f0;
        border-radius: 8px;
        font-size: 15px;
        outline: none;
    }
    QListWidget::item {
        padding: 10px 16px;
        border-bottom: 1px solid #e8f0ff;
    }
    QListWidget::item:hover {
        background-color: #e8f0ff;
        color: #214196;
    }
    QListWidget::item:selected {
        background-color: #214196;
        color: #ffffff;
        border-left: 3px solid #fcb525;
    }
    QTextEdit {
        background-color: #ffffff;
        color: #1a2a5e;
        border: 1px solid #c8d4f0;
        border-radius: 8px;
        font-size: 14px;
        padding: 8px;
    }
    QLineEdit {
        background-color: #ffffff;
        color: #1a2a5e;
        border: 1px solid #c8d4f0;
        border-radius: 8px;
        font-size: 15px;
        padding: 10px 14px;
    }
    QLineEdit:focus {
        border: 1px solid #214196;
    }
    QPushButton#primary-btn, QPushButton#ok-btn {
        background-color: #214196;
        color: #ffffff;
        border: none;
        border-radius: 8px;
        font-size: 15px;
        min-height: 44px;
    }
    QPushButton#primary-btn:hover, QPushButton#ok-btn:hover {
        background-color: #1a3278;
    }
    QPushButton#primary-btn:disabled, QPushButton#ok-btn:disabled {
        color: #a8bce8;
        background-color: #e0e8f8;
    }
    QPushButton#secondary-btn, QPushButton#cancel-btn {
        background-color: transparent;
        color: #5a7abf;
        border: 1px solid #c8d4f0;
        border-radius: 8px;
        font-size: 15px;
        min-height: 44px;
    }
    QPushButton#secondary-btn:hover, QPushButton#cancel-btn:hover {
        background-color: #e8f0ff;
        color: #214196;
        border: 1px solid #214196;
    }
"""
