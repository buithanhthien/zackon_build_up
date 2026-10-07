"""Shared presentation for screens and dialogs launched from Startup."""
from startup_style import PALETTE

DIALOG_STYLESHEET = """
QMainWindow, QDialog, QMessageBox { background: %(bg)s; color: %(ink)s; }
QWidget { font-family: "Noto Sans", "DejaVu Sans", sans-serif; font-size: 14px; color: %(ink)s; }
QLabel { background: transparent; }
QLabel#title, QLabel#seq-label, QLabel#panel-title { font-size: 18px; font-weight: 700; padding: 4px 0; }
QLabel#header-title { font-size: 24px; font-weight: 700; }
QLabel#muted, QLabel#info-text, QLabel#section-label, QLabel#section-title, QLabel#log-title { color: %(muted)s; }
QLabel#wordmark { color: %(blue)s; font-size: 22px; font-weight: 700; padding: 20px; }
QLabel#clock { color: %(muted)s; background: white; border: 1px solid %(line)s; border-radius: 8px; padding: 8px; }
QPushButton { background: white; color: %(blue)s; border: 1px solid %(line)s; border-radius: 8px; min-height: 44px; padding: 0 12px; font-size: 14px; }
QPushButton:hover { background: %(blue_tint)s; border-color: %(blue)s; }
QPushButton:pressed { background: #dce7fc; }
QPushButton:focus { border: 2px solid %(blue)s; }
QPushButton:checked { background: %(blue)s; color: white; border-color: %(blue)s; }
QPushButton#primary-btn, QPushButton#ok-btn, QPushButton#apply-btn, QPushButton#dock-btn { background: %(yellow)s; color: %(blue)s; border-color: %(yellow)s; font-weight: 700; }
QPushButton#primary-btn:hover, QPushButton#ok-btn:hover, QPushButton#apply-btn:hover, QPushButton#dock-btn:hover { background: #ffe293; }
QPushButton#danger-btn, QPushButton#cancel-mapping-btn, QPushButton#interrupt-btn { background: %(red)s; color: white; border-color: %(red)s; font-weight: 700; }
QPushButton#danger-btn:hover, QPushButton#cancel-mapping-btn:hover, QPushButton#interrupt-btn:hover { background: #941a28; }
QPushButton:disabled, QPushButton#primary-btn:disabled, QPushButton#ok-btn:disabled,
QPushButton#apply-btn:disabled, QPushButton#dock-btn:disabled, QPushButton#danger-btn:disabled,
QPushButton#cancel-mapping-btn:disabled { background: #e8edf4; color: #63738a; border-color: %(line)s; }
QLineEdit, QListWidget, QTextEdit { background: white; color: %(ink)s; border: 1px solid %(line)s; border-radius: 8px; font-size: 15px; selection-background-color: %(blue)s; selection-color: white; }
QLineEdit { min-height: 44px; padding: 0 12px; }
QLineEdit:focus, QListWidget:focus, QTextEdit:focus { border: 2px solid %(blue)s; }
QTextEdit { padding: 8px; }
QListWidget { outline: none; }
QListWidget::item { min-height: 44px; padding: 4px 12px; border-bottom: 1px solid %(line)s; }
QListWidget::item:hover { background: %(blue_tint)s; }
QListWidget::item:selected { background: %(blue)s; color: white; border-left: 3px solid %(yellow)s; }
QCheckBox { min-height: 44px; spacing: 10px; }
QCheckBox::indicator { width: 22px; height: 22px; }
QProgressBar { background: white; border: 1px solid %(line)s; border-radius: 8px; min-height: 28px; text-align: center; }
QProgressBar::chunk { background: %(yellow)s; border-radius: 7px; }
QScrollBar:vertical { background: %(blue_tint)s; width: 10px; }
QScrollBar::handle:vertical { background: #a8b8cc; min-height: 28px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QWidget#left-panel { background: white; border-right: 1px solid %(line)s; }
QWidget#content-panel { background: %(bg)s; }
QWidget#status-card, QWidget#log-panel, QWidget#chat-panel { background: white; border: 1px solid %(line)s; border-radius: 12px; }
QLabel#status-light { font-size: 48px; }
QLabel#status-text { font-size: 16px; font-weight: 600; }
QPushButton#panel-tab:checked { background: %(blue)s; color: white; border-bottom: 3px solid %(yellow)s; }
QPushButton#keyboard-toggle { padding: 0; }
QPushButton#voice-btn { background: %(yellow)s; color: %(blue)s; font-weight: 700; }
QPushButton#voice-btn:checked { background: #ffe293; }
""" % PALETTE

MAIN_STYLESHEET = DIALOG_STYLESHEET
