"""Shared waypoint dialogs; callers own navigation and robot lifecycle."""
import json
import os
from PyQt6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QListWidget, QListWidgetItem, QTextEdit, QLineEdit, QMessageBox, QCheckBox)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont
from waypoint_store import normalize_waypoints, deletion_reason


class NewWaypointDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Tạo địa điểm mới")
        self.setModal(True)
        self.resize(400, 200)
        self.setStyleSheet("""
            QDialog { background-color: #f0f4ff; color: #1a2a5e; }
            QLabel { color: #5a7abf; font-size: 11px; letter-spacing: 2px; padding-bottom: 8px; }
            QLineEdit {
                background-color: #ffffff; color: #1a2a5e;
                border: 1px solid #c8d4f0; border-radius: 8px;
                font-size: 15px; padding: 10px 14px;
            }
            QLineEdit:focus { border: 1px solid #214196; }
            QPushButton#ok-btn {
                background-color: #214196; color: #ffffff;
                border: none; border-radius: 8px;
                font-size: 15px; min-height: 44px;
            }
            QPushButton#ok-btn:hover { background-color: #1a3278; }
            QPushButton#cancel-btn {
                background-color: transparent; color: #5a7abf;
                border: 1px solid #c8d4f0; border-radius: 8px;
                font-size: 15px; min-height: 44px;
            }
            QPushButton#cancel-btn:hover { background-color: #e8f0ff; color: #214196; border: 1px solid #214196; }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        label = QLabel("TÊN ĐỊA ĐIỂM")
        label.setFont(QFont("DM Sans", 11))
        layout.addWidget(label)

        self.name_input = QLineEdit()
        self.name_input.setFont(QFont("JetBrains Mono", 15))
        self.name_input.setPlaceholderText("Nhập tên...")
        self.name_input.returnPressed.connect(self.accept)
        layout.addWidget(self.name_input)
        self.deletable_checkbox = QCheckBox("Cho phép xóa")
        self.deletable_checkbox.setChecked(True)
        layout.addWidget(self.deletable_checkbox)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)
        btn_confirm = QPushButton("Xác nhận")
        btn_confirm.setObjectName("ok-btn")
        btn_confirm.setFont(QFont("JetBrains Mono", 15))
        btn_confirm.clicked.connect(self.accept)
        btn_back = QPushButton("Hủy")
        btn_back.setObjectName("cancel-btn")
        btn_back.setFont(QFont("JetBrains Mono", 15))
        btn_back.clicked.connect(self.reject)
        btn_layout.addWidget(btn_confirm)
        btn_layout.addWidget(btn_back)
        layout.addLayout(btn_layout)

    def get_name(self):
        return self.name_input.text().strip()

    def get_deletable(self):
        return self.deletable_checkbox.isChecked()


class WaypointPickerDialog(QDialog):
    def __init__(self, waypoints, current_map_name, parent=None, delete_callback=None):
        super().__init__(parent)
        self.setWindowTitle("Chọn địa điểm")
        self.setModal(True)
        self.resize(480, 560)
        self._selected_key = None
        self._waypoints = normalize_waypoints(waypoints)
        self._delete_callback = delete_callback
        self.setStyleSheet("""
            QDialog { background-color: #f0f4ff; color: #1a2a5e; }
            QLabel#title { color: #5a7abf; font-size: 11px; letter-spacing: 2px; padding-bottom: 8px; }
            QListWidget {
                background-color: #ffffff; color: #1a2a5e;
                border: 1px solid #c8d4f0; border-radius: 8px;
                font-size: 15px; outline: none;
            }
            QListWidget::item { padding: 12px 16px; border-bottom: 1px solid #e8f0ff; }
            QListWidget::item:hover { background-color: #e8f0ff; color: #214196; }
            QListWidget::item:selected { background-color: #214196; color: #ffffff; border-left: 3px solid #fcb525; }
            QPushButton#ok-btn {
                background-color: #214196; color: #ffffff;
                border: none; border-radius: 8px; font-size: 15px; min-height: 44px;
            }
            QPushButton#ok-btn:hover { background-color: #1a3278; }
            QPushButton#ok-btn:disabled { color: #a8bce8; background-color: #e0e8f8; }
            QPushButton#cancel-btn {
                background-color: transparent; color: #5a7abf;
                border: 1px solid #c8d4f0; border-radius: 8px; font-size: 15px; min-height: 44px;
            }
            QPushButton#cancel-btn:hover { background-color: #e8f0ff; color: #214196; border: 1px solid #214196; }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        title = QLabel("ĐỊA ĐIỂM")
        title.setObjectName("title")
        title.setFont(QFont("DM Sans", 11))
        layout.addWidget(title)

        self.list_widget = QListWidget()
        self.list_widget.setFont(QFont("JetBrains Mono", 15))
        for key, data in self._waypoints.items():
            if data.get('map_name') == current_map_name:
                item = QListWidgetItem(data['display_name'])
                item.setData(Qt.ItemDataRole.UserRole, key)
                self.list_widget.addItem(item)
        self.list_widget.currentItemChanged.connect(self._selection_changed)
        self.list_widget.itemDoubleClicked.connect(lambda item: self.accept())
        layout.addWidget(self.list_widget)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        self.btn_ok = QPushButton("Đi tới")
        self.btn_ok.setObjectName("ok-btn")
        self.btn_ok.setFont(QFont("JetBrains Mono", 15))
        self.btn_ok.setEnabled(False)
        self.btn_ok.clicked.connect(self.accept)
        self.btn_delete = QPushButton("Xóa điểm đến")
        self.btn_delete.setEnabled(False)
        self.btn_delete.clicked.connect(self._delete_selected)
        self.delete_hint = QLabel("Chọn một điểm đến để xem quyền xóa.")
        self.delete_hint.setWordWrap(True)
        layout.addWidget(self.delete_hint)
        btn_cancel = QPushButton("Hủy")
        btn_cancel.setObjectName("cancel-btn")
        btn_cancel.setFont(QFont("JetBrains Mono", 15))
        btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(self.btn_ok)
        btn_row.addWidget(self.btn_delete)
        btn_row.addWidget(btn_cancel)
        layout.addLayout(btn_row)
        self.btn_delete.setVisible(delete_callback is not None)
        self.delete_hint.setVisible(delete_callback is not None)

    def _selection_changed(self, item, previous=None):
        self._selected_key = item.data(Qt.ItemDataRole.UserRole) if item else None
        self.btn_ok.setEnabled(item is not None)
        reason = (deletion_reason(self._selected_key, self._waypoints[self._selected_key])
                  if item else 'Chọn một điểm đến để xem quyền xóa.')
        if not self._delete_callback and not reason:
            reason = 'Không có chức năng lưu thay đổi trong màn hình này.'
        self.btn_delete.setEnabled(item is not None and not reason)
        self.delete_hint.setText(reason)

    def _delete_selected(self):
        key = self._selected_key
        if (not key or not self._delete_callback
                or deletion_reason(key, self._waypoints[key])):
            return
        if self._delete_callback(key):
            row = self.list_widget.currentRow()
            self.list_widget.takeItem(row)
            del self._waypoints[key]
            self._selection_changed(self.list_widget.currentItem())

    def get_selected_key(self):
        return self._selected_key


DIALOG_STYLE = """
    QDialog { background-color: #f0f4ff; color: #1a2a5e; }
    QLabel#title { color: #5a7abf; font-size: 11px; letter-spacing: 2px; padding-bottom: 4px; }
    QLabel#seq-label { color: #5a7abf; font-size: 11px; letter-spacing: 2px; padding-bottom: 4px; }
    QListWidget {
        background-color: #ffffff; color: #1a2a5e;
        border: 1px solid #c8d4f0; border-radius: 8px; font-size: 15px; outline: none;
    }
    QListWidget::item { padding: 10px 16px; border-bottom: 1px solid #e8f0ff; }
    QListWidget::item:hover { background-color: #e8f0ff; color: #214196; }
    QListWidget::item:selected { background-color: #214196; color: #ffffff; border-left: 3px solid #fcb525; }
    QTextEdit { background-color: #ffffff; color: #1a2a5e; border: 1px solid #c8d4f0; border-radius: 8px; font-size: 14px; padding: 8px; }
    QLineEdit {
        background-color: #ffffff; color: #1a2a5e;
        border: 1px solid #c8d4f0; border-radius: 8px; font-size: 15px; padding: 10px 14px;
    }
    QLineEdit:focus { border: 1px solid #214196; }
    QPushButton#primary-btn {
        background-color: #214196; color: #ffffff;
        border: none; border-radius: 8px; font-size: 15px; min-height: 44px;
    }
    QPushButton#primary-btn:hover { background-color: #1a3278; }
    QPushButton#primary-btn:disabled { color: #a8bce8; background-color: #e0e8f8; }
    QPushButton#secondary-btn {
        background-color: transparent; color: #5a7abf;
        border: 1px solid #c8d4f0; border-radius: 8px; font-size: 15px; min-height: 44px;
    }
    QPushButton#secondary-btn:hover { background-color: #e8f0ff; color: #214196; border: 1px solid #214196; }
"""


class PathManagerDialog(QDialog):
    """Shows saved paths from multi_waypoints.json. Run path / New path / Back."""
    run_path_requested = pyqtSignal(list)   # emits sequence of keys

    def __init__(self, multi_wp_file, current_map, parent=None):
        super().__init__(parent)
        self.multi_wp_file = multi_wp_file
        self.current_map = current_map
        self.setWindowTitle("Tạo lộ trình")
        self.setModal(True)
        self.resize(480, 520)
        self.setStyleSheet(DIALOG_STYLE)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        title = QLabel("LỘ TRÌNH ĐÃ LƯU")
        title.setObjectName("title")
        title.setFont(QFont("DM Sans", 11))
        layout.addWidget(title)

        self.list_widget = QListWidget()
        self.list_widget.setFont(QFont("JetBrains Mono", 15))
        self._reload_list()
        layout.addWidget(self.list_widget)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        self.btn_run = QPushButton("Chạy lộ trình")
        self.btn_run.setObjectName("primary-btn")
        self.btn_run.setFont(QFont("JetBrains Mono", 15))
        self.btn_run.setEnabled(False)
        self.btn_run.clicked.connect(self._on_run)

        btn_new = QPushButton("Lộ trình mới")
        btn_new.setObjectName("primary-btn")
        btn_new.setFont(QFont("JetBrains Mono", 15))
        btn_new.clicked.connect(self._on_new)

        btn_remove = QPushButton("Xóa lộ trình")
        btn_remove.setObjectName("secondary-btn")
        btn_remove.setFont(QFont("JetBrains Mono", 15))
        btn_remove.clicked.connect(self._on_remove)

        btn_back = QPushButton("Quay lại")
        btn_back.setObjectName("secondary-btn")
        btn_back.setFont(QFont("JetBrains Mono", 15))
        btn_back.clicked.connect(self.reject)

        btn_row.addWidget(self.btn_run)
        btn_row.addWidget(btn_new)
        btn_row.addWidget(btn_remove)
        btn_row.addWidget(btn_back)
        layout.addLayout(btn_row)

        self.list_widget.itemClicked.connect(lambda: self.btn_run.setEnabled(True))

    def _reload_list(self):
        self.list_widget.clear()
        try:
            with open(self.multi_wp_file) as f:
                data = json.load(f)
            for name, info in data.items():
                if info.get('map_name') == self.current_map:
                    seq = ', '.join(info.get('sequence', []))
                    self.list_widget.addItem(QListWidgetItem(f"{name}  [{seq}]"))
        except Exception:
            pass

    def _on_run(self):
        item = self.list_widget.currentItem()
        if not item:
            return
        path_name = item.text().split('  [')[0]
        try:
            with open(self.multi_wp_file) as f:
                data = json.load(f)
            sequence = data[path_name]['sequence']
            self.run_path_requested.emit(sequence)
            self.accept()
        except Exception:
            pass

    def _on_new(self):
        self.done(2)   # custom code 2 = open NewPathDialog

    def _on_remove(self):
        item = self.list_widget.currentItem()
        if not item:
            return
        path_name = item.text().split('  [')[0]
        from PyQt6.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self, "Xác nhận xóa",
            f"Bạn có chắc muốn xóa lộ trình '{path_name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            with open(self.multi_wp_file) as f:
                data = json.load(f)
            if path_name in data:
                del data[path_name]
                with open(self.multi_wp_file, 'w') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                self._reload_list()
                self.btn_run.setEnabled(False)
        except Exception:
            pass


class NewPathDialog(QDialog):
    """Select goals in sequence, name the path, confirm to save."""

    def __init__(self, waypoints, current_map, multi_wp_file, parent=None):
        super().__init__(parent)
        self.waypoints = waypoints
        self.current_map = current_map
        self.multi_wp_file = multi_wp_file
        self.sequence = []
        self.setWindowTitle("Lộ trình mới")
        self.setModal(True)
        self.resize(700, 520)
        self.setStyleSheet(DIALOG_STYLE)
        self._build_ui()

    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(16)

        # ── Left: sequence preview ──
        left = QVBoxLayout()
        seq_label = QLabel("THỨ TỰ LỘ TRÌNH")
        seq_label.setObjectName("seq-label")
        seq_label.setFont(QFont("DM Sans", 11))
        left.addWidget(seq_label)

        self.seq_text = QTextEdit()
        self.seq_text.setReadOnly(True)
        self.seq_text.setFont(QFont("JetBrains Mono", 14))
        self.seq_text.setPlaceholderText("(chưa chọn)")
        left.addWidget(self.seq_text, 1)

        name_label = QLabel("TÊN LỘ TRÌNH")
        name_label.setObjectName("seq-label")
        name_label.setFont(QFont("DM Sans", 11))
        left.addWidget(name_label)

        self.name_input = QLineEdit()
        self.name_input.setFont(QFont("JetBrains Mono", 14))
        self.name_input.setPlaceholderText("Nhập tên...")
        left.addWidget(self.name_input)

        root.addLayout(left, 1)

        # ── Right: goal list + buttons ──
        right = QVBoxLayout()
        goal_label = QLabel("ĐỊA ĐIỂM")
        goal_label.setObjectName("title")
        goal_label.setFont(QFont("DM Sans", 11))
        right.addWidget(goal_label)

        self.goal_list = QListWidget()
        self.goal_list.setFont(QFont("JetBrains Mono", 14))
        for key, data in self.waypoints.items():
            if data.get('map_name') == self.current_map:
                item = QListWidgetItem(data.get('display_name', key))
                item.setData(Qt.ItemDataRole.UserRole, key)
                self.goal_list.addItem(item)
        self.goal_list.itemClicked.connect(self._add_goal)
        right.addWidget(self.goal_list, 1)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        btn_confirm = QPushButton("Xác nhận")
        btn_confirm.setObjectName("primary-btn")
        btn_confirm.setFont(QFont("JetBrains Mono", 14))
        btn_confirm.clicked.connect(self._confirm)

        btn_undo = QPushButton("Hoàn tác")
        btn_undo.setObjectName("secondary-btn")
        btn_undo.setFont(QFont("JetBrains Mono", 14))
        btn_undo.clicked.connect(self._undo)

        btn_back = QPushButton("Hủy")
        btn_back.setObjectName("secondary-btn")
        btn_back.setFont(QFont("JetBrains Mono", 14))
        btn_back.clicked.connect(self.reject)

        btn_row.addWidget(btn_confirm)
        btn_row.addWidget(btn_undo)
        btn_row.addWidget(btn_back)
        right.addLayout(btn_row)

        root.addLayout(right, 1)

    def _add_goal(self, item):
        self.sequence.append(item.data(Qt.ItemDataRole.UserRole))
        self._refresh_preview()

    def _undo(self):
        if self.sequence:
            self.sequence.pop()
            self._refresh_preview()

    def _refresh_preview(self):
        lines = [f"{i+1}. {k}" for i, k in enumerate(self.sequence)]
        self.seq_text.setPlainText('\n'.join(lines))

    def _confirm(self):
        name = self.name_input.text().strip()
        if not name:
            self.name_input.setPlaceholderText("⚠ Nhập tên trước!")
            return
        if not self.sequence:
            return

        data = {}
        if os.path.exists(self.multi_wp_file):
            try:
                with open(self.multi_wp_file, encoding='utf-8') as file:
                    data = json.load(file)
            except (OSError, json.JSONDecodeError) as exc:
                QMessageBox.warning(
                    self,
                    "Không thể lưu lộ trình",
                    f"Không đọc được dữ liệu lộ trình: {exc}",
                )
                return
            if not isinstance(data, dict):
                QMessageBox.warning(
                    self,
                    "Không thể lưu lộ trình",
                    "File lộ trình không đúng định dạng.",
                )
                return

        if name in data:
            answer = QMessageBox.question(
                self,
                "Lộ trình đã tồn tại",
                f"Bạn muốn thay thế lộ trình '{name}' không?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return

        data[name] = {'map_name': self.current_map, 'sequence': self.sequence}
        temp_path = self.multi_wp_file + '.tmp'
        try:
            with open(temp_path, 'w', encoding='utf-8') as file:
                json.dump(data, file, indent=2, ensure_ascii=False)
            os.replace(temp_path, self.multi_wp_file)
        except OSError as exc:
            QMessageBox.warning(
                self,
                "Không thể lưu lộ trình",
                f"Không ghi được file lộ trình: {exc}",
            )
            return
        self.accept()


class DestinationDialog(QDialog):
    """Only the four destination actions; no robot process ownership."""

    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Điểm đến")
        self.setStyleSheet(DIALOG_STYLE)
        self.resize(440, 340)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(12)
        for label, callback in (
            ("Tải bản đồ", parent.load_map),
            ("Địa điểm", parent.open_waypoint_picker),
            ("Tạo lộ trình", parent.open_path_manager),
            ("Tạo địa điểm mới", parent.open_new_waypoint_dialog),
        ):
            button = QPushButton(label)
            button.setObjectName("primary-btn")
            button.setFont(QFont("JetBrains Mono", 15))
            button.clicked.connect(callback)
            layout.addWidget(button)
        close_button = QPushButton("Đóng")
        close_button.setObjectName("secondary-btn")
        close_button.clicked.connect(self.reject)
        layout.addWidget(close_button)
