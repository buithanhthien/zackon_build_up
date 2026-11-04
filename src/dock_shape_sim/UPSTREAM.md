# Nguồn thuật toán

Nguồn cục bộ: `/home/khoa/zackon_docking/src/dock_shape_sim`.

Bản phần cứng được đồng bộ từ
`/home/khoaiuh/code_language/zackon_build_up_docking/src/dock_shape_sim`
vào `/home/khoaiuh/zackon_build_up/src/dock_shape_sim`, kèm
`robot_ui/docking_layout.py`, `robot_ui/docking_bridge.py` và các điểm tích hợp
docking trong `robot_ui/startup_layout.py`. Giữ map hiện tại và chức năng thang máy
của workspace đích.

Ngày đưa vào workspace phần cứng: 21/10/2026. Giữ giấy phép Apache-2.0 khai báo bởi package gốc.

Giữ nguyên perception/planning/tracker; adapter mission/controller được thêm guard phần cứng, theo dõi hủy và launch mới. Không sao chép GUI PyQt5, Gazebo, map mẫu hoặc package view_robot_pkg.

Các hash dưới đây là SHA256 của file nguồn gốc tại thời điểm sao chép:

- `control/__init__.py`: `aa0701c7c97bbd4d01d1657f3aaa590d341fa2b68e21a230ab13bea3094bbb67`
- `control/node.py`: `c2f4a00dd556e1bf0b1ba5ea54f3d14da4fa1f3e9cf573bd45feb507ac0d945d`
- `control/teleop.py`: `5f9af3879661db254721c57ca6baab89ecbf89d754477199f5e486bb4cca83a7`
- `control/tracker.py`: `98148dc6b0afaf2b662154b0e0c865f79e4c4f24e65f0f3c3136576539cbdc8d`
- `mission/__init__.py`: `cd2509c0136c097762e83b9836067f0030ca42b361c94bfb64eb547af3152cbd`
- `mission/node.py`: `9e1936ecedfe065e3ec8194fc75c46c50ed42c7c7188bc64aabff9cfccb8d145`
- `mission/profile.py`: `950c8d4c737691754a9f0ded666dcb1d5071d6046205d0ab343c134acdb3d0ec`
- `perception/__init__.py`: `5849d6f27f91675ec9f32ec4397beead62b85da78ae6dced12813d71ccb39e6d`
- `perception/v_shape.py`: `9428f33cfdb42a3d6fae7255463ef6e89f895257402078f064f1b29203942c38`
- `planning/__init__.py`: `f40d64ecab9e713d79553993491f75012c6c409fba3e65d38f0b0893f4770f6c`
- `planning/quintic.py`: `f3a29f2c400fabf032cb1c7379c5cc47cc4d8c9329c60eeb00c0ac96ee966694`
