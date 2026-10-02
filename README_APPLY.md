# Apply the IUH local-search + correction-overlay update

Target repository: `/home/khoaiuh/zackon_build_up`

1. Back up the current files:

```bash
cd /home/khoaiuh/zackon_build_up
cp robot_ui/chat_panel_widget.py robot_ui/chat_panel_widget.py.bak
cp robot_ui/correction_memory.py robot_ui/correction_memory.py.bak
cp tests/test_correction_memory.py tests/test_correction_memory.py.bak
```

2. Copy these bundle files into the repository, preserving paths:

- `robot_ui/chat_panel_widget.py`
- `robot_ui/correction_memory.py`
- `robot_ui/iuh_local_search.py` (new)
- `tests/test_correction_memory.py`
- `tests/test_correction_scope.py` (new)
- `tests/test_iuh_local_search.py` (new)
- `HINDSIGHT_DOCKER_SETUP.md`

Do **not** replace `robot_ui/iuh_database.json`. It remains your repository-managed IUH data source.

3. Run the narrow regression suite:

```bash
cd /home/khoaiuh/zackon_build_up
python3 -m unittest -v \
  tests/test_iuh_local_search.py \
  tests/test_correction_scope.py \
  tests/test_correction_memory.py
```

Expected result for this bundle: `Ran 37 tests ... OK`.

4. Start the UI normally. No changes are required in `docker-compose.hindsight.yml`, `config.py`, or `start_beson.sh`.

5. Manual acceptance checks:

- Ask: `Email phòng đào tạo là gì?` -> local JSON, no web.
- Ask: `IUH có bao nhiêu sinh viên?` -> local is insufficient, web fallback.
- Ask: `Địa chỉ là gì?` -> clarification, no guess.
- Ask: `Tin mới nhất của IUH là gì?` -> web route.
- Teach a correction for the training-office email, then ask again -> correction wins only for `lien_he.phong_dao_tao`.
- Ask for the FEET email -> the training-office correction must not affect it.
- Stop Hindsight and ask a local JSON question -> local answer must still work.
- Edit and save `robot_ui/iuh_database.json` while the UI is running, then ask the affected fact again -> the local retriever reloads the saved JSON on the next search.
