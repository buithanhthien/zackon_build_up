# Rà soát bộ nhớ Bé Sơn — 2026-10-12

Branch: `fix/beson-hindsight-memory-persistence`.

## Kết quả thực tế trong workspace

| Bước | Kết quả | Giới hạn |
| --- | --- | --- |
| P0: dạy và hỏi trong UI | Chưa thực hiện với LLM thật | Không ghi nhận phản hồi UI thực tế; không coi test mock là đã lưu |
| P1: cấu hình | `.env` đặt `HINDSIGHT_ENABLED=false`; không có override Hindsight trong process kiểm tra | Cần kiểm tra riêng môi trường launcher trên máy robot |
| P1: API / bank | Cấu hình được UI đọc cho ra `http://127.0.0.1:8888`, `beson-corrections` | Không chứng minh container dùng cùng cấu hình |
| P1: dịch vụ | Không có lệnh `docker`; TCP 8888 trả `ConnectionRefusedError` ngoài sandbox | Không đọc được health/log/container config |
| P2: document / restart | Chưa kiểm chứng | Không có dịch vụ để kiểm tra bản ghi, thời điểm, volume thực tế |
| P3: code | Sửa truy vấn recall, phục hồi metadata, kiểm tra memory khi local thiếu dữ liệu | Cần đánh giá recall/LLM thật với tiếng Việt |
| P4: test | `python3 -m unittest discover -s tests -q`: 202 test, 14 skip, không lỗi; `git diff --check` sạch | Memory/HTTP/LLM được mock, không thay thế bài Docker acceptance |

Không sửa `.env`, không xóa volume, không ghi dữ liệu thử vào bank thật và không
đưa API key vào báo cáo. Khi bộ nhớ tắt, code chỉ xác nhận thông tin trong phiên;
việc đóng panel làm mất các thông tin đó là hành vi hiện tại, không phải bằng
chứng Hindsight mất dữ liệu đã lưu.

## Lỗi đã khoanh vùng trong code

1. Trước đây recall nhận cả câu hỏi và lịch sử/session snapshots. API 0.4.9 từ
   chối trên 500 token; khi recall thất bại, ứng dụng không thực hiện retain và
   chỉ xác nhận trong phiên. Nay truyền câu query đã giải quyết ngữ cảnh từ
   planner, giữ lịch sử đầy đủ riêng trong prompt phân loại.
2. Serializer recall 0.4.9 không điền metadata. Bộ lọc JSON cần `json_path` và
   `base_json_value`, nên bản ghi từ phiên mới có thể bị loại dù recall tìm thấy.
   Nay lấy `retain_params.metadata` qua danh sách documents, so khớp chính xác
   ID, hỗ trợ phân trang/JSON string và dùng lại kết quả trong cùng lần recall.
   Giữ nguyên kiểm tra phạm vi, giá trị gốc và provenance.
3. Route local trước đây đi thẳng ra web nếu JSON không đủ. Nay hỏi memory trước
   khi fallback; các yêu cầu web rõ ràng và tác vụ general vẫn giữ route riêng.

Đối chiếu nguồn phiên bản được pin:

- [HTTP API: giới hạn recall, serializer và documents](https://github.com/vectorize-io/hindsight/blob/v0.4.9/hindsight-api/hindsight_api/api/http.py)
- [Danh sách documents trả retain_params](https://github.com/vectorize-io/hindsight/blob/v0.4.9/hindsight-api/hindsight_api/engine/memory_engine.py)
- [Retain lưu metadata vào retain_params](https://github.com/vectorize-io/hindsight/blob/v0.4.9/hindsight-api/hindsight_api/engine/retain/orchestrator.py)

## Phần cần chạy trên máy robot

Khởi động Hindsight theo `HINDSIGHT_DOCKER_SETUP.md`, bật
`HINDSIGHT_ENABLED=true`, khởi động lại UI rồi dạy lại thông tin. Ghi nguyên văn
xác nhận và câu trả lời; kiểm tra document và recall trong đúng bank trước/sau
restart bình thường. Chạy cả dữ liệu giả lập không thuộc IUH và đính chính một
trường IUH có scope. Dùng bảng ghi kết quả trong tài liệu setup. Chưa thể kết
luận lỗi đã hết trên hệ thống thật cho tới khi các bước này đạt.
