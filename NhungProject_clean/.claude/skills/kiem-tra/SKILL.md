---
name: kiem-tra
description: Kiểm tra toàn bộ dự án xe NhungProject - build firmware STM32 (bản UART và bản Bluetooth HC-05), chạy 58 kịch bản mô phỏng C++ (400 và 700 CPS) và đối chiếu bộ mô phỏng web với C++, rồi báo cáo bằng tiếng Việt. Dùng khi người dùng gõ /kiem-tra, hoặc sau khi sửa src/, include/, test/ hay tools/web/ để xác nhận không có gì hỏng.
---

# /kiem-tra: kiểm tra toàn bộ dự án xe

Chạy từ thư mục project (chứa `platformio.ini`). Trên Windows dùng Python và `pio` của PlatformIO:
`PY=%USERPROFILE%/.platformio/penv/Scripts/python.exe`, `PIO=%USERPROFILE%/.platformio/penv/Scripts/pio.exe`
(macOS/Linux: `~/.platformio/penv/bin/python`, `~/.platformio/penv/bin/pio`).

## Các bước (chạy song song bước 1 và 2 được)

1. **Build firmware** cho cả hai môi trường; ghi lại % RAM/Flash:
   - `PIO run -e test_static_route` (UART 115200)
   - `PIO run -e test_static_route_bt` (HC-05 38400)
   Flash trên 95% là cảnh báo (Blue Pill chỉ có 63 KB cho chương trình).
2. **Mô phỏng + đối chiếu web**: `PY tools/run_host_tests.py`
   - Tự tìm g++ (WinLibs) và Node.js; thiếu thì in lệnh `winget` để cài, hãy báo người dùng thay vì tự cài.
   - Đạt khi in `400 CPS: 29/29 PASS`, `700 CPS: 29/29 PASS`, `all checks passed` và `TONG KET: tat ca dat`.
3. Nếu có lỗi: đọc 8 dòng cuối của kịch bản hỏng mà script in ra, tìm nguyên nhân trong code, **không** sửa test cho
   qua. Kịch bản và ý nghĩa nằm trong `VERIFICATION.md`.

## Báo cáo (tiếng Việt, ngắn)
- Bảng: hạng mục | kết quả (build UART, build Bluetooth, 400 CPS, 700 CPS, web↔C++).
- Số liệu chính từ kịch bản `route`: thời gian, quãng đường, sai số đích, khe thân xe nhỏ nhất; so với
  `VERIFICATION.md` và nêu rõ nếu thay đổi.
- Lỗi nào còn lại và đề xuất sửa. Nhắc rằng mô phỏng là lý tưởng: chưa thay cho chạy thử xe thật.
