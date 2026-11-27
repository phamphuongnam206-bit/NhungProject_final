# Kiểm tra thực hiện (2026-11-28)

Phạm vi: lộ trình ôm cua + né vật cản, tham số chỉnh lúc chạy qua UART/Bluetooth, sa bàn web.
Tái hiện toàn bộ: `python tools/run_host_tests.py` (hoặc `/kiem-tra` trong Claude Code).

- Build PlatformIO cho STM32F103: `test_static_route` và `test_static_route_bt` đều thành công. RAM 30,7% (6288 B), Flash 83,3% (53 716 B). Bản `-DSTATIC_ROUTE_NO_RANGE` cũng build được.
- Test mô phỏng trên PC (`test/static_route_host.cpp`, GCC 16.2 MinGW, `-Wall` không cảnh báo): 29 kịch bản × 2 tốc độ (400 và 700 CPS) = **58/58 đạt**.
- Bộ mô phỏng web (`tools/web/sim.js`) đối chiếu với C++ bằng `node tools/web/check_sim.js`: **14/14 đạt**, thời gian và quãng đường trùng khớp (36 370 ms / 23 220 ms).
- Codex soát lại code firmware (chỉ đọc) và nêu 8 vấn đề; cả 8 đã được kiểm chứng và sửa (xem cuối file).
- Mô phỏng là mô hình lý tưởng: bánh bám setpoint, cảm biến được dò tia tới hộp MAP. **Không** xác nhận đáp ứng PWM/ma sát, quãng trôi khi cắt động cơ, nhiễu cảm biến thật, vị trí lắp cảm biến, thời gian tính toán trên STM32, hay vị trí thật của xe. Chưa nạp ST-LINK, chưa chạy xe thật, chưa thử kết nối Web Serial với xe.

| Kịch bản | Kiểm tra | 400 | 700 |
|---|---|---|---|
| commands | `$GET/$SET/$DEFAULTS/$START/$STOP/$PING/$REBOOT`, từ chối sai định dạng/ngoài giới hạn/khi đang chạy, in số, đọc số | PASS | PASS |
| obstacle-map | bộ nhớ vật cản đầy → báo lỗi; ô cũ > 2 s được tái dùng; hộp cố định không chiếm chỗ | PASS | PASS |
| clearance | **thân xe** trên đường bo cung cách hộp cố định, vùng cấm, 4 ô ứng viên ≥ 15 mm (đo được 23,7 mm); công thức khe hở và độ cong đường né | PASS | PASS |
| route, wrap | đi hết lộ trình không dừng–xoay, không né oan, về GOAL ≤ 12 mm, chạy lại lần 2 giống lần 1 | PASS | PASS |
| random14..17 | mỗi ô ứng viên có vật: xe thấy vật, không né, khe thân xe ≥ 19,7 mm | PASS | PASS |
| block | vật lạ lấn một phần đường cuối (1510, 307): né mượt sang trái rồi về GOAL; thân xe cách vật 57,6 mm (400) / 44,2 mm (700) | PASS | PASS |
| block-center | vật lạ giữa hành lang N1→N2 (362, 652): không có khe → `NO_AVOID_PATH`, dừng cách vật > 230 mm | PASS | PASS |
| late | vật xuất hiện 230 mm trước mũi xe: dừng an toàn, không chạm (165 / 169 mm) | PASS | PASS |
| range-stale | mất dữ liệu cảm biến → `RANGE_STALE` | PASS | PASS |
| manual, stall-left/right, wrong-dir, mpu-read/stale/invalid/frozen, timeout, encoder-jump, init-bias, init-systick, uart | các lớp bảo vệ cũ giữ nguyên | PASS | PASS |
| miss, goal, coast | lệch khỏi đường > 100 mm → `OFF_PATH`; qua GOAL lệch 18 mm / trôi ra sau khi dừng → `MISSED_WAYPOINT` | PASS | PASS |

## Kết quả tuyến (mô phỏng)

| | Bản cũ dừng–xoay | Bản mới ôm cua |
|---|---|---|
| 400 CPS | 46,5 s, 2933 mm, sai số GOAL 4,9 mm | **36,4 s**, 2859 mm, sai số GOAL 4,6 mm, lệch ngang tối đa 9,3 mm, khe thân xe 23,7 mm |
| 700 CPS | 35,9 s, 2937 mm, sai số GOAL 4,6 mm | **23,2 s**, 2877 mm, sai số GOAL 4,7 mm, lệch ngang tối đa 9,2 mm, khe thân xe 19,7 mm |

Mô phỏng web với số xung/mm thật lệch 2% so với firmware: xe vẫn tự báo về đích, nhưng vị trí thật cách GOAL 21 mm. Hiệu chuẩn bánh xe là bước quan trọng nhất trước khi thi.

## 8 vấn đề Codex nêu và cách xử lý
1. Va chạm được kiểm tra như một điểm → kiểm tra hình chữ nhật thân xe xoay đúng hướng (`footprintGap`).
2. Hướng/độ cong đường né trên cung tròn xấp xỉ → công thức chính xác `A = 1 − k·d` (`shiftedCurv`, `shiftedHeading`).
3. Giữ kế hoạch né cũ khi ước lượng đổi → chỉ giữ nếu phần còn lại vẫn đủ khe với ước lượng mới, không thì dừng.
4. Xét lại kế hoạch theo vị trí cũ → cập nhật vị trí trước khi xét; kế hoạch đã vào dốc thì chốt.
5. Lập đường né nặng → lọc nhanh hộp ở xa, không lưu hộp cố định, mỗi 50 ms tối đa 1 lần lập; đo `#MAX_CONTROL_US` trên xe.
6. Dừng khẩn thả trôi → ngưỡng tăng theo tốc độ với vật lạ (cần đo quãng trôi thật).
7. Bộ nhớ vật cản đầy im lặng → tái dùng ô cũ, đầy thật thì `OBSTACLE_MAP_FULL`.
8. Mẫu cảm biến có thời điểm 0 bị coi là chưa có → cờ riêng `sampledBits`.
