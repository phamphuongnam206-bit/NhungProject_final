# Việc tiếp theo

## Thử trên xe thật (theo thứ tự)
1. Đặt xe trên sàn (không nâng bánh), nạp `test_static_route`.
2. Hiệu chỉnh bánh bằng `$CAL` (web → Hiệu chỉnh → Đẩy thẳng 1000 mm).
3. Hiệu chỉnh gyro (web → Hiệu chỉnh → Xoay tại chỗ 2 vòng → Áp dụng).
4. Đặt xe ở START trên sa bàn và xem `#PLACE`. Nếu luôn lệch đều 10–20 mm thì gửi `$SET,START_FIX,0`.
5. Chạy thử, lưu log và xem `#SYNC`, `#MAX_SYNC_MM`, `#POSE_FIX`, `#GOAL_ERROR_MM`.

## Đã biết
- Bánh trái có vùng chết lớn ở PWM thấp (khoảng PWM 20 mới quay). Có thể cần "mồi" PWM khi khởi động.
- Nâng bánh lên thì xe báo `GYRO_ENCODER_DISAGREE` (đúng thiết kế).

## ƯU TIÊN NGÀY MAI: offset từng cảm biến VL53
- Đo thực tế (áp tay sát J9 và J10): J9 báo 35 mm, J10 báo 58 mm. J10 đọc dư hơn J9 khoảng 23 mm. Các lần đo trước J10 cũng luôn cao hơn J9 khoảng 20–30 mm, nên đây là offset cảm biến, không phải góc lắp.
- Hệ quả: START_FIX hiểu sai thành xe xoay lệch rồi báo FAR. RANGE_FIX có thể kéo vị trí sai.
- Trước mắt: `$SET,START_FIX,0` và `$SET,RANGE_FIX,0`.
- Cần làm:
  - Thêm tham số `RANGE_OFS_0..3` (mm) vào TuneTable. Trừ offset khi đọc ở `range_sensors.h` hoặc `scanRanges`, `StartBeams`.
  - Thêm vào `TUNE` (sim.js) và `PARAM_UI` (sa_ban.html), nếu không web sẽ lỗi.
  - Cách đo: tấm phẳng cách đúng 200 mm trước từng cảm biến. Offset = số đọc − 200.
- Góc lắp thật của 4 cảm biến vẫn chưa đo (thân xe bát giác, 2 cảm biến giữa hơi chếch). Đo xe song song tường ở 200 mm, sửa bảng `Mounts` trong `test_static_route.cpp`.

## Cần kiểm tra thêm (Claude ghi)
- [ ] `#PLACE` lúc chờ SW1: mất bao nhiêu ms trên chip thật (fitStart chạy 1 lần/s). Nếu thấy lỗi MPU_STALE khi đang chờ thì giảm số vòng lặp.
- [ ] RANGE0 và RANGE3 từng báo đúng 257 mm suốt cả lượt (log ngày 28/11). Kiểm tra lại khi đặt xe trên sa bàn, nếu không đổi thì nghi cảm biến lỗi.
- [ ] Thứ tự J8..J11: có khớp trái ngoài, trái giữa, phải giữa, phải ngoài trong code không. Che tay từng cái để xác nhận.
- [ ] Bánh trái có vùng chết lớn ở PWM thấp (PWM 12–20 mà gần như không quay). Đo lại ở PWM 15/20/25/30 trên sàn, cân nhắc thêm "mồi" PWM khi khởi động.
- [ ] Đo quãng trôi khi phanh: `CoastDecelMm = 300` đang là giá trị giả định.
- [ ] `#SYNC` khi chạy thật: dưới 5 mm là tốt. Nếu xe rung thì giảm SYNC_GAIN.
- [ ] Gyro: lúc đứng yên đo được +1.4 °/s. Xác nhận firmware trừ đúng độ lệch này (`gyroBias`) khi khởi động.
- [ ] HC-05 đã đổi sang 115200 bằng `AT+UART=115200,0,0` chưa (web giờ chỉ còn 115200).
- [ ] Gia tốc kế trục z đọc 1.11 g (firmware không dùng, chỉ ghi nhận).
- [ ] `test_all.cpp` bản Claude viết chưa từng được build. Bản hiện tại trên máy có thể đã bị sửa, nên build lại trước khi dùng.

## Chưa làm (có prompt sẵn trong lịch sử chat)
- Chế độ IDEAL_RUN: chạy lộ trình không cần VL53, lỗi mềm chỉ cảnh báo, giữ đồng bộ 2 bánh và odometry.
- Đưa `Kp`/`Ki` từng bánh lên web để chỉnh trực tiếp.
