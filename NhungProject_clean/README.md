# NhungProject — workspace STM32 gọn

## Bắt đầu
1. Giải nén vào `C:\CODE_NHUNG\NhungProject_clean` (đường dẫn ASCII), mở thư mục đó bằng VS Code. Không mở thư mục cha chứa các project cũ.
2. Cài extension PlatformIO IDE nếu chưa có. Mở `src/test_static_route.cpp`.
3. Nhấn Ctrl+Shift+B, chọn *Nap code* để build và nạp ST-LINK, hoặc *Doc Serial* để chọn cổng UART (ví dụ COM4) và mở Serial. Chọn file firmware đúng trước khi gọi task. Chỉ build: `pio run -e test_static_route`, hoặc gõ `/kiem-tra` trong Claude Code.
4. Chương trình test lộ trình chờ SW1 sau hiệu chuẩn gyro. Thử trên sàn trống, đặt đúng START và hướng 90°, sẵn sàng ngắt nguồn. Không kê bánh khi chạy lộ trình vì có kiểm tra trượt bánh. Các bài test encoder/PI riêng thì kê xe để xác nhận chiều.

## Sa bàn web: mô phỏng, chỉnh tham số, điều khiển xe
Mở `tools/web/sa_ban.html` bằng **Chrome hoặc Edge** (task VS Code `Web - Mo sa ban`).
- **Mô phỏng:** chạy đúng thuật toán của firmware trong trình duyệt (`tools/web/sim.js`, cho kết quả trùng bản C++: 36 370 ms ở 400 CPS). Chỉnh 9 tham số bằng thanh trượt, chọn ô vật cản ngẫu nhiên, bấm lên sa bàn để đặt vật lạ, thử sai lệch hiệu chuẩn bánh và nhiễu cảm biến. Hiện vệt xe thật và vệt xe tự ước lượng, khe hở thân xe và va chạm.
- **Xe thật:** nút *Kết nối* chọn cổng COM: dây USB–UART (115200) hoặc Bluetooth HC-05 đã ghép đôi (38400, cũng hiện như một cổng COM trên Windows). Xem xe chạy trực tiếp, *Đọc tham số* / *Gửi tham số* xuống xe, *START*, **DỪNG XE** (phím `Esc`), *Lưu log*.
- **Mở log:** xem lại log UART đã lưu (của xe hoặc của mô phỏng C++).
- **Xuất cấu hình:** sinh `build_flags` cho `platformio.ini` từ các giá trị đang chỉnh.
- Biểu đồ dùng Chart.js từ CDN; không có Internet thì chép `chart.umd.min.js` (4.4.1) vào `tools/web/vendor/`. Sa bàn và kết nối xe vẫn chạy khi offline.

### Lệnh gửi xe (UART hoặc Bluetooth, mỗi lệnh một dòng)
| Lệnh | Tác dụng |
|---|---|
| `$PING` | `#PONG,<trạng thái>` |
| `$GET` | in mọi `#PARAM,TÊN,GIÁ_TRỊ,MIN,MAX` và `#STATE` |
| `$SET,TÊN,GIÁ_TRỊ` | đổi tham số khi xe **không chạy** (đang chạy: `#ERR,BUSY`; ngoài giới hạn: `#ERR,RANGE`) |
| `$DEFAULTS` | về giá trị lúc build |
| `$START` / `$STOP` | như bấm SW1 để chạy / dừng (`REMOTE_STOP`) |
| `$REBOOT` | khởi động lại (không khi đang chạy) |
| `$CAL,START` / `$CAL,STOP` | hiệu chuẩn bằng tay khi chờ START, động cơ tắt: in `#CAL` / `#CAL_DONE,ms,xung_trái,xung_phải,góc_gyro` |

### Hiệu chuẩn bánh xe (làm trước khi thi)
Trên web, mục *Hiệu chuẩn bánh xe → Đo bằng tay*:
1. **Đẩy thẳng:** đặt xe trên vạch, *Bắt đầu đo*, đẩy xe thẳng bằng tay đúng 1000 mm (đo bằng thước), *Kết thúc*. Web tính số xung/mm từng bánh và cảnh báo nếu hai bánh lệch > 2% hoặc đẩy bị quay.
2. **Xoay tại chỗ** (sau bước 1): xoay xe quanh tâm hai bánh đúng 2 vòng, web tính khoảng cách 2 bánh và so với gyro.
3. *Áp dụng kết quả* → chép `build_flags` (`MAP_NAM_COUNTS_PER_MM`, `MAP_NAM_TRACK_MM`) vào `platformio.ini` → nạp lại.
Lặp lại 2–3 lần, lấy trung bình. Trên mô phỏng, chỉ lệch 2% số xung/mm là xe cách GOAL thật 21 mm dù tự báo đã về đích.

Tham số chỉnh được lúc chạy (`src/static_route/tuning.h`): `CRUISE_CPS` 200–700, `APPROACH_CPS`, `AVOID_CPS`, `LAT_ACCEL`, `LATERAL_GAIN`, `HEADING_GAIN`, `FILLET_MM`, `SAFE_MARGIN`, `DETECT_MM`. Giá trị nằm trong RAM: khởi động lại là về mặc định, muốn cố định thì dán `build_flags` từ trang web. Hiệu chuẩn bánh là hằng số lúc build: `-DMAP_NAM_COUNTS_PER_MM=…f -DMAP_NAM_TRACK_MM=…f`.

### Bluetooth HC-05
HC-05 cắm vào PA9/PA10 (cùng USART1 với dây UART, chỉ dùng một trong hai). Build/nạp env `test_static_route_bt` (38400 baud; task `STM32 - Nap ban Bluetooth (HC-05 38400)`).

### Công cụ phát triển
- `tools/run_host_tests.py`: build và chạy 58 kịch bản mô phỏng C++ (400 + 700 CPS) và đối chiếu web với C++ (task `Mo phong - Chay toan bo test`, Ctrl+Shift+P → *Run Test Task*). Cần GCC (`winget install --id BrechtSanders.WinLibs.POSIX.UCRT -e`) và Node.js (`winget install --id OpenJS.NodeJS.LTS -e`).
- Claude Code: gõ `/kiem-tra` để build cả hai firmware, chạy toàn bộ test và nhận báo cáo tiếng Việt (`.claude/skills/kiem-tra`).
- Extension gợi ý (`.vscode/extensions.json`): PlatformIO, Serial Monitor, Codex, Claude Code, Python.

## Vị trí các file
- `src/test_static_route.cpp`: firmware đang phát triển. Xe chạy 5 node **liên tục, ôm cua** (không dừng–xoay), bám đường tham chiếu bo cung tại N1..N3, và **né vật cản mượt** bằng cách dịch ngang đường tham chiếu (lên dốc cosin – giữ – về), dùng 4 cảm biến VL53L0X đầu xe. Giữ nguyên bộ PI bánh, odometry và mọi lớp bảo vệ của bản TEST_LOTRINH gốc.
- `src/static_route/path.h`: dựng đường tham chiếu (đoạn thẳng + cung tròn R = `STATIC_ROUTE_FILLET_MM`, mặc định 150 mm), chiếu vị trí xe lên đường.
- `src/static_route/avoid.h`: bộ nhớ vật cản (hút vào hộp cố định/ứng viên của MAP nếu điểm đo cách ≤ 45 mm, gộp các ước lượng của cùng một hộp; hộp cố định không chiếm bộ nhớ), lập đường né và kiểm tra va chạm bằng **hình chữ nhật thân xe 130 × 100 mm** theo đúng hướng xe với hộp cố định, vùng cấm, biên MAP. Hướng và độ cong của đường né tính chính xác cả trên cung tròn. Khi chưa thấy vật ngẫu nhiên nào, cả 4 ô ứng viên được coi là có thể bị chiếm.
- `src/static_route/tuning.h`: 9 tham số chỉnh được lúc chạy, giới hạn của từng tham số.
- `tools/web/`: sa bàn web (`sa_ban.html`), bộ mô phỏng JS (`sim.js`), bộ đọc log UART (`uart.js`), đối chiếu JS–C++ (`check_sim.js`).
- `src/navigation/range_sensors.h`: driver 4 VL53L0X không chặn vòng lặp (bus I2C2 PB11/PB10, XSHUT PA2..PA5, địa chỉ 0x30..0x33). Vị trí lắp cảm biến lấy theo firmware thi đấu cũ (J8..J11: 2 tia thẳng ±25 mm, 2 tia chéo ±35,43°, cách tâm 45 mm về trước) — **cần đo lại trên xe thật** và sửa `Mounts[]` trong `test_static_route.cpp` nếu khác. Thứ tự 0..3 phải khớp XSHUT PA2..PA5; kiểm bằng `diagnostics/test_vl53.cpp`.
- `include/map_config.h`: MAP 5 node đã thống nhất, tọa độ mm, START (300,-65), N1 (120,390), N2 (605,915), N3 (1600,570), GOAL (1600,-100). `src/static_route/config.h` dùng trực tiếp các node của header này, không còn bảng waypoint thứ hai. MAP 130 x 100 mm, margin 15 mm là thông số đã thống nhất trước đó; header cũ trong ZIP ghi 136 x 116 mm, cần đo xe thật. Kích thước MAP không được dùng để đảm bảo tránh va chạm trong firmware test này.
- `src/static_route/config.h`: dung sai, tham số quay/dừng, thời gian điều khiển.
- `src/navigation/`: driver/pin và hiệu chuẩn dùng bởi test lộ trình. `wheel_calibration.h` mặc định 4.5 count/mm, track 90 mm, chưa hiệu chuẩn thực tế. Encoder TIM2/TIM3, dấu -1/+1.
- `src/diagnostics/`: các chương trình thử phần cứng độc lập. `test_encoder_raw.cpp` lấy bản PWM cố định từ clone lồng; `test_motor_pi.cpp` lấy bản PI trong thư mục ngoài, đổi mặc định từ PIVOT_RIGHT 200 sang FORWARD 200 để tránh quay ngoài ý muốn. Bật đúng chế độ khi thử. `test_led.cpp` vốn là `main.cpp` của dự án cũ.
- `src/competition_legacy/`: giữ riêng các firmware thi đấu cũ (`hybrid_2laps.cpp`, `xe_full.cpp` cùng module `bluetooth.cpp`); chưa ghép với PI/lộ trình mới. Không nạp chúng để thử lộ trình 5 node. `bluetooth.cpp` không có setup/loop độc lập.
- `test/`: test mô phỏng C++ trên máy tính, không phải firmware để nạp.
- `tools/`: script chọn đúng env theo file đang mở và vẽ log UART.
- `platformio.ini`: cấu hình duy nhất, mặc định `test_static_route`, các env có bộ lọc riêng. Platform ststm32@20.0.0 lấy từ cấu hình test lộ trình. UART tự chọn cổng, tốc độ 115200 cho test, 38400 cho firmware thi đấu cũ. Không dùng đường dẫn COM13 cố định.

## Chọn firmware
| File | PlatformIO env | Baud |
|---|---|---:|
| test_static_route.cpp | test_static_route | 115200 |
| test_static_route.cpp (HC-05) | test_static_route_bt | 38400 |
| diagnostics/test_motor_pi.cpp | test_motor_pi | 115200 |
| diagnostics/test_encoder_raw.cpp | test_encoder | 115200 |
| diagnostics/test_mpu.cpp | test_mpu | 115200 |
| diagnostics/test_vl53.cpp | test_vl53 | 115200 |
| diagnostics/test_vl53_encoder.cpp | test_vl53_encoder | 115200 |
| diagnostics/test_distance.cpp | test_distance | 115200 |
| diagnostics/test_led.cpp | test_led | 115200 |
| competition_legacy/hybrid_2laps.cpp | hybrid_2laps | 38400 |
| competition_legacy/xe_full.cpp | xe_full | 38400 |

Nếu task VS Code không tìm được Python của PlatformIO (cài tại vị trí khác mặc định), dùng PlatformIO terminal:
```
pio run -e test_static_route
pio run -e test_static_route -t upload
pio device monitor -p COM4 -b 115200
```
Thay COM4 bằng cổng thật. Các task nạp, monitor chưa được chạy trên Windows hoặc macOS trong phiên này.

## Tốc độ
Trong `src/test_static_route.cpp`, `STATIC_ROUTE_CRUISE_CPS` mặc định 400 CPS/bánh; gần GOAL giới hạn `ApproachCps` 200 CPS, trong vùng né `AvoidCps` 300 CPS. Tốc độ được tính trước theo độ cong phía trước 350 mm (gia tốc ngang ≤ 300 mm/s², bánh ngoài không vượt giới hạn CPS) nên xe giảm tốc **trước** khi vào cua/né. Sau khi chạy đúng dưới tải, có thể build với `-DSTATIC_ROUTE_CRUISE_CPS=700.0f`.

## Né vật cản — cách hoạt động và giới hạn
- 3 lần đo trúng liên tiếp (khoảng cách ≤ 650 mm) mới xác nhận một vật. Vật trùng hộp cố định hoặc ô ứng viên của MAP được gắn đúng tên (`FIXED`/`RANDOM`), vật lạ là `UNKNOWN`.
- Hộp cố định không bao giờ làm xe né: đường tham chiếu đã được kiểm tra, **thân xe** cách mọi hộp cố định, vùng cấm và cả 4 ô ứng viên ≥ 23,7 mm (khe an toàn yêu cầu 15 mm). Vì vậy **vật ngẫu nhiên ở bất kỳ ô nào trong 4 ô đều không chắn lộ trình hiện tại** — xe đi qua không cần né.
- Vật chắn đường: xe dịch ngang đủ để tâm xe cách tâm hộp ≥ 125 mm, lên dốc dài 100..300 mm (bán kính cong ≥ 100 mm), giữ ±100 mm quanh hộp rồi về đường cũ. Kế hoạch được tinh chỉnh khi ước lượng tốt hơn và **chốt** khi xe vào dốc hoặc còn cách hộp 320 mm.
- Không có chỗ lách (ví dụ hộp đặt giữa hành lang N1→N2: mỗi bên chỉ còn ~50 mm cho xe rộng 100 mm): xe dừng an toàn từ xa với `NO_AVOID_PATH`. Thấy quá muộn: `OBSTACLE_TOO_CLOSE`. Tia thẳng thấy vật lạ quá gần: `OBSTACLE_STOP` (ngưỡng 90 mm cộng quãng trôi theo tốc độ, với giả định giảm tốc khi thả trôi 300 mm/s², **cần đo trên sàn thật**). Mất dữ liệu cảm biến > 100 ms: `RANGE_STALE`. Bộ nhớ vật cản đầy: `OBSTACLE_MAP_FULL`.
- Báo cáo cuối có `#MAX_CONTROL_US`: thời gian vòng điều khiển lâu nhất đo **trên xe thật**. Nếu gần 10 000 µs thì việc lập đường né đang quá nặng cho STM32.
- Build `-DSTATIC_ROUTE_NO_RANGE` để thử bám đường mà không cần cảm biến.

## Log UART (115200, hoặc 38400 qua HC-05)
- Lúc khởi động: `#HELLO,STATIC_ROUTE,2`. Mỗi lượt chạy in lại toàn bộ `#PARAM` sau `#ROUTE_STARTED`.
- `#PATH_SEG,i,X0,Y0,H0_DEG,LEN,RADIUS` — đường tham chiếu, in lúc START (RADIUS 0 = thẳng, + = rẽ trái).
- CSV mỗi 100 ms: 16 cột cũ + `S_MM` (quãng đường dọc tham chiếu), `OFFSET_MM` (độ dịch né, + = trái), `RANGE0..3` (mm, -1 = không có vật). `PHASE` là `ALIGN`/`TRACK`/`AVOID`/`SETTLE`.
- `#WAYPOINT_START/END,...` khi đi qua từng node (giữa cung bo).
- `#OBSTACLE,ID,KIND,X,Y,S,LAT` khi xác nhận vật; `#AVOID,ID,SIDE,OFFSET,S0,S1,S2,S3` khi lập kế hoạch né và lần nữa khi chốt (dòng sau cùng là kế hoạch thật); `#AVOID_FAIL,ID,REASON`.
- Báo cáo cuối như cũ (`#ROUTE_FINISHED`/`#ROUTE_FAULT`, `#STOP_REASON`, ...). Lỗi khởi tạo cảm biến in `#RANGE_READY_MASK`.
- Vẽ log: `python tools/plot_route_test.py log.csv -o run.png` (cần matplotlib + shapely) — vẽ cả hộp MAP, đường bo cung, vật phát hiện và đường né.

## Kiểm thử và sao lưu
Xem `VERIFICATION.md`. Bản ZIP gốc không bị sửa. `NhungProject_Archive.zip` là gói nguồn cũ tách riêng, bao gồm cả bản clone lồng và firmware C riêng; không chứa `.git`, `.pio`, build binary hoặc cache. Giữ ZIP gốc nếu cần lịch sử Git và các binary gốc. Không tự chép thư mục archive vào project sạch. Không merge/push/xóa nhánh trên GitHub trong thao tác này. Nếu muốn đưa bản sạch vào Git, hãy dùng nhánh mới sau khi build và kiểm tra các thay đổi, không ghi đè repository hiện tại.

## Định vị bằng VL53 (localize.h)

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `GYRO_SCALE` | 1.0 | Hệ số nhân tốc độ quay gyro. Trang web → Hiệu chỉnh → "Xoay tại chỗ" N vòng → Áp dụng (tự gửi `$SET`). |
| `START_FIX` | 1 | Lúc chờ SW1, trung bình 4 tia rồi khớp với hộp cố định 1–5 để tính lệch đặt xe (±50 mm, ±15°). |
| `RANGE_FIX` | 1 | Lúc chạy, tia trúng mặt hộp cố định sẽ hiệu chỉnh x, y, góc (Kalman vô hướng, tối đa 4 mm và 0.5° mỗi mẫu). |

Log UART:
- `#PLACE,OK|FAR|NO_DATA,dx,dy,dgóc,số_tia,rms`: in mỗi 1 s khi chờ, xem xe đặt lệch bao nhiêu.
- `#START_POSE,...`: giá trị dùng khi START. Nếu `FAR` thì xe giữ vị trí danh định.
- `#POSE_FIX,n,dx,dy`: tổng hiệu chỉnh trong 1 s.

Kết quả mô phỏng:
- Đặt lệch 20 mm, 15 mm, 4°: sai số ở GOAL 4.8 mm (`place`).
- Bánh xe và gyro cùng sai 2%: sai số ở GOAL 31 mm khi bật sửa, 55 mm khi tắt (`drift` / `drift-nofix`). Đoạn cuối không thấy hộp cố định nào, nên vẫn cần `$CAL` và `GYRO_SCALE`.

Giới hạn: sai số offset của VL53 (khoảng ±10 mm) vào thẳng kết quả `START_FIX`. Nếu `#PLACE` báo lệch đều khi xe đã đặt chuẩn, tắt bằng `$SET,START_FIX,0`.

## Đồng bộ 2 bánh (SYNC_GAIN)
- `SYNC_GAIN` mặc định 3, đặt 0 để tắt. Mỗi chu kỳ so số xung "đáng lẽ có" với số xung thật của từng bánh. Bánh nào tụt lại thì được bù tốc độ, mức bù tối đa 30% tốc độ của bánh chậm hơn.
- Log: `#SYNC,<mm>` mỗi 1 s, `#MAX_SYNC_MM` ở báo cáo cuối. Mô phỏng motor phải yếu hơn 15%: lệch 1.3 mm khi bật, 8.5 mm khi tắt.
- Nâng bánh khỏi mặt đất thì firmware báo `GYRO_ENCODER_DISAGREE`. Đây là chủ ý: phải chạy thử trên sàn.
