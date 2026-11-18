# UWB_Positioning

Hệ định vị trong nhà bằng UWB: một **tag** đo khoảng cách (SS-TWR) tới nhiều **anchor**
đặt cố định. Máy tính đọc các khoảng cách đó, lọc bằng Kalman và vẽ vị trí tag lên bản đồ 2D.

```
 anchor 1 ─┐
 anchor 2 ─┼── UWB ──  tag ── USB ──  Tools/uwb_tracker  ──  bản đồ 2D
 anchor 3 ─┘
```

## Cấu trúc repo

```
Libraries/
  Dw3000/                      driver DWM3000 (thư viện Arduino "DW3000")
    examples/
      uwb_tag/                 tag: đo khoảng cách tới các anchor
      uwb_anchor_dw3000/       anchor dùng DWM3000
      debug_board_test/        kiểm tra phần cứng một board mới (chân, SPI, driver, nguồn)
      debug_radio_link/        kiểm tra đường truyền radio giữa 2 board (TX/RX, mất gói, dBm)
      debug_imu/               kiểm tra MPU6050 trên tag (bus I2C, WHO_AM_I) và hiệu chuẩn gia tốc 6 mặt
  Dw1000/                      driver DWM1000 (thư viện Arduino "DW1000")
    examples/
      uwb_anchor_dw1000/       anchor dùng DWM1000
Tools/uwb_tracker/             Python: Kalman (EKF) + bản đồ 2D (xem README trong đó)
install_libraries.bat          liên kết 2 thư viện vào Arduino IDE
```

## Phần cứng

| Thiết bị | Vai trò | Ghi chú |
|---|---|---|
| Board custom STM32F103RCT6 + DWM3000 | tag hoặc anchor | Module cắm vào header J7/J8. SPI bit-bang trên PB3/PB4/PB5, CS PA15, IRQ PC12, RST PD2. Log qua cổng USB của board. Nạp bằng ST-Link (SWD). |
| Board custom STM32F103RCT6 + DWM1000 | anchor | Cùng header và chân như trên. |
| ESP32-WROOM-32 + DWM3000 | anchor (trạm thử) | RST 4, IRQ 22, CS 21, SCK 18, MISO 19, MOSI 23. |
| ESP32-C3 SuperMini + DWM1000 | anchor (trạm thử) | RST 2, IRQ 3, SCK 4, MISO 5, MOSI 6, CS 7. |
| MPU6050 (GY-521) trên tag | IMU | I2C1: SCL PB6, SDA PB7, 3V3, địa chỉ 0x68. Hệ số hiệu chuẩn gia tốc ghi trong `uwb_tag` là của module trên board B (trục Z lệch +0.46 g); module khác thì đo lại bằng `debug_imu` (lệnh `a`). |

Module chỉ cấp **3.3 V**. Khi một module chạy chập chờn, trước hết đo **GND giữa board và
module** (phải khoảng 0 Ω): một chân GND hở ở header từng làm board chạy "lúc được lúc không".

## Cài đặt

1. **Arduino IDE 2** và các board package:
   - STM32: `STMicroelectronics:stm32` 3.0.0 (URL: `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`), kèm STM32CubeProgrammer để nạp qua ST-Link.
   - ESP32 (chỉ cho trạm thử): `esp32:esp32` 3.3.x.
2. Clone repo rồi chạy **`install_libraries.bat`**. Script tạo liên kết
   `Documents\Arduino\libraries\Dw3000` và `Dw1000` trỏ vào repo, nên mọi thay đổi trong
   repo có tác dụng ngay khi build. Không copy thư viện bằng tay: bản copy cũ từng làm
   board chạy code khác với code trong repo.
3. Khởi động lại Arduino IDE. Các sketch nằm trong *File > Examples > DW3000 / DW1000*.

Cấu hình board trong Arduino IDE:

| Board | Tools |
|---|---|
| Board custom | Board: *Generic STM32F1 series*, Board part number: *Generic F103RCTx*, USB support: *CDC (generic 'Serial' supersede U(S)ART)*, Upload method: *STM32CubeProgrammer (SWD)* |
| ESP32-WROOM-32 | Board: *ESP32 Dev Module* |
| ESP32-C3 SuperMini | Board: *ESP32C3 Dev Module*, USB CDC On Boot: *Disabled* |

## Sử dụng

1. **Kiểm tra phần cứng** từng board mới bằng `debug_board_test`: mọi dòng phải `[PASS]`.
   Hai board kiểm tra được với nhau bằng `debug_radio_link`.
2. **Anchor**: mỗi anchor một ID riêng (`#define ANCHOR_ID` ở đầu sketch).
   - Module DWM3000: `uwb_anchor_dw3000`
   - Module DWM1000: `uwb_anchor_dw1000`
3. **Tag**: nạp `uwb_tag`. Khai báo các anchor trong bảng `ANCHORS[]` ở đầu sketch: ID và mức
   công suất (`LVL_DW3000` hoặc `LVL_DW1000`). Log in `DIST <id> <mét> m raw <mét>` cho mỗi lần đo
   và `IMU <t_ms> ax ay az gx gy gz` (g, °/s) 100 lần/giây. Lúc bật nguồn để tag nằm yên ~1 s
   để đo bias gyro.
4. **Bản đồ**: điền tọa độ anchor vào `Tools/uwb_tracker/anchors.json`, đóng Serial Monitor rồi
   chạy `python Tools/uwb_tracker/uwb_tracker.py --port COMx` (COM của tag).

Lệnh qua Serial Monitor:

| Sketch | Lệnh |
|---|---|
| `uwb_tag` | `p` dừng/chạy, `z` xóa thống kê, `1`..`9` bật/tắt anchor theo ID, `d` / `i` bật/tắt dòng DIST / IMU, `g` khởi tạo lại IMU + đo bias gyro |
| `debug_imu` | `a` hiệu chuẩn gia tốc 6 mặt, `c` bias gyro, `f` tần số in, `i` / `w` bật/tắt dòng IMU / số thô, `s` dò lại bus |
| `uwb_anchor_dw3000` | `v` in từng lần trả lời, `z` xóa thống kê |
| `uwb_anchor_dw1000` | `v`, `z`, `r` chạy lại kiểm tra board |

## Giao thức

Single-sided TWR, frame IEEE 802.15.4 với địa chỉ ngắn 16 bit (little-endian), PAN `0xDECA`:

```
poll  41 88 | seq | CA DE | dst = anchor | src = tag    | E0                          | CRC
resp  41 88 | seq | CA DE | dst = tag    | src = anchor | E1 | poll_rx_ts | resp_tx_ts | CRC
```

- Radio: kênh 5, preamble 128, PAC 8, code 9, **SFD IEEE 8 symbol** (để DWM1000 cùng giải mã được), 6.8 Mb/s, tắt STS.
- Anchor chép `seq` của poll và trả lời đúng **3 ms** sau timestamp nhận poll. Tag mở RX sau 2 ms,
  chờ tối đa 10 ms, rồi tính khoảng cách có bù lệch xung nhịp (`dwt_readclockoffset`).
- Tag ID mặc định là 100, anchor ID từ 1.
- Đổi bất kỳ điều gì ở trên thì phải sửa cả `uwb_tag`, `uwb_anchor_dw3000` và `uwb_anchor_dw1000`.

## Nguồn gốc mã

- `Libraries/Dw3000`: driver DW3000 C0 của Decawave/Qorvo (API 04.00.00) qua bản port Arduino của Makerfabs, đã sửa lớp nền tảng cho board custom và ESP32.
- `Libraries/Dw1000`: phần cấp thấp của [thotro/arduino-dw1000](https://github.com/thotro/arduino-dw1000) (Apache 2.0), có thêm SPI bit-bang cho STM32.
