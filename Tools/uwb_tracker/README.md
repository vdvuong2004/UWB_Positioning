# UWB tracker: Kalman filter + bản đồ 2D

Đọc khoảng cách từ tag (sketch `uwb_tag`) qua USB, lọc bằng
Kalman (EKF) rồi vẽ vị trí tag lên bản đồ 2D theo thời gian thực.

## Cài đặt

```
python -m pip install pyserial numpy matplotlib
```

## Chuẩn bị

1. Đặt các anchor (tối thiểu 3) thành một tam giác, càng rộng càng tốt. Không đặt
   3 anchor thẳng hàng. Cả 3 anchor và tag nên ở cùng độ cao.
2. Chọn một gốc tọa độ, đo bằng thước vị trí x, y (mét) của từng anchor, rồi điền
   vào `anchors.json` theo ID của anchor (`ANCHOR_ID` trong sketch anchor).
3. Đóng Serial Monitor của Arduino IDE, vì script cần dùng cổng COM của tag.

Script tự bật trên tag mọi anchor có trong `anchors.json` (gửi phím ID nếu anchor đang tắt).

## Chạy

```
python uwb_tracker.py --port COM6                 chạy trực tiếp từ board B
python uwb_tracker.py --port COM6 --log run.txt   vừa chạy vừa lưu dữ liệu
python uwb_tracker.py --replay run.txt            phát lại một lần đo đã lưu
python uwb_tracker.py --simulate                  tag giả lập, không cần phần cứng
```

## Trên bản đồ

- Tam giác: anchor. Vòng tròn nét đứt: khoảng cách đo mới nhất tới anchor đó.
- Chấm xám: nghiệm least-squares thô (chưa lọc).
- Đường đỏ, chấm đỏ: quỹ đạo và vị trí sau Kalman. Elip đỏ: vùng tin cậy 2σ.
- Góc trái: tọa độ, vận tốc, số khoảng cách được dùng hoặc bị loại, số lần khởi động lại bộ lọc.

## Tham số trong `anchors.json`

| Tham số | Ý nghĩa |
|---|---|
| `x`, `y` | Vị trí anchor (m) |
| `bias_m` | Sai lệch cố định của khoảng cách tới anchor đó (m), lấy từ phép đo ở khoảng cách đã biết: `bias = đo được - thật` |
| `range_sigma_m` | Độ nhiễu của một phép đo khoảng cách (std, khoảng 0.1 m) |
| `accel_sigma_mps2` | Mức "linh hoạt" của tag: nhỏ thì đường mượt nhưng trễ, lớn thì bám nhanh nhưng rung. Tag đi bộ: 0.5; di chuyển nhanh, đổi hướng gắt: 1.0 |

## Thuật toán

- EKF, trạng thái `[x, y, vx, vy]`, mô hình vận tốc không đổi.
- Mỗi khoảng cách tới là một lần cập nhật riêng, nên không cần chờ đủ 3 anchor.
- Khoảng cách lệch quá 3σ so với dự đoán thì bị loại (chống đo sai do vật cản / NLOS).
- Bộ lọc được khởi tạo từ nghiệm least-squares đầu tiên. Nếu nhiều nghiệm sạch liên
  tiếp không khớp với bộ lọc (bộ lọc "mất dấu"), nó được khởi tạo lại.

Mô phỏng (tag đi chậm ~0.3 m/s, nhiễu 0.12 m, 3% đo sai): sai số RMS của EKF khoảng
0.18 m, của least-squares thô khoảng 0.33 m.
