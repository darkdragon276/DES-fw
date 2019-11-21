# ROBOT FIRMWARE

### Tính năng

+ Điều khiển robot 5 bậc + 1 cripper bằng 6 động cơ servo
+ Tính toán động học ngược , điều khiển được vị trí của robot
+ Tính toán độ thay đổi của cripper, điều khiển được kích thước và chiều dài của cripper
+ Quy hoạch vận tốc hình thang (lsbp). Cho tay máy cử động ít rung lắc.
+ Quy hoạch thời gian 1 hoạt động của tay máy
+ Đồng bộ lệnh từ máy tính: Nhận lệnh và trả lời lại nếu làm xong yêu cầu hoặc báo lỗi nếu chưa làm
+ Kiểm soát luồn nhận có hàng chờ: Đảm bảo nhận nhiều yêu cầu 1 lúc mà không bị xung đột hoặc mất yêu cầu.

### Cấu trúc request

`<ID_COMMAND> <COMMAND> <PARAMETER>`

### Các lệnh hiện hành

```
SETPOS X Y Z
SETWID WIDTH
SETHOME
SETDUTY DUTY CHANNEL
SETPOSNARG X Y Z ANGLE
SETTIME TIME
SETWIDPOS WIDTH X Y Z
SETPOSANGWID X Y Z ANGLE WIDTH
SAVE
```

### Các lệnh trả lời

```
ERROR COMMAND 	// Lệnh yêu cầu sai cấu trúc
ERROR TRANSMIT	// Lỗi truyền nhận
ERROR ARGUMENT	// Sai tham số truyền vào lệnh
ERROR			// Lỗi Firmware
OVERFLOW		// Tràn buffer lệnh, reset buffer.(quá nhiều lệnh)
PROCESSING		// Đang xử lý lệnh
DONE			// Thực thi xong
```

