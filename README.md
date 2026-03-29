## 🛠 Tên Dự Án 

`Xây dựng hệ thống nhận diện giọng nói dựa trên TinyML`

Người làm chính: Lê Chí Hiếu - B23DCDT089

Đồ án này tập trung vào việc phát triển một hệ thống nhận diện giọng nói đơn giản, có thể nhận diện 4 từ khóa: "bật đèn", "tắt đèn", "mở cửa" và "đóng cửa" và 1 từ Wakeup "xin chào". Mục đích của hệ thống là giúp người dùng điều khiển các thiết bị như đèn và cửa chỉ bằng giọng nói mà không cần sử dụng các thiết bị điều khiển phức tạp.

## 🧰 Tổng quan về công nghệ và thư viện được dùng

•	ESP32-S3: ESP32-S3 là vi điều khiển mạnh mẽ với khả năng xử lý tín hiệu và hỗ trợ AI trực tiếp trên phần cứng. Với bộ vi xử lý này, hệ thống có thể thực hiện các tác vụ nhận diện giọng nói ngay trên thiết bị mà không cần kết nối đến máy chủ, giúp tiết kiệm băng thông và giảm độ trễ.

•	Edge Impulse: Edge Impulse là một nền tảng cho phép phát triển và huấn luyện các mô hình TinyML. Nền tảng này hỗ trợ thu thập dữ liệu, xử lý tín hiệu, huấn luyện mô hình và triển khai mô hình lên các thiết bị nhúng một cách dễ dàng và hiệu quả.

•	MFCC (Mel Frequency Cepstral Coefficients): MFCC là phương pháp phổ biến trong xử lý âm thanh, đặc biệt là trong nhận diện giọng nói. MFCC giúp chuyển đổi tín hiệu âm thanh từ dạng sóng (waveform) sang dạng đặc trưng có thể sử dụng cho học máy. Các đặc trưng này sẽ được dùng làm đầu vào cho các mô hình học máy để nhận diện lệnh giọng nói.

•	Arduino trên VS Code với PlatformIO: Môi trường phát triển này cung cấp một nền tảng lập trình quen thuộc và mạnh mẽ cho việc phát triển ứng dụng trên ESP32-S3. Nó hỗ trợ nhiều thư viện và công cụ cần thiết để lập trình và tương tác với phần cứng một cách dễ dàng.



## Hiệu suất 

Training performance 

<img width="1089" height="1374" alt="Ảnh chụp màn hình 2026-03-29 080139" src="https://github.com/user-attachments/assets/a4fbaf18-733d-4e0b-a1e4-19f39d7a16a8" />


## 💻 Lập Trình

Ngôn ngữ: PlatformIO

Cách build:
```bash
platformio run
```
Cách upload: 
```bash 
platformio run --target upload
```

cách bật monitor:
```bash 
platformio device monitor
```


## Tài liệu chi tiết

[Đồ án hệ thống nhúng](https://docs.google.com/document/d/1MevTHBgarqiUmr2diGxpP9_f-btOGW0k/edit?usp=sharing&ouid=106490323673097930968&rtpof=true&sd=true)


## Demo

https://github.com/user-attachments/assets/7d96476d-f939-46e8-88c0-75df771ffcdd



