# Bắt đầu trên JetBot qua Jupyter

Áp dụng khi laptop và JetBot cùng WiFi, bạn đã mở được trang Jupyter của JetBot bằng IP. Các lệnh ở phần **Terminal JetBot** chạy trên Jetson Nano, không chạy trên laptop. Chưa có model weights và `config/calibration.yaml` vẫn còn `TBD`, vì vậy giai đoạn này **không chạy `--mode robot`**.

## Kết nối lại trong một buổi khác

1. Bật JetBot và đợi WiFi kết nối. Laptop cần ở cùng mạng với JetBot.
2. Mở lại **đúng URL Jupyter đã dùng lần trước**, gồm IP, port và đường dẫn (nếu có). Nếu trang không lên, IP của JetBot có thể đã đổi sau reboot.
3. Xem danh sách thiết bị đang kết nối trong trang quản trị router để lấy IP hiện tại của JetBot; hoặc cắm màn hình/bàn phím vào JetBot và chạy `hostname -I`. Nếu trước đây đã bật SSH và biết hostname, có thể thử kết nối qua hostname, nhưng chỉ dùng khi mạng của bạn phân giải được tên đó.
4. Khi đã vào Jupyter, mở **New → Terminal**, chạy `hostname -I` và `pwd` để xác nhận đang thao tác trên JetBot. Đặt DHCP reservation trong router nếu muốn IP ổn định giữa các buổi.

Để dùng pipeline trong Jupyter, bạn **upload cả package Python**, sau đó mở notebook `00_jetbot_first_session.ipynb` nằm trong package. Notebook gọi các module qua Python của chính kernel; không cần chép toàn bộ source vào từng cell.

## 1. Chuyển source từ laptop qua Jupyter

Trên laptop, upload file `jetbot_hri_upload.zip` bằng nút **Upload** trong file browser Jupyter. File ZIP được tạo từ source ở `C:\Users\ADMIN\Documents\CS532\jetbot_hri` và chứa thư mục `jetbot_hri/` ở cấp trên cùng.

Trong Jupyter, mở **New → Terminal**. Nếu bản Jupyter đang dùng không có Terminal, báo lại để chọn cách SSH hoặc chạy lệnh không tương tác qua notebook.

```bash
pwd
ls -lh jetbot_hri_upload.zip
unzip -l jetbot_hri_upload.zip | head
unzip jetbot_hri_upload.zip
cd jetbot_hri
ls
```

Nếu `ls` không tìm thấy ZIP, kiểm tra thư mục đang đứng bằng `pwd` và chuyển Terminal đến thư mục hiển thị ZIP trong file browser Jupyter; đừng chạy `unzip` khi chưa thấy đúng file. Giải nén ngay tại thư mục đó để notebook mới xuất hiện trong file browser. Nếu `jetbot_hri` đã tồn tại ở đó, sao lưu `config/calibration.yaml` và log trước khi trả lời câu hỏi ghi đè của `unzip`. Chỉ upload lại source khi không có chương trình JetBot cũ đang chạy.

Sau khi giải nén, vào thư mục `jetbot_hri` bằng file browser Jupyter và mở **`00_jetbot_first_session.ipynb`**. Chạy lần lượt các cell. Notebook sẽ kiểm tra môi trường, cài source, chạy dry-run toàn FSM, unit tests, WiFi và pin mock; nó không chạy motor thật.

## 2. Kiểm tra môi trường trước khi cài thêm package

Trong Terminal JetBot:

```bash
python3 --version
python3 -c "import sys; print(sys.executable)"
python3 -m pip show jetbot
python3 -c "import torch; print(torch.__version__)"
python3 -c "import tensorrt; print(tensorrt.__version__)"
bash scripts/inspect_platform.sh | tee platform.txt
```

Nếu notebook JetBot đang mở, chạy `import sys; print(sys.executable)` trong một cell để so sánh Python của kernel với Python của Terminal. Chúng có thể khác nhau. **Đừng cài lại Torch/TensorRT/trt_pose theo phiên bản desktop** trước khi biết JetPack/L4T thật.

Gửi lại `platform.txt` và đường dẫn `sys.executable` của notebook. Output có thể có vài dòng `not found`; cứ gửi nguyên văn. Không gửi mật khẩu WiFi hoặc token Jupyter.

## 3. Cài riêng source và chạy dry-run

Trong Terminal JetBot, tại `~/jetbot_hri`:

```bash
python3 -m pip install --user -e .
python3 -m jetbot_hri.main --mode dry-run
python3 -m unittest discover -s tests -q
```

Kết quả mong đợi: dry-run in các chuyển trạng thái đến `Dry-run COMPLETE`; tests đạt. Nếu `pip` lỗi, dừng ở đó và gửi output; đừng tự nâng cấp toàn bộ NVIDIA packages. Dry-run không dùng motor hoặc model thật.

## 4. Kiểm tra từng phần, chưa cho robot chạy

- **WiFi:** `python3 -m scripts.test_wifi`; mong đợi SSID/IP hoặc lỗi có thông tin.
- **Camera:** trước tiên đóng các notebook đang giữ camera. Sau đó `python3 -m scripts.test_camera`. Script hiện dùng OpenCV `VideoCapture(0)`; nếu JetBot image của bạn dùng GStreamer/JetBot Camera API, lỗi ở bước này **không chứng minh camera hỏng**. Gửi output để sửa camera adapter theo image thật.
- **Pin:** nếu đã có `i2c-tools`, chạy `i2cdetect -y -r 1`; nếu chưa có, cài bằng package manager trên JetBot. Chỉ điền `battery.i2c_address` theo thiết bị vừa quét, rồi cài `pi-ina219`/`smbus2` tương thích và chạy `python3 -m scripts.test_battery --config config/calibration.yaml`. Có thể chạy mock trước: `python3 -m scripts.test_battery --mock`.
- **Motor:** chưa chạy `scripts.test_motor` cho tới khi đã xác minh pin, địa chỉ I2C, camera, hướng motor và STOP. Khi thử đầu tiên, nhấc bánh khỏi mặt đất, đặt tốc độ thấp trong config và có ngắt nguồn vật lý.

Các chế độ `--mode perception` và `--mode robot` của baseline hiện yêu cầu model pose/collision đã cài đúng định dạng. Chưa có weights trong ZIP, nên hai chế độ đó chưa phải bước tiếp theo. Không bật service tự khởi động lúc này.

Khi đã có camera, model pose và model collision tương thích, thử `--mode perception` **không motor** trước trong Terminal. Sau khi quan sát prediction/log và hiệu chỉnh `calibration.yaml`, thử motor có bánh nhấc khỏi đất rồi mới chạy `--mode robot` thủ công ở tốc độ thấp. Jupyter dùng để quan sát/chạy lệnh; vòng điều khiển thật vẫn là tiến trình `python3 -m jetbot_hri.main` chạy trên Jetson. Đóng notebook nào đang giữ camera/motor trước khi chạy tiến trình đó.

## 5. Dataset cho né vật cản bạn tự chọn

**Có**, nếu dùng classifier `FREE/BLOCKED` bằng camera RGB như baseline hiện tại, hãy thu ảnh có nhãn từ **camera JetBot thật** trong khu vực demo. Đặt robot đứng yên và chụp ở cùng chiều cao/góc camera khi sử dụng. Chụp nhiều vị trí, khoảng cách, hướng và ánh sáng cho từng loại vật cản bạn chọn; tách ảnh train và ảnh kiểm tra theo cảnh/buổi chụp để kiểm tra khả năng tổng quát.

Nhãn phải theo **khả năng đi qua hành lang phía trước**, không chỉ theo tên vật:

| Nhãn | Ví dụ cần có |
| --- | --- |
| `FREE` | Lối trống; Owner/Receiver ở khoảng tương tác cho phép, không chắn đường. |
| `BLOCKED` | Hộp/ghế/vật bạn chọn trong hành lang; người quá sát; vật chắn trước người đích. |

Nên thêm ca khó: vật ở rìa ảnh, nền/ánh sáng thay đổi, vật nhỏ/thấp, nhiều vật, bóng đổ, người đứng sau vật cản. Giữ một tập ảnh **chưa dùng để huấn luyện** và kiểm tra kỹ false `FREE` trên cảnh bị chắn. Không cần chờ robot tự lái để thu dataset; notebook thu ảnh có thể chạy khi robot đứng yên.

Trong Jupyter JetBot, tìm `~/Notebooks/collision_avoidance/data_collection.ipynb` (đường dẫn có thể khác theo image). Notebook chính thức của NVIDIA hướng dẫn thu nhãn `free` và `blocked`, sau đó train classifier; xem [tài liệu Collision Avoidance](https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/docs/examples/collision_avoidance.md). Hãy dùng notebook đó để thử thu ảnh và cho biết nó lưu model ở định dạng nào. **Model của notebook JetBot không tự động tương thích** với `CollisionModelBackend` hiện đang nhận TorchScript một logit; sau khi có phiên bản JetPack/notebook/weights, cần chuyển model hoặc sửa adapter tương ứng.

Classifier `FREE/BLOCKED` chỉ quyết định **dừng hay được đi tiếp** trong góc nhìn hiện tại. Dataset không tự tạo ra đường vòng. `AVOID` của baseline hiện turn theo `avoid.turn_side` đã cấu hình, LOOK rồi commit-forward ngắn; phải thử riêng và hiệu chỉnh trong không gian an toàn.

## 6. Output cần gửi để chúng ta làm bước kế tiếp

1. `platform.txt` từ script inspect và `sys.executable` của notebook.
2. Output `python3 -m jetbot_hri.main --mode dry-run` và test.
3. Output `python3 -m scripts.test_wifi`, `python3 -m scripts.test_camera`, và `i2cdetect -y -r 1` nếu chạy được.
4. Ảnh cây thư mục `collision_avoidance` trong Jupyter hoặc tên các notebook/weights đang có; nói rõ các loại vật cản muốn robot tránh.

Sau đó mới chốt dependency, camera backend, cách train/export collision model và thứ tự calibration. Không cần bật motor để lấy các thông tin đầu tiên này.
