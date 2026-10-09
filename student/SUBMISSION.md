# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Công Thịnh
- MSSV: 2A202602781
- Email: thinh20033101@gmail.com
- Link repo (fork): https://github.com/2003congthinh/K4-L2L3-DAY23-NguyenCongThinh-2A202602781-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): xem commit hash đã nộp trên LMS (commit cuối của nhánh `main`)

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]` (199 frame), `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `seed = 0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision = 0.9701, recall = 0.7004, tp = 519, fp = 16, fn = 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1503 m, matches = 502, sum_sq_err = 11.3436 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1359 m, matches = 502, sum_sq_err = 9.2668 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  - **Số ghép, ghost, miss giống hệt nhau** giữa hai mode (502 / 0 / 239), và mình đã kiểm tra trên
    `grade_run.log`: ở **cả 199 frame**, `confirmed`, `matches`, `ghosts`, `misses` của `lidar` và `fused`
    trùng nhau. Đây đúng là thiết kế track-then-fuse: camera không tạo, không xác nhận, không xóa track, nên
    tập confirmed tracks (và việc ghép với GT) không đổi; camera chỉ làm thay đổi **vị trí** ước lượng.
  - Vì cùng một tập 502 cặp ghép, so sánh RMSE là công bằng: fused giảm `sum_sq_err` từ 11.34 xuống 9.27 m²,
    RMSE từ 0.1503 xuống 0.1359 m (**−0.0145 m**, ≈ −9.6 %). Fused tốt hơn ở 137 frame, kém hơn ở 58 frame,
    bằng nhau ở 4 frame (các frame đầu chưa có track confirmed). Đo pixel FRONT bổ sung thông tin ngang/đứng
    (u, v) cho các xe phía trước; nhưng đây là tâm hộp 2D **ground-truth** có nhiễu, nên mức cải thiện này
    không chứng minh một camera detector thật sẽ tốt như vậy.
  - **Chất lượng theo rubric** (cả hai mode như nhau ngoài RMSE): `precision_track = 502/(502+0) = 1.00`
    (≥ 0.75), `coverage = 502/519 = 0.967` (≥ 0.70), RMSE ≤ 0.45 m → hệ số chất lượng q = 1 cho cả hai mode;
    `rmse_fused − rmse_lidar = −0.0145 m ≤ 0.05 m`.
  - **Đọc miss cùng detection:** 239 miss gồm 222 xe mà detector bỏ sót (`det_fn`, tracker không thể bám nếu
    không có đo LiDAR) và 17 lần do trễ xác nhận: track mới cần 4 hit liên tiếp mới vượt `score > 0.8`
    (track confirmed đầu tiên xuất hiện ở frame 4; riêng frame 0–3 đã có 8 miss). Đổi lại, 0 ghost: chưa có
    track confirmed nào nằm xa xe thật quá 2 m. Recall detection 0.70 là giới hạn chính, còn tracker gần như
    không làm mất xe đã được detect (coverage 0.967).

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?

   - **LiDAR:** `z = [x, y, z]ᵀ` (3×1, mét) là tâm hộp 3D của detector, `R = diag(0.1², 0.1², 0.1²)` m².
     `h(x)` là phép biến đổi cứng `R·p + t` nên `H` (3×6) tuyến tính, hằng số theo `x`
     (chính là ma trận quay, cột vận tốc bằng 0). LiDAR đo được cả độ sâu.
   - **Camera:** `z = [u, v]ᵀ` (2×1, pixel), `R = diag(5², 5²)` px²
     ([camera_fusion.py:91](workspace/camera_fusion.py#L91), `build_camera_measurement`).
     `h(x)` là phép chiếu pinhole `u = c_i − f_i·y_s/x_s`, `v = c_j − f_j·z_s/x_s`
     ([camera_fusion.py:62](workspace/camera_fusion.py#L62)) — **phi tuyến** vì chia cho độ sâu `x_s`,
     nên EKF phải dùng Jacobian `H` (2×6) tính lại tại mỗi `x`. Camera **không** đo được độ sâu:
     một pixel ứng với cả một tia, nên update camera chủ yếu sửa vị trí ngang/đứng, còn độ sâu do LiDAR giữ.
   - Cùng một hàm `ekf_update` dùng cho cả hai ([kalman.py:116](workspace/kalman.py#L116)):
     `γ = z − h(x)`, `S = H P Hᵀ + R`, `K = P Hᵀ S⁻¹`; chỉ khác `z`, `R`, `h`, `H` do `meas.sensor` cung cấp.

2. Vì sao cần gating Mahalanobis trước khi gán?

   - Mỗi frame có nhiều track và nhiều đo; gán sai một đo cho track (đo của xe khác, false positive)
     sẽ kéo trạng thái EKF đi xa và có thể làm đổi ID. Gating loại các cặp *không thể* trước khi gán greedy.
   - Khoảng cách Mahalanobis `d² = γᵀ S⁻¹ γ` ([association.py:27](workspace/association.py#L27)) đo độ lệch
     **theo đơn vị độ bất định**: `S` gồm cả độ bất định của track (`H P Hᵀ`) và nhiễu cảm biến (`R`).
     Khi `P` lớn (track mới, vận tốc chưa biết), cùng một độ lệch 1 m sẽ có `d²` nhỏ → vẫn được ghép;
     khi track đã chắc chắn, độ lệch đó bị coi là xa. Euclidean không phân biệt được hai trường hợp này,
     và cũng không so sánh được mét (LiDAR) với pixel (camera).
   - Nếu đo đúng là của track thì `d²` tuân theo phân phối χ² với số bậc tự do = số chiều đo, nên ngưỡng
     `chi2.ppf(0.995, dim_meas)` (≈ 12.84 với LiDAR 3D, ≈ 10.60 với camera 2D) giữ lại 99.5 % cặp đúng
     ([association.py:57](workspace/association.py#L57)). Cặp ngoài FOV bị đặt `inf` **trước** khi tính
     Mahalanobis ([association.py:79](workspace/association.py#L79)), nên xe sau lưng camera không bao giờ bị chiếu.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.

   - **Track-then-fuse.** Chỉ có một danh sách track. Trong vòng lặp mỗi frame của
     `platform/fusion_lab/scripts/run_lab.py`: predict mọi track **một lần** (dòng 199–201) → **AssocL**
     `associate_and_update(..., lidar_sensor)` (dòng 202) → khi fused, **AssocC**
     `associate_and_update(..., camera_sensor)` (dòng 203–209). Không có bước gộp point cloud với ảnh
     trước detection: detector chỉ thấy BEV LiDAR.
   - **Trên log:** `grade_run.log` có hai record cho mỗi frame, `"mode": "lidar"` và `"mode": "fused"`.
     Các trường detection `det_tp`, `det_fp`, `det_fn`, `valid_gt` **giống hệt** nhau giữa hai mode ở mọi frame
     (grader còn bắt buộc điều này trong `aggregate_records`), chỉ các trường tracking (`confirmed`, `matches`,
     `sum_sq_err`, `ghosts`, `misses`) khác nhau. Nghĩa là camera không thay đổi detection; nó chỉ tác động lên
     trạng thái track sau khi đã track bằng LiDAR. Ví dụ frame 131 trong `grade_run.log`:
     `lidar` và `fused` cùng `det_tp=3, det_fp=0, det_fn=0, valid_gt=3, confirmed=3, matches=3, ghosts=0,
     misses=0`, chỉ khác `sum_sq_err` = 0.1230 m² (lidar) so với 0.0581 m² (fused). Kiểm tra toàn bộ log:
     detection và số track trùng nhau ở cả 199 frame, chỉ `sum_sq_err` khác.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?

   - Lệch extrinsic (xoay/tịnh tiến) làm `h(x)` chiếu sai một lượng **có hệ thống**, nên innovation camera
     `γ = z − h(x)` không còn trung bình 0 mà bị **bias cùng dấu** qua nhiều frame và nhiều track
     (ví dụ lệch yaw → `u` lệch đều về một phía; lệch pitch → `v` lệch đều). NIS `γᵀS⁻¹γ` trung bình
     sẽ lớn hơn số chiều đo (2) thay vì ≈ 2.
   - Hai chế độ hỏng: **bias nhỏ** (còn trong cổng χ²) → camera vẫn được gán và kéo trạng thái lệch ngang
     một cách nhất quán, RMSE fused tăng dần so với LiDAR; LiDAR update ở frame sau kéo lại, tạo dao động
     "răng cưa". **Bias lớn** → `d²` vượt ngưỡng ≈ 10.6, gating loại hầu hết đo camera → camera mất tác dụng,
     fused ≈ LiDAR-only. Ước lượng (không đo trong bài): camera FRONT của segment có `f_i ≈ 2083 px`, nên lệch yaw 1°
     dời mọi điểm khoảng `f_i·tan(1°) ≈ 36 px` (không phụ thuộc độ sâu); với `S` cỡ vài trăm px² thì vẫn lọt
     cổng, tức là lỗi calibration nhỏ **nguy hiểm hơn** vì không bị chặn.
   - Do track chỉ được tạo/xóa bởi LiDAR, camera lệch không tạo ghost hay xóa track, chỉ làm sai vị trí.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.

   - Khi `meas_list` rỗng, không thể suy ra sensor từ `meas_list[0].sensor`. Nhưng vẫn phải gọi
     `manager.manage_tracks(unassigned_tracks, [], sensor)` ([association.py:151](workspace/association.py#L151)):
     với **LiDAR**, frame rỗng nghĩa là mọi track trong FOV đều **miss** → trừ score → track cũ phải bị xóa
     (test `test_empty_lidar_frame_scores_then_deletes_exhausted_track`). Nếu bỏ qua hoặc đoán sai sensor:
     LiDAR rỗng bị bỏ qua → track không bao giờ chết → ghost; hoặc camera rỗng bị coi là LiDAR → trừ score oan
     → xóa track thật. `manage_tracks` dựa vào `sensor.name` để quyết định
     (`platform/fusion_lab/tracking/manager.py`, dòng 111: camera thì `return` ngay).
   - LiDAR quyết định tồn tại vì: (a) chỉ LiDAR có detector thật cho mỗi frame trên toàn cửa sổ 50 m × 50 m
     với vị trí 3D đầy đủ, nên tạo track từ đo LiDAR mới khởi tạo được cả độ sâu
     ([track_management.py:20](workspace/track_management.py#L20)); camera 2D không có độ sâu nên không khởi tạo được;
     (b) camera chỉ nhìn phía trước (FOV hẹp) — nếu camera trừ score thì xe ngoài FOV camera bị phạt oan;
     (c) camera trong lab là nhãn GT có nhiễu, không phải detector — để nó cộng score sẽ "mượn" GT để xác nhận track.
     Vì vậy `handle_updated_track` chỉ cộng score khi `sensor.name == "lidar"` (manager.py dòng 95) và camera
     chỉ đi qua `filter_obj.update` để tinh chỉnh `x`, `P`.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.

   - Khởi tạo: `score = 1/window = 1/6`, `state = "initialized"` ([track_management.py:20](workspace/track_management.py#L20)).
   - Mỗi lượt LiDAR (một lần/frame, [track_management.py:46](workspace/track_management.py#L46)):
     hit → `score = min(1, score + 1/6)`; miss **trong FOV LiDAR** → `score −= 1/6` (ngoài FOV không trừ).
     Track chưa confirmed mà có hit → `"tentative"`.
   - **Xác nhận:** `score > 0.8` (strict) → `"confirmed"`. Từ 1/6 cần thêm 4 hit liên tiếp
     (1/6 → 2/6 → 3/6 → 4/6 → 5/6 ≈ 0.833 > 0.8); đúng 0.8 thì chưa confirmed.
   - **Giữ confirmed sau miss:** một track confirmed không bao giờ bị hạ trạng thái; một miss từ `1.0` còn
     `5/6 ≈ 0.833 ≥ 0.6` nên vẫn sống và vẫn confirmed. Hai miss → `4/6 ≈ 0.667` vẫn sống; ba miss → `0.5 < 0.6` bị xóa.
   - **Xóa** ([track_management.py:75](workspace/track_management.py#L75)), các điều kiện OR:
     `P[0,0] > max_P = 9` hoặc `P[1,1] > 9` (độ lệch chuẩn vị trí > 3 m, track đã "lạc") bất kể score;
     hoặc confirmed và `score < 0.6`; hoặc chưa confirmed và `score ≤ 0`.
     Camera không bao giờ gọi hàm này.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (mô hình Claude Opus 5.5)
- Dùng cho phần nào (hàm, câu hỏi, debug): debug và final review cho part E–H (`kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`), soạn nháp câu trả lời 6 câu E–H, debug lỗi `float()` trên ma trận 1×1 với NumPy 2.5 và lỗi encoding cp1252 khi chạy test trên Windows
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` → 128 passed, không còn failed/xfailed; đối chiếu công thức EKF, Mahalanobis/χ², pinhole với `docs/HUONG_DAN_KY_THUAT.md` và các TODO trong file; chạy thử 13 frame `--fusion fused` trước lần chạy chấm điểm; số liệu trong báo cáo lấy từ `metrics.json` của lần chạy `--fusion compare --seed 0`

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
