import pyrealsense2 as rs
import numpy as np
import cv2

# -------------------- 初始化 RealSense 相机 --------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 开始流
profile = pipeline.start(config)

# 获取深度缩放因子（通常为 0.001，表示深度值单位为毫米，转换为米）
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"深度缩放因子: {depth_scale} (深度值 * 缩放因子 = 米)")

# 获取深度相机的内参（用于反投影）
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
print(f"相机内参: {depth_intrinsics}")

# -------------------- 全局变量 --------------------
depth_frame = None               # 当前深度帧（由主循环更新）
points_pixel = []                # 存储两个点的像素坐标 (u, v)
points_3d = []                   # 存储两个点的三维坐标 (x, y, z) 单位：米
MAX_POINTS = 2                   # 最多选取两个点

# -------------------- 鼠标回调函数 --------------------
def mouse_callback(event, x, y, flags, param):
    global depth_frame, points_pixel, points_3d

    if event == cv2.EVENT_LBUTTONDOWN and len(points_pixel) < MAX_POINTS:
        # 确保深度帧有效
        if depth_frame is None:
            print("深度帧未就绪，请稍后点击")
            return

        # 获取该像素的深度值（单位：米）
        depth_value = depth_frame.get_distance(x, y)
        if depth_value == 0.0:
            print(f"点 ({x}, {y}) 深度无效（可能超出范围或物体反光），请重新选择")
            return

        # 将像素坐标 + 深度反投影到相机坐标系下的三维点
        point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth_value)
        points_pixel.append((x, y))
        points_3d.append(point_3d)
        print(f"点 {len(points_pixel)}: 像素 ({x}, {y}), 深度 {depth_value:.3f} m, 三维坐标 ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})")

        # 如果已选满两个点，计算距离
        if len(points_pixel) == MAX_POINTS:
            p1 = np.array(points_3d[0])
            p2 = np.array(points_3d[1])
            dist = np.linalg.norm(p1 - p2)
            print(f"两点之间的距离 = {dist:.3f} 米")

# -------------------- 创建窗口并绑定回调 --------------------
cv2.namedWindow("RealSense Distance Measure")
cv2.setMouseCallback("RealSense Distance Measure", mouse_callback)

print("\n使用说明：")
print("  - 鼠标左键点击选择第一个点，再点击选择第二个点")
print("  - 按 'r' 键重置选择，重新选点")
print("  - 按 'q' 键退出程序")

# -------------------- 主循环 --------------------
try:
    while True:
        # 等待新的帧集
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()   # 更新全局深度帧
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 将彩色帧转换为 numpy 数组
        color_image = np.asanyarray(color_frame.get_data())

        # 在图像上绘制已选中的点
        for i, (u, v) in enumerate(points_pixel):
            cv2.circle(color_image, (u, v), 5, (0, 0, 255), -1)          # 红色圆点
            cv2.putText(color_image, str(i+1), (u+10, v-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 如果两个点都已选中，在图像上显示距离
        if len(points_3d) == MAX_POINTS:
            p1 = np.array(points_3d[0])
            p2 = np.array(points_3d[1])
            dist = np.linalg.norm(p1 - p2)
            cv2.putText(color_image, f"Distance: {dist:.3f} m", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 显示图像
        cv2.imshow("RealSense Distance Measure", color_image)

        # 键盘响应
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            points_pixel.clear()
            points_3d.clear()
            print("已重置选择，可以重新选点")

finally:
    # 停止相机流并关闭窗口
    pipeline.stop()
    cv2.destroyAllWindows()