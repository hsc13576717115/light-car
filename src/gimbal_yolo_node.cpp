// //yolov11

// // #include "rclcpp/rclcpp.hpp"
// // #include "geometry_msgs/msg/point.hpp"
// // #include "visualization_msgs/msg/marker.hpp"
// // #include "damiao_ros2_control/damiao.h"
// // #include "damiao_ros2_control/SerialPort.h"

// // #include <opencv2/opencv.hpp>
// // #include <rknn_api.h>
// // #include <mutex>  
// // #include <thread>
// // #include <vector>
// // #include <cmath>
// // #include <algorithm>
// // #include <chrono>

// // // ====================== 模型 & 输入参数配置 ======================
// // const std::string RKNN_MODEL = "/home/orangepi/yolov11n_rknn/light-pro.rknn";
// // const int INPUT_W = 640, INPUT_H = 480;
// // const float OBJ_THRESH = 0.4f;  // 降低阈值以提高检测灵敏度
// // const float NMS_THRESH  = 0.45f;  // 适当降低NMS阈值
// // const std::vector<std::string> CLASSES = {"ball"};

// // // ====================== 摄像头 & 控制参数配置 ======================
// // const std::string SERIAL_DEV = "/dev/ttyACM0";
// // const int SERIAL_BAUD = B921600;
// // const double CAM_CX = INPUT_W  / 2.0;
// // const double CAM_CY = INPUT_H / 2.0;

// // // 目标区域阈值：在该范围内认为已对准（像素）
// // const double TARGET_ZONE_THRESH = 5.0;
// // // 平滑过渡系数
// // const float SMOOTH_ALPHA = 0.4f;  // 提高平滑系数

// // // ====================== 全局共享变量（使用互斥锁保护） ======================
// // struct TargetInfo { 
// //     bool detected = false; 
// //     float x = 0, y = 0; 
// //     cv::Rect bbox;  // 目标边界框
// //     float score = 0.0f;  // 置信度
// //     float fps = 0.0f;    // 当前帧率
// // };
// // TargetInfo g_target;
// // std::mutex g_target_mutex;  // 保护目标信息的互斥锁

// // // ====================== 工具函数：加载RKNN模型 ======================
// // static unsigned char* load_model(const char* fn, int* size) {
// //     FILE* fp = fopen(fn, "rb");
// //     if (!fp) return nullptr;
// //     fseek(fp, 0, SEEK_END);
// //     *size = ftell(fp);
// //     unsigned char* buf = (unsigned char*)malloc(*size);
// //     fseek(fp, 0, SEEK_SET);
// //     fread(buf, 1, *size, fp);
// //     fclose(fp);
// //     return buf;
// // }

// // // ====================== 目标检测核心函数 ======================
// // struct DetectBox {
// //     int   classId;
// //     float score;
// //     float xmin, ymin, xmax, ymax;
// //     DetectBox(int id, float s, float x1, float y1, float x2, float y2)
// //         : classId(id), score(s), xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}
// // };

// // static float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

// // static float IOU(float xmin1, float ymin1, float xmax1, float ymax1,
// //                  float xmin2, float ymin2, float xmax2, float ymax2) {
// //     float xmin = std::max(xmin1, xmin2);
// //     float ymin = std::max(ymin1, ymin2);
// //     float xmax = std::min(xmax1, xmax2);
// //     float ymax = std::min(ymax1, ymax2);

// //     float innerW = std::max(xmax - xmin, 0.f);
// //     float innerH = std::max(ymax - ymin, 0.f);
// //     float innerA = innerW * innerH;

// //     float area1 = std::max(0.f, (xmax1 - xmin1)) * std::max(0.f, (ymax1 - ymin1));
// //     float area2 = std::max(0.f, (xmax2 - xmin2)) * std::max(0.f, (ymax2 - ymin2));
// //     return innerA / (area1 + area2 - innerA + 1e-6f);
// // }

// // static std::vector<DetectBox> NMS(const std::vector<DetectBox>& dets) {
// //     if (dets.empty()) return {};
// //     std::vector<cv::Rect> boxes;
// //     std::vector<float>    scores;
// //     for (const auto& b : dets) {
// //         boxes.emplace_back(int(b.xmin), int(b.ymin), int(b.xmax - b.xmin), int(b.ymax - b.ymin));
// //         scores.push_back(b.score);
// //     }
// //     std::vector<int> idxs(scores.size());
// //     for (size_t i = 0; i < idxs.size(); ++i) idxs[i] = int(i);
// //     std::sort(idxs.begin(), idxs.end(),
// //               [&scores](int l, int r) { return scores[l] > scores[r]; });

// //     std::vector<DetectBox> keep;
// //     std::vector<bool>      suppressed(scores.size(), false);
// //     for (size_t k = 0; k < idxs.size(); ++k) {
// //         int i = idxs[k];
// //         if (suppressed[i]) continue;
// //         keep.push_back(dets[i]);
// //         for (size_t t = k + 1; t < idxs.size(); ++t) {
// //             int j = idxs[t];
// //             if (IOU(boxes[i].x, boxes[i].y,
// //                     boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height,
// //                     boxes[j].x, boxes[j].y,
// //                     boxes[j].x + boxes[j].width, boxes[j].y + boxes[j].height) > NMS_THRESH)
// //                 suppressed[j] = true;
// //         }
// //     }
// //     return keep;
// // }

// // static std::vector<DetectBox> postprocess(const std::vector<rknn_output>& out,
// //                                           int img_h, int img_w) {
// //     const int headNum = 3;
// //     const std::vector<std::vector<int>> mapSize = {{60, 80}, {30, 40}, {15, 20}};
// //     const std::vector<int> strides = {8, 16, 32};
// //     const int class_num = 1;
// //     const float objectThresh = OBJ_THRESH;

// //     std::vector<std::vector<float>> outputs;
// //     outputs.reserve(out.size());
// //     for (const auto& o : out) {
// //         size_t cnt = o.size / sizeof(float);
// //         float* p = static_cast<float*>(o.buf);
// //         outputs.emplace_back(p, p + cnt);
// //     }

// //     float scale_h = static_cast<float>(img_h) / INPUT_H;
// //     float scale_w = static_cast<float>(img_w) / INPUT_W;

// //     std::vector<DetectBox> detectResult;

// //     for (int idx = 0; idx < headNum; ++idx) {
// //         const auto& ms = mapSize[idx];
// //         int map_h = ms[0];
// //         int map_w = ms[1];
// //         const std::vector<float>& reg = outputs[idx * 2 + 0];
// //         const std::vector<float>& cls = outputs[idx * 2 + 1];

// //         if ((int)cls.size() != class_num * map_h * map_w) {
// //             RCLCPP_WARN(rclcpp::get_logger("yolo"), "cls size mismatch head=%d: got %zu expected %d", idx, cls.size(), class_num * map_h * map_w);
// //             continue;
// //         }
// //         if ((int)reg.size() != 4 * 16 * map_h * map_w) {
// //             RCLCPP_WARN(rclcpp::get_logger("yolo"), "reg size mismatch head=%d: got %zu expected %d", idx, reg.size(), 4 * 16 * map_h * map_w);
// //             continue;
// //         }

// //         for (int h = 0; h < map_h; ++h) {
// //             for (int w = 0; w < map_w; ++w) {
// //                 int cls_index = 0 * map_h * map_w + h * map_w + w;
// //                 float raw_conf = cls[cls_index];
// //                 float conf = sigmoid(raw_conf);
// //                 if (conf <= objectThresh) continue;

// //                 float regdfl[4] = {0,0,0,0};
// //                 for (int side = 0; side < 4; ++side) {
// //                     int side_base = side * (16 * map_h * map_w);
// //                     float sum_exp = 0.f;
// //                     float exps[16];
// //                     for (int k = 0; k < 16; ++k) {
// //                         int idx_reg = side_base + k * (map_h * map_w) + h * map_w + w;
// //                         float v = reg[idx_reg];
// //                         float e = std::exp(v);
// //                         exps[k] = e;
// //                         sum_exp += e;
// //                     }
// //                     if (sum_exp <= 0.f) sum_exp = 1e-6f;
// //                     float loc = 0.f;
// //                     for (int k = 0; k < 16; ++k) loc += exps[k] * k;
// //                     loc /= sum_exp;
// //                     regdfl[side] = loc;
// //                 }

// //                 float grid_x = (w + 0.5f);
// //                 float grid_y = (h + 0.5f);
// //                 float stride = static_cast<float>(strides[idx]);

// //                 float x1 = (grid_x - regdfl[0]) * stride;
// //                 float y1 = (grid_y - regdfl[1]) * stride;
// //                 float x2 = (grid_x + regdfl[2]) * stride;
// //                 float y2 = (grid_y + regdfl[3]) * stride;

// //                 float xmin = x1 * scale_w;
// //                 float ymin = y1 * scale_h;
// //                 float xmax = x2 * scale_w;
// //                 float ymax = y2 * scale_h;

// //                 xmin = std::max(0.f, xmin);
// //                 ymin = std::max(0.f, ymin);
// //                 xmax = std::min(static_cast<float>(img_w), xmax);
// //                 ymax = std::min(static_cast<float>(img_h), ymax);

// //                 detectResult.emplace_back(0, conf, xmin, ymin, xmax, ymax);
// //             }
// //         }
// //     }

// //     return NMS(detectResult);
// // }

// // // ====================== PID控制器类 -----------------------
// // struct PIDController {
// //     double kp;                  // 比例增益
// //     double ki;                  // 积分增益
// //     double kd;                  // 微分增益
// //     double out_min;             // 输出最小值
// //     double out_max;             // 输出最大值
// //     double target_zone_thresh;  // 目标区域阈值
    
// //     double integral;            // 积分项
// //     double prev_error;          // 上一次误差
// //     double derivative;          // 微分项

// //     // 构造函数
// //     PIDController(double kp_val, double ki_val, double kd_val, 
// //                  double min_val, double max_val, double zone)
// //         : kp(kp_val), ki(ki_val), kd(kd_val), 
// //           out_min(min_val), out_max(max_val), 
// //           target_zone_thresh(zone),
// //           integral(0.0), prev_error(0.0), derivative(0.0) {}

// //     double update(double err, double dt) {
// //         // 如果在目标区域内，重置积分并输出0
// //         if (std::abs(err) < target_zone_thresh) {
// //             integral = 0.0;
// //             prev_error = 0.0;
// //             derivative = 0.0;
// //             return 0.0;
// //         }
        
// //         // 计算微分
// //         derivative = (err - prev_error) / dt;
        
// //         // 计算积分（带积分限幅）
// //         integral += err * dt;
// //         // 积分限幅，防止积分饱和
// //         double max_integral = (out_max * 0.5) / ki;
// //         if (ki > 0) {
// //             integral = std::clamp(integral, -max_integral, max_integral);
// //         } else {
// //             integral = 0.0;  // 如果ki为0，禁用积分
// //         }
        
// //         // PID计算
// //         double out = kp * err + ki * integral + kd * derivative;
        
// //         // 保存当前误差作为下一次的前项误差
// //         prev_error = err;
        
// //         // 输出限幅
// //         return std::clamp(out, out_min, out_max);
// //     }
    
// //     // 重置PID控制器
// //     void reset() {
// //         integral = 0.0;
// //         prev_error = 0.0;
// //         derivative = 0.0;
// //     }
// // };

// // static inline float ema_filter(float prev, float val, float alpha) {
// //     return prev * (1.0f - alpha) + val * alpha;
// // }

// // // 平滑过渡函数：当位置变化过大时进行平滑
// // static inline cv::Point2f smooth_transition(const cv::Point2f& prev_pos, 
// //                                            const cv::Point2f& new_pos, 
// //                                            float max_change) {
// //     float dx = new_pos.x - prev_pos.x;
// //     float dy = new_pos.y - prev_pos.y;
// //     float dist = std::sqrt(dx*dx + dy*dy);
    
// //     // 如果变化在阈值内，直接使用新位置
// //     if (dist <= max_change) {
// //         return new_pos;
// //     }
    
// //     // 否则按比例平滑过渡
// //     float ratio = max_change / dist;
// //     return cv::Point2f(
// //         prev_pos.x + dx * ratio * SMOOTH_ALPHA,
// //         prev_pos.y + dy * ratio * SMOOTH_ALPHA
// //     );
// // }

// // // ====================== 线程：YOLO推理与目标检测 ======================
// // void yolo_thread() {
// //     int model_len = 0;
// //     unsigned char* model_buf = load_model(RKNN_MODEL.c_str(), &model_len);
// //     if (!model_buf) {
// //         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Load model failed");
// //         return;
// //     }

// //     rknn_context ctx;
// //     if (rknn_init(&ctx, model_buf, model_len, RKNN_FLAG_PRIOR_MEDIUM, nullptr) != RKNN_SUCC) {
// //         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "rknn_init failed");
// //         free(model_buf);
// //         return;
// //     }
// //     free(model_buf);

// //     // 帧率计算相关变量
// //     int frame_count = 0;
// //     float fps = 0.0f;
// //     auto start_time = std::chrono::steady_clock::now();
// //     const int FPS_UPDATE_INTERVAL = 10;  // 每10帧更新一次帧率
    
// //     cv::VideoCapture cap(0, cv::CAP_V4L2);
// //     cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
// //     cap.set(cv::CAP_PROP_FRAME_WIDTH,  INPUT_W);
// //     cap.set(cv::CAP_PROP_FRAME_HEIGHT, INPUT_H);
// //     cap.set(cv::CAP_PROP_FPS, 30);
// //     if (!cap.isOpened()) {
// //         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Camera open failed");
// //         rknn_destroy(ctx);
// //         return;
// //     }

// //     cv::Mat frame, rgb, show;
// //     const int OUT_NUM = 6;
// //     rknn_output outputs[OUT_NUM] = {};
    
// //     // 用于平滑目标位置
// //     cv::Point2f last_smoothed_pos(CAM_CX, CAM_CY);
// //     const float POSITION_CHANGE_THRESH = 20.0f;  // 位置突变阈值（像素）

// //     while (true) {
// //         cap >> frame;
// //         if (frame.empty()) continue;

// //         // 计算帧率
// //         frame_count++;
// //         if (frame_count % FPS_UPDATE_INTERVAL == 0) {
// //             auto end_time = std::chrono::steady_clock::now();
// //             std::chrono::duration<float> elapsed = end_time - start_time;
// //             fps = FPS_UPDATE_INTERVAL / elapsed.count();
// //             start_time = end_time;
// //         }

// //         // 预处理图像
// //         cv::resize(frame, rgb,  cv::Size(INPUT_W, INPUT_H));
// //         show = frame.clone();  // 用于显示的图像
// //         cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);

// //         // 存储最终要发布的目标信息
// //         TargetInfo current_target;
// //         current_target.fps = fps;  // 更新帧率信息
// //         bool detected_by_yolo = false;
// //         cv::Point2f yolo_pos;
// //         float yolo_score = 0.0f;

// //         // YOLO检测
// //         rknn_input inputs[1] = {};
// //         inputs[0].index = 0;
// //         inputs[0].type  = RKNN_TENSOR_UINT8;
// //         inputs[0].fmt   = RKNN_TENSOR_NHWC;
// //         inputs[0].buf   = rgb.data;
// //         inputs[0].size  = rgb.total() * rgb.elemSize();
// //         if (rknn_inputs_set(ctx, 1, inputs) == RKNN_SUCC) {
// //             if (rknn_run(ctx, nullptr) == RKNN_SUCC) {
// //                 for (int i = 0; i < OUT_NUM; ++i) outputs[i].want_float = 1;
// //                 if (rknn_outputs_get(ctx, OUT_NUM, outputs, nullptr) == RKNN_SUCC) {
// //                     std::vector<rknn_output> out_vec(outputs, outputs + OUT_NUM);
// //                     std::vector<DetectBox> keep = postprocess(out_vec, INPUT_H, INPUT_W);

// //                     // 如果检测到目标
// //                     if (!keep.empty()) {
// //                         detected_by_yolo = true;
                        
// //                         // 取置信度最高的目标
// //                         auto& best_box = keep[0];
// //                         yolo_score = best_box.score;  // 保存置信度
// //                         int x1 = int(std::round(best_box.xmin));
// //                         int y1 = int(std::round(best_box.ymin));
// //                         int x2 = int(std::round(best_box.xmax));
// //                         int y2 = int(std::round(best_box.ymax));
// //                         cv::Rect bbox(x1, y1, x2 - x1, y2 - y1);

// //                         // 绘制YOLO检测框（绿色）
// //                         cv::rectangle(show, bbox, cv::Scalar(0, 255, 0), 2);
// //                         cv::putText(show, cv::format("ball %.2f", best_box.score),
// //                                     cv::Point(x1, std::max(0, y1 - 5)), cv::FONT_HERSHEY_SIMPLEX,
// //                                     0.6, cv::Scalar(0, 0, 255), 2);

// //                         // 计算目标中心
// //                         yolo_pos.x = (best_box.xmin + best_box.xmax) * 0.5f;
// //                         yolo_pos.y = (best_box.ymin + best_box.ymax) * 0.5f;
                        
// //                         // 平滑目标位置
// //                         last_smoothed_pos = smooth_transition(last_smoothed_pos, yolo_pos, POSITION_CHANGE_THRESH);
                        
// //                         // 更新目标信息
// //                         current_target.detected = true;
// //                         current_target.x = last_smoothed_pos.x;
// //                         current_target.y = last_smoothed_pos.y;
// //                         current_target.bbox = bbox;
// //                         current_target.score = yolo_score;
// //                     }
// //                     rknn_outputs_release(ctx, OUT_NUM, outputs);
// //                 }
// //             }
// //         }

// //         // 如果没有检测到目标
// //         if (!detected_by_yolo) {
// //             current_target.detected = false;
// //         }

// //         // 加锁更新共享目标信息
// //         std::lock_guard<std::mutex> lock(g_target_mutex);
// //         g_target = current_target;

// //         // 绘制目标区域辅助调试
// //         cv::circle(show, cv::Point((int)CAM_CX, (int)CAM_CY), (int)TARGET_ZONE_THRESH, cv::Scalar(0, 255, 255), 1);
// //         // 绘制检测到的中心点（红色）
// //         if (current_target.detected) {
// //             cv::circle(show, cv::Point((int)last_smoothed_pos.x, (int)last_smoothed_pos.y), 3, cv::Scalar(0, 0, 255), -1);
// //         }

// //         // 显示帧率
// //         cv::putText(show, cv::format("FPS: %.1f", fps),
// //                     cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
// //                     1.0, cv::Scalar(255, 0, 0), 2);

// //         cv::imshow("YOLO Detection", show);
// //         cv::waitKey(1);
// //     }

// //     rknn_destroy(ctx);
// // }

// // // ====================== ROS 2 节点类：PID控制云台 ======================
// // class GimbalYoloNode : public rclcpp::Node {
// // public:
// //     // 构造函数
// //     GimbalYoloNode()
// //     : Node("gimbal_yolo_node"),
// //       pid_pan_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),  // 临时初始值
// //       pid_tilt_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  // 临时初始值
// //     {
// //         // 声明PID控制参数
// //         declare_parameter<double>("kp_pan", 0.0025);    // 水平方向比例增益
// //         declare_parameter<double>("ki_pan", 0.0001);   // 水平方向积分增益
// //         declare_parameter<double>("kd_pan", 0.0005);   // 水平方向微分增益
// //         declare_parameter<double>("kp_tilt", 0.0013);  // 垂直方向比例增益
// //         declare_parameter<double>("ki_tilt", 0.00005); // 垂直方向积分增益
// //         declare_parameter<double>("kd_tilt", 0.0002);  // 垂直方向微分增益

// //         // 读取参数
// //         double kp_pan = get_parameter("kp_pan").as_double();
// //         double ki_pan = get_parameter("ki_pan").as_double();
// //         double kd_pan = get_parameter("kd_pan").as_double();
// //         double kp_tilt = get_parameter("kp_tilt").as_double();
// //         double ki_tilt = get_parameter("ki_tilt").as_double();
// //         double kd_tilt = get_parameter("kd_tilt").as_double();

// //         // 初始化串口和电机控制
// //         serial_ = std::make_shared<SerialPort>(SERIAL_DEV, SERIAL_BAUD);
// //         dm_ = damiao::Motor_Control(serial_);

// //         // 电机配置
// //         m_horizontal_ = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x06, 0x16);
// //         m_vertical_   = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x05, 0x15);

// //         dm_.addMotor(m_horizontal_.get());
// //         dm_.addMotor(m_vertical_.get());

// //         // 速度模式
// //         dm_.switchControlMode(*m_horizontal_, damiao::VEL_MODE);
// //         dm_.switchControlMode(*m_vertical_,   damiao::VEL_MODE);

// //         dm_.set_zero_position(*m_horizontal_);
// //         dm_.set_zero_position(*m_vertical_);

// //         dm_.enable(*m_horizontal_);
// //         dm_.enable(*m_vertical_);

// //         // 初始化PID控制器
// //         max_vel_ = 15.0;  // 电机最大速度
// //         pid_pan_ = PIDController(kp_pan, ki_pan, kd_pan, -max_vel_, max_vel_, TARGET_ZONE_THRESH);
// //         pid_tilt_ = PIDController(kp_tilt, ki_tilt, kd_tilt, -max_vel_, max_vel_, TARGET_ZONE_THRESH);
        
// //         // 过滤参数
// //         meas_smooth_alpha_ = 0.5f;  // 测量滤波
// //         cmd_smooth_alpha_ = 0.25f;  // 命令平滑
// //         deadzone_px_ = 10.0;         // 死区像素

// //         cmd_prev_pan_ = 0.0;
// //         cmd_prev_tilt_ = 0.0;
// //         filt_cx_ = CAM_CX;
// //         filt_cy_ = CAM_CY;
// //         last_seen_time_ = std::chrono::steady_clock::now();
// //         last_control_time_ = std::chrono::steady_clock::now();

// //         // 控制循环定时器 (50Hz)
// //         timer_ = create_wall_timer(std::chrono::milliseconds(20),
// //                                    std::bind(&GimbalYoloNode::control_loop, this));
// //         marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
// //         RCLCPP_INFO(get_logger(), "Gimbal + YOLO PID control node started.");
// //     }

// // private:
// //     void control_loop() {
// //         // 计算控制周期
// //         auto now = std::chrono::steady_clock::now();
// //         std::chrono::duration<double> dt_duration = now - last_control_time_;
// //         double dt = dt_duration.count();
// //         last_control_time_ = now;

// //         // 加锁读取共享目标信息
// //         TargetInfo info;
// //         {
// //             std::lock_guard<std::mutex> lock(g_target_mutex);
// //             info = g_target;
// //         }
        
// //         if (info.detected) {
// //             last_seen_time_ = now;

// //             // 测量值平滑滤波
// //             filt_cx_ = ema_filter(filt_cx_, info.x, (float)meas_smooth_alpha_);
// //             filt_cy_ = ema_filter(filt_cy_, info.y, (float)meas_smooth_alpha_);

// //             // 计算像素误差
// //             double err_x = filt_cx_ - CAM_CX;
// //             double err_y = filt_cy_ - CAM_CY;

// //             // 死区处理
// //             if (std::abs(err_x) < deadzone_px_) err_x = 0.0;
// //             if (std::abs(err_y) < deadzone_px_) err_y = 0.0;

// //             // PID控制计算速度命令
// //             double cmd_pan = pid_pan_.update(err_x, dt);
// //             double cmd_tilt = pid_tilt_.update(err_y, dt);

// //             // 命令平滑
// //             cmd_prev_pan_  = ema_filter((float)cmd_prev_pan_,  (float)cmd_pan,  (float)cmd_smooth_alpha_);
// //             cmd_prev_tilt_ = ema_filter((float)cmd_prev_tilt_, (float)cmd_tilt, (float)cmd_smooth_alpha_);

// //             // 下发速度命令（注意方向调整）
// //             dm_.control_vel(*m_horizontal_, -cmd_prev_pan_);
// //             dm_.control_vel(*m_vertical_,    cmd_prev_tilt_);

// //             // 发布调试Marker
// //             publish_marker(err_x, err_y);
// //             return;
// //         }

// //         // 目标未检测到：停止电机并重置PID
// //         cmd_prev_pan_ = 0.0;
// //         cmd_prev_tilt_ = 0.0;
// //         dm_.control_vel(*m_horizontal_, 0.0);
// //         dm_.control_vel(*m_vertical_,   0.0);
// //         filt_cx_ = CAM_CX;
// //         filt_cy_ = CAM_CY;
// //         pid_pan_.reset();  // 重置PID控制器
// //         pid_tilt_.reset();
// //         publish_marker(NAN, NAN);
// //     }

// //     void publish_marker(double err_x, double err_y) {
// //         visualization_msgs::msg::Marker m;
// //         m.header.frame_id = "map";
// //         m.header.stamp = now();
// //         m.ns = "target";
// //         m.id = 0;
// //         m.type = m.SPHERE;
// //         m.action = m.ADD;
// //         if (std::isnan(err_x) || std::isnan(err_y)) {
// //             m.pose.position.x = 0;
// //             m.pose.position.y = 0;
// //         } else {
// //             m.pose.position.x = err_x;
// //             m.pose.position.y = err_y;
// //         }
// //         m.pose.position.z = 0;
// //         m.scale.x = m.scale.y = m.scale.z = 0.05;
// //         m.color.a = 1.0;
// //         m.color.g = 1.0;
// //         marker_pub_->publish(m);
// //     }

// //     // 成员变量
// //     rclcpp::TimerBase::SharedPtr timer_;
// //     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
// //     std::shared_ptr<SerialPort> serial_;
// //     damiao::Motor_Control dm_;
// //     std::shared_ptr<damiao::Motor> m_horizontal_, m_vertical_;

// //     // 控制参数
// //     PIDController pid_pan_, pid_tilt_;  // PID控制器
// //     double max_vel_;
// //     double cmd_prev_pan_, cmd_prev_tilt_;
// //     double filt_cx_, filt_cy_;
// //     double meas_smooth_alpha_, cmd_smooth_alpha_;
// //     double deadzone_px_;
// //     std::chrono::steady_clock::time_point last_seen_time_;
// //     std::chrono::steady_clock::time_point last_control_time_;
// // };

// // // ====================== 主函数 ======================
// // int main(int argc, char** argv) {
// //     std::thread t(yolo_thread);
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<GimbalYoloNode>());
// //     rclcpp::shutdown();
// //     t.join();
// //     return 0;
// // }

// //yolov11+追踪器kcf

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "visualization_msgs/msg/marker.hpp"
// #include "damiao_ros2_control/damiao.h"
// #include "damiao_ros2_control/SerialPort.h"

// #include <opencv2/opencv.hpp>
// #include <opencv2/tracking.hpp>     // KCF
// #include <rknn_api.h>
// #include <mutex>
// #include <thread>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <chrono>
// #include <optional>

// // ====================== 模型 & 输入参数配置 ======================
// const std::string RKNN_MODEL = "/home/orangepi/yolov11n_rknn/light-1.rknn";
// const int INPUT_W = 640, INPUT_H = 480;
// const float OBJ_THRESH = 0.4f;
// const float NMS_THRESH  = 0.45f;
// const std::vector<std::string> CLASSES = {"ball"};

// // ====================== 摄像头 & 控制参数配置 ======================
// const std::string SERIAL_DEV = "/dev/ttyACM0";
// const int SERIAL_BAUD = B921600;
// const double CAM_CX = INPUT_W  / 2.0;
// const double CAM_CY = INPUT_H / 2.0;

// // 目标区域阈值：在该范围内认为已对准（像素）
// const double TARGET_ZONE_THRESH = 0.0;
// // 平滑过渡系数
// const float SMOOTH_ALPHA = 0.4f;

// // ========== YOLO 与 Tracker 协同的调度/容错参数 ==========
// const int   YOLO_INTERVAL          = 3;     // 每隔几帧运行一次 YOLO
// const int   TRACKER_MAX_LOSS       = 15;    // 追踪器连续失败多少帧后判定丢失
// const float REINIT_IOU_THRESH      = 0.3f;  // YOLO 与 Tracker 框 IoU 过低则用 YOLO 重置
// const float SCORE_REINIT_THRESH    = 0.3f; // YOLO 置信度超过该阈值时优先以 YOLO 框为准
// const float POSITION_CHANGE_THRESH = 20.0f; // 目标位置突变平滑阈值（像素）

// // ====================== 全局共享变量（使用互斥锁保护） ======================
// struct TargetInfo {
//     bool detected = false;
//     float x = 0, y = 0;
//     cv::Rect bbox;     // 目标边界框
//     float score = 0.0f;
//     float fps = 0.0f;
//     enum Source { NONE=0, YOLO=1, TRACKER=2 } src = NONE;
// };
// TargetInfo g_target;
// std::mutex g_target_mutex;

// // ====================== 工具函数：加载RKNN模型 ======================
// static unsigned char* load_model(const char* fn, int* size) {
//     FILE* fp = fopen(fn, "rb");
//     if (!fp) return nullptr;
//     fseek(fp, 0, SEEK_END);
//     *size = ftell(fp);
//     unsigned char* buf = (unsigned char*)malloc(*size);
//     fseek(fp, 0, SEEK_SET);
//     fread(buf, 1, *size, fp);
//     fclose(fp);
//     return buf;
// }

// // ====================== 检测 & NMS ======================
// struct DetectBox {
//     int   classId;
//     float score;
//     float xmin, ymin, xmax, ymax;
//     DetectBox(int id, float s, float x1, float y1, float x2, float y2)
//         : classId(id), score(s), xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}
// };

// static float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

// static float IOU(float xmin1, float ymin1, float xmax1, float ymax1,
//                  float xmin2, float ymin2, float xmax2, float ymax2) {
//     float xmin = std::max(xmin1, xmin2);
//     float ymin = std::max(ymin1, ymin2);
//     float xmax = std::min(xmax1, xmax2);
//     float ymax = std::min(ymax1, ymax2);

//     float innerW = std::max(xmax - xmin, 0.f);
//     float innerH = std::max(ymax - ymin, 0.f);
//     float innerA = innerW * innerH;

//     float area1 = std::max(0.f, (xmax1 - xmin1)) * std::max(0.f, (ymax1 - ymin1));
//     float area2 = std::max(0.f, (xmax2 - xmin2)) * std::max(0.f, (ymax2 - ymin2));
//     return innerA / (area1 + area2 - innerA + 1e-6f);
// }

// static std::vector<DetectBox> NMS(const std::vector<DetectBox>& dets) {
//     if (dets.empty()) return {};
//     std::vector<cv::Rect> boxes;
//     std::vector<float>    scores;
//     for (const auto& b : dets) {
//         boxes.emplace_back(int(b.xmin), int(b.ymin), int(b.xmax - b.xmin), int(b.ymax - b.ymin));
//         scores.push_back(b.score);
//     }
//     std::vector<int> idxs(scores.size());
//     for (size_t i = 0; i < idxs.size(); ++i) idxs[i] = int(i);
//     std::sort(idxs.begin(), idxs.end(),
//               [&scores](int l, int r) { return scores[l] > scores[r]; });

//     std::vector<DetectBox> keep;
//     std::vector<bool>      suppressed(scores.size(), false);
//     for (size_t k = 0; k < idxs.size(); ++k) {
//         int i = idxs[k];
//         if (suppressed[i]) continue;
//         keep.push_back(dets[i]);
//         for (size_t t = k + 1; t < idxs.size(); ++t) {
//             int j = idxs[t];
//             if (IOU(boxes[i].x, boxes[i].y,
//                     boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height,
//                     boxes[j].x, boxes[j].y,
//                     boxes[j].x + boxes[j].width, boxes[j].y + boxes[j].height) > NMS_THRESH)
//                 suppressed[j] = true;
//         }
//     }
//     return keep;
// }

// static std::vector<DetectBox> postprocess(const std::vector<rknn_output>& out,
//                                           int img_h, int img_w) {
//     const int headNum = 3;
//     const std::vector<std::vector<int>> mapSize = {{60, 80}, {30, 40}, {15, 20}};
//     const std::vector<int> strides = {8, 16, 32};
//     const int class_num = 1;
//     const float objectThresh = OBJ_THRESH;

//     std::vector<std::vector<float>> outputs;
//     outputs.reserve(out.size());
//     for (const auto& o : out) {
//         size_t cnt = o.size / sizeof(float);
//         float* p = static_cast<float*>(o.buf);
//         outputs.emplace_back(p, p + cnt);
//     }

//     float scale_h = static_cast<float>(img_h) / INPUT_H;
//     float scale_w = static_cast<float>(img_w) / INPUT_W;

//     std::vector<DetectBox> detectResult;

//     for (int idx = 0; idx < headNum; ++idx) {
//         const auto& ms = mapSize[idx];
//         int map_h = ms[0];
//         int map_w = ms[1];
//         const std::vector<float>& reg = outputs[idx * 2 + 0];
//         const std::vector<float>& cls = outputs[idx * 2 + 1];

//         if ((int)cls.size() != class_num * map_h * map_w) {
//             RCLCPP_WARN(rclcpp::get_logger("yolo"), "cls size mismatch head=%d: got %zu expected %d", idx, cls.size(), class_num * map_h * map_w);
//             continue;
//         }
//         if ((int)reg.size() != 4 * 16 * map_h * map_w) {
//             RCLCPP_WARN(rclcpp::get_logger("yolo"), "reg size mismatch head=%d: got %zu expected %d", idx, reg.size(), 4 * 16 * map_h * map_w);
//             continue;
//         }

//         for (int h = 0; h < map_h; ++h) {
//             for (int w = 0; w < map_w; ++w) {
//                 int cls_index = 0 * map_h * map_w + h * map_w + w;
//                 float raw_conf = cls[cls_index];
//                 float conf = sigmoid(raw_conf);
//                 if (conf <= objectThresh) continue;

//                 float regdfl[4] = {0,0,0,0};
//                 for (int side = 0; side < 4; ++side) {
//                     int side_base = side * (16 * map_h * map_w);
//                     float sum_exp = 0.f;
//                     float exps[16];
//                     for (int k = 0; k < 16; ++k) {
//                         int idx_reg = side_base + k * (map_h * map_w) + h * map_w + w;
//                         float v = reg[idx_reg];
//                         float e = std::exp(v);
//                         exps[k] = e;
//                         sum_exp += e;
//                     }
//                     if (sum_exp <= 0.f) sum_exp = 1e-6f;
//                     float loc = 0.f;
//                     for (int k = 0; k < 16; ++k) loc += exps[k] * k;
//                     loc /= sum_exp;
//                     regdfl[side] = loc;
//                 }

//                 float grid_x = (w + 0.5f);
//                 float grid_y = (h + 0.5f);
//                 float stride = static_cast<float>(strides[idx]);

//                 float x1 = (grid_x - regdfl[0]) * stride;
//                 float y1 = (grid_y - regdfl[1]) * stride;
//                 float x2 = (grid_x + regdfl[2]) * stride;
//                 float y2 = (grid_y + regdfl[3]) * stride;

//                 float xmin = x1 * scale_w;
//                 float ymin = y1 * scale_h;
//                 float xmax = x2 * scale_w;
//                 float ymax = y2 * scale_h;

//                 xmin = std::max(0.f, xmin);
//                 ymin = std::max(0.f, ymin);
//                 xmax = std::min(static_cast<float>(img_w), xmax);
//                 ymax = std::min(static_cast<float>(img_h), ymax);

//                 detectResult.emplace_back(0, conf, xmin, ymin, xmax, ymax);
//             }
//         }
//     }

//     return NMS(detectResult);
// }

// // ====================== PID控制器类 -----------------------
// struct PIDController {
//     double kp, ki, kd;
//     double out_min, out_max;
//     double target_zone_thresh;

//     double integral;
//     double prev_error;
//     double derivative;

//     PIDController(double kp_val, double ki_val, double kd_val,
//                   double min_val, double max_val, double zone)
//         : kp(kp_val), ki(ki_val), kd(kd_val),
//           out_min(min_val), out_max(max_val),
//           target_zone_thresh(zone),
//           integral(0.0), prev_error(0.0), derivative(0.0) {}

//     double update(double err, double dt) {
//         if (std::abs(err) < target_zone_thresh) {
//             integral = 0.0;
//             prev_error = 0.0;
//             derivative = 0.0;
//             return 0.0;
//         }
//         derivative = (err - prev_error) / dt;
//         integral += err * dt;
//         if (ki > 0) {
//             double max_integral = (out_max * 0.5) / ki;
//             integral = std::clamp(integral, -max_integral, max_integral);
//         } else {
//             integral = 0.0;
//         }
//         double out = kp * err + ki * integral + kd * derivative;
//         prev_error = err;
//         return std::clamp(out, out_min, out_max);
//     }

//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         derivative = 0.0;
//     }
// };

// static inline float ema_filter(float prev, float val, float alpha) {
//     return prev * (1.0f - alpha) + val * alpha;
// }

// // 平滑过渡函数：当位置变化过大时进行平滑
// static inline cv::Point2f smooth_transition(const cv::Point2f& prev_pos,
//                                            const cv::Point2f& new_pos,
//                                            float max_change) {
//     float dx = new_pos.x - prev_pos.x;
//     float dy = new_pos.y - prev_pos.y;
//     float dist = std::sqrt(dx*dx + dy*dy);
//     if (dist <= max_change) return new_pos;
//     float ratio = max_change / dist;
//     return cv::Point2f(
//         prev_pos.x + dx * ratio * SMOOTH_ALPHA,
//         prev_pos.y + dy * ratio * SMOOTH_ALPHA
//     );
// }

// // 计算两个矩形的 IoU
// static inline float rect_iou(const cv::Rect& a, const cv::Rect& b) {
//     int x1 = std::max(a.x, b.x);
//     int y1 = std::max(a.y, b.y);
//     int x2 = std::min(a.x + a.width,  b.x + b.width);
//     int y2 = std::min(a.y + a.height, b.y + b.height);
//     int w = std::max(0, x2 - x1);
//     int h = std::max(0, y2 - y1);
//     int inter = w * h;
//     int areaA = a.width * a.height;
//     int areaB = b.width * b.height;
//     return inter > 0 ? float(inter) / float(areaA + areaB - inter + 1e-6f) : 0.f;
// }

// // ====================== 线程：YOLO + KCF 协同 ======================
// void yolo_thread() {
//     int model_len = 0;
//     unsigned char* model_buf = load_model(RKNN_MODEL.c_str(), &model_len);
//     if (!model_buf) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Load model failed");
//         return;
//     }

//     rknn_context ctx;
//     if (rknn_init(&ctx, model_buf, model_len, RKNN_FLAG_PRIOR_MEDIUM, nullptr) != RKNN_SUCC) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "rknn_init failed");
//         free(model_buf);
//         return;
//     }
//     free(model_buf);

//     // 帧率统计
//     int frame_count = 0;
//     float fps = 0.0f;
//     auto start_time = std::chrono::steady_clock::now();
//     const int FPS_UPDATE_INTERVAL = 10;

//     cv::VideoCapture cap(0, cv::CAP_V4L2);
//     cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
//     cap.set(cv::CAP_PROP_FRAME_WIDTH,  INPUT_W);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, INPUT_H);
//     cap.set(cv::CAP_PROP_FPS, 60);
//     if (!cap.isOpened()) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Camera open failed");
//         rknn_destroy(ctx);
//         return;
//     }

//     cv::Mat frame, rgb, show;
//     const int OUT_NUM = 6;
//     rknn_output outputs[OUT_NUM] = {};

//     // 位置平滑
//     cv::Point2f last_smoothed_pos(CAM_CX, CAM_CY);

//     // 追踪器
//     cv::Ptr<cv::Tracker> tracker;
//     bool tracker_inited = false;
//     cv::Rect tracker_bbox;
//     int tracker_lost_count = 0;

//     auto reset_tracker = [&]() {
//         tracker.release();
//         tracker_inited = false;
//         tracker_lost_count = 0;
//     };

//     auto init_tracker_with = [&](const cv::Mat& img, const cv::Rect& box) {
//         // 注意：有的 OpenCV 版本需要 legacy 命名空间；若你的构建不支持下面这一行，把其改为 cv::legacy::TrackerKCF::create()
//         tracker = cv::TrackerKCF::create();
//         tracker->init(img, box);
//         tracker_inited = true;
//         tracker_bbox = box;
//         tracker_lost_count = 0;
//         if (!tracker_inited) {
//             RCLCPP_WARN(rclcpp::get_logger("yolo"), "Tracker init failed.");
//         }
//         return tracker_inited;
//     };

//     int last_yolo_frame_id = -1000;  // 上一次 YOLO 成功检测的帧号
//     float last_yolo_score = 0.0f;

//     while (true) {
//         cap >> frame;
//         if (frame.empty()) continue;
//         show = frame.clone();

//         // 帧率更新
//         frame_count++;
//         if (frame_count % FPS_UPDATE_INTERVAL == 0) {
//             auto end_time = std::chrono::steady_clock::now();
//             std::chrono::duration<float> elapsed = end_time - start_time;
//             fps = FPS_UPDATE_INTERVAL / elapsed.count();
//             start_time = end_time;
//         }

//         // 运行 YOLO 的判定
//         bool run_yolo_now = (frame_count % YOLO_INTERVAL == 0);

//         bool yolo_got = false;
//         cv::Rect yolo_bbox;
//         float yolo_score = 0.f;

//         if (run_yolo_now) {
//             // 预处理
//             cv::resize(frame, rgb, cv::Size(INPUT_W, INPUT_H));
//             cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);

//             // YOLO 推理
//             rknn_input inputs[1] = {};
//             inputs[0].index = 0;
//             inputs[0].type  = RKNN_TENSOR_UINT8;
//             inputs[0].fmt   = RKNN_TENSOR_NHWC;
//             inputs[0].buf   = rgb.data;
//             inputs[0].size  = rgb.total() * rgb.elemSize();

//             if (rknn_inputs_set(ctx, 1, inputs) == RKNN_SUCC) {
//                 if (rknn_run(ctx, nullptr) == RKNN_SUCC) {
//                     for (int i = 0; i < OUT_NUM; ++i) outputs[i].want_float = 1;
//                     if (rknn_outputs_get(ctx, OUT_NUM, outputs, nullptr) == RKNN_SUCC) {
//                         std::vector<rknn_output> out_vec(outputs, outputs + OUT_NUM);
//                         std::vector<DetectBox> keep = postprocess(out_vec, INPUT_H, INPUT_W);

//                         if (!keep.empty()) {
//                             // 取置信度最高的
//                             const auto& best = keep[0];
//                             int x1 = int(std::round(best.xmin));
//                             int y1 = int(std::round(best.ymin));
//                             int x2 = int(std::round(best.xmax));
//                             int y2 = int(std::round(best.ymax));
//                             yolo_bbox = cv::Rect(x1, y1, x2 - x1, y2 - y1);
//                             yolo_bbox &= cv::Rect(0,0,INPUT_W,INPUT_H);
//                             yolo_score = best.score;
//                             yolo_got = true;
//                             last_yolo_frame_id = frame_count;
//                             last_yolo_score = yolo_score;

//                             // 显示 YOLO 检测框（绿色）
//                             cv::rectangle(show, yolo_bbox, cv::Scalar(0,255,0), 2);
//                             cv::putText(show, cv::format("YOLO ball %.2f", yolo_score),
//                                         cv::Point(yolo_bbox.x, std::max(0, yolo_bbox.y - 5)),
//                                         cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);
//                         }
//                         rknn_outputs_release(ctx, OUT_NUM, outputs);
//                     }
//                 }
//             }
//         }

//         // Tracker 更新
//         bool tracker_ok = false;
//         if (tracker_inited) {
//             tracker_ok = tracker->update(frame, tracker_bbox);
//             if (!tracker_ok) {
//                 tracker_lost_count++;
//                 if (tracker_lost_count > TRACKER_MAX_LOSS) {
//                     reset_tracker();
//                 }
//             } else {
//                 tracker_lost_count = 0;
//                 // 画追踪框（蓝色）
//                 cv::rectangle(show, tracker_bbox, cv::Scalar(255,0,0), 2);
//                 cv::putText(show, "KCF", cv::Point(tracker_bbox.x, std::max(0, tracker_bbox.y - 5)),
//                             cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0), 2);
//             }
//         }

//         // 融合决策：优先 YOLO；当有 YOLO 时与 Tracker 一致性判断
//         TargetInfo current_target;
//         current_target.fps = fps;

//         if (yolo_got) {
//             // 若已有 Tracker，则检查 IoU；若低于阈值或 YOLO 置信度很高，则用 YOLO 重置 Tracker
//             bool need_reinit = (!tracker_inited) ||
//                                (rect_iou(tracker_bbox, yolo_bbox) < REINIT_IOU_THRESH) ||
//                                (yolo_score >= SCORE_REINIT_THRESH);

//             if (need_reinit) {
//                 init_tracker_with(frame, yolo_bbox);
//             }

//             // 用 YOLO 的中心点，并做位置平滑
//             cv::Point2f yolo_center(yolo_bbox.x + yolo_bbox.width * 0.5f,
//                                     yolo_bbox.y + yolo_bbox.height * 0.5f);
//             last_smoothed_pos = smooth_transition(last_smoothed_pos, yolo_center, POSITION_CHANGE_THRESH);

//             current_target.detected = true;
//             current_target.bbox     = yolo_bbox;
//             current_target.x        = last_smoothed_pos.x;
//             current_target.y        = last_smoothed_pos.y;
//             current_target.score    = yolo_score;
//             current_target.src      = TargetInfo::YOLO;

//         } else if (tracker_inited && tracker_ok) {
//             // 仅有追踪器成功
//             cv::Point2f trk_center(tracker_bbox.x + tracker_bbox.width * 0.5f,
//                                    tracker_bbox.y + tracker_bbox.height * 0.5f);
//             last_smoothed_pos = smooth_transition(last_smoothed_pos, trk_center, POSITION_CHANGE_THRESH);

//             current_target.detected = true;
//             current_target.bbox     = tracker_bbox;
//             current_target.x        = last_smoothed_pos.x;
//             current_target.y        = last_smoothed_pos.y;
//             current_target.score    = last_yolo_score * 0.9f; // 给个参考值
//             current_target.src      = TargetInfo::TRACKER;
//         } else {
//             // 都没有
//             current_target.detected = false;
//             current_target.src      = TargetInfo::NONE;
//             // 若长时间没有 YOLO/Tracker，重置
//             if (tracker_inited && tracker_lost_count > TRACKER_MAX_LOSS) {
//                 reset_tracker();
//             }
//         }

//         // 画中心点与目标区域
//         cv::circle(show, cv::Point((int)CAM_CX, (int)CAM_CY), (int)TARGET_ZONE_THRESH, cv::Scalar(0,255,255), 1);
//         if (current_target.detected) {
//             cv::circle(show, cv::Point((int)last_smoothed_pos.x, (int)last_smoothed_pos.y), 3, cv::Scalar(0,0,255), -1);
//             cv::putText(show,
//                         current_target.src == TargetInfo::YOLO ? "SRC: YOLO" : "SRC: KCF",
//                         cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
//         } else {
//             cv::putText(show, "SRC: NONE", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
//         }

//         // 显示帧率
//         cv::putText(show, cv::format("FPS: %.1f", fps),
//                     cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
//                     1.0, cv::Scalar(255, 0, 0), 2);

//         // 更新共享目标
//         {
//             std::lock_guard<std::mutex> lock(g_target_mutex);
//             g_target = current_target;
//         }

//         cv::imshow("YOLO+KCF", show);
//         cv::waitKey(1);
//     }

//     rknn_destroy(ctx);
// }

// // ====================== ROS 2 节点类：PID控制云台 ======================
// class GimbalYoloNode : public rclcpp::Node {
// public:
//     GimbalYoloNode()
//     : Node("gimbal_yolo_node"),
//       pid_pan_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
//       pid_tilt_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
//     {
//         // PID 参数
//         declare_parameter<double>("kp_pan",  0.011);
//         declare_parameter<double>("ki_pan",  0.003);
//         declare_parameter<double>("kd_pan",  0.0005);
//         declare_parameter<double>("kp_tilt", 0.011);
//         declare_parameter<double>("ki_tilt", 0.00005);
//         declare_parameter<double>("kd_tilt", 0.0002);

//         double kp_pan = get_parameter("kp_pan").as_double();
//         double ki_pan = get_parameter("ki_pan").as_double();
//         double kd_pan = get_parameter("kd_pan").as_double();
//         double kp_tilt = get_parameter("kp_tilt").as_double();
//         double ki_tilt = get_parameter("ki_tilt").as_double();
//         double kd_tilt = get_parameter("kd_tilt").as_double();

//         // 串口与电机
//         serial_ = std::make_shared<SerialPort>(SERIAL_DEV, SERIAL_BAUD);
//         dm_ = damiao::Motor_Control(serial_);

//         m_horizontal_ = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x06, 0x16);
//         m_vertical_   = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x05, 0x15);

//         dm_.addMotor(m_horizontal_.get());
//         dm_.addMotor(m_vertical_.get());

//         dm_.switchControlMode(*m_horizontal_, damiao::VEL_MODE);
//         dm_.switchControlMode(*m_vertical_,   damiao::VEL_MODE);

//         dm_.set_zero_position(*m_horizontal_);
//         dm_.set_zero_position(*m_vertical_);

//         dm_.enable(*m_horizontal_);
//         dm_.enable(*m_vertical_);

//         // PID 初始化
//         max_vel_ = 100.0;
//         pid_pan_  = PIDController(kp_pan,  ki_pan,  kd_pan,  -max_vel_, max_vel_, TARGET_ZONE_THRESH);
//         pid_tilt_ = PIDController(kp_tilt, ki_tilt, kd_tilt, -max_vel_, max_vel_, TARGET_ZONE_THRESH);

//         // 过滤参数
//         meas_smooth_alpha_ = 0.7f;
//         cmd_smooth_alpha_  = 0.5f;
//         deadzone_px_       = 10.0;

//         cmd_prev_pan_  = 0.0;
//         cmd_prev_tilt_ = 0.0;
//         filt_cx_ = CAM_CX;
//         filt_cy_ = CAM_CY;
//         last_seen_time_ = std::chrono::steady_clock::now();
//         last_control_time_ = std::chrono::steady_clock::now();

//         // 控制循环 50Hz
//         timer_ = create_wall_timer(std::chrono::milliseconds(10),
//                                    std::bind(&GimbalYoloNode::control_loop, this));
//         marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
//         RCLCPP_INFO(get_logger(), "Gimbal + YOLO+KCF control node started.");
//     }

// private:
//     void control_loop() {
//         auto now_tp = std::chrono::steady_clock::now();
//         std::chrono::duration<double> dt_duration = now_tp - last_control_time_;
//         double dt = dt_duration.count();
//         last_control_time_ = now_tp;

//         TargetInfo info;
//         {
//             std::lock_guard<std::mutex> lock(g_target_mutex);
//             info = g_target;
//         }

//         if (info.detected) {
//             last_seen_time_ = now_tp;

//             // 测量 EMA
//             filt_cx_ = ema_filter(filt_cx_, info.x, (float)meas_smooth_alpha_);
//             filt_cy_ = ema_filter(filt_cy_, info.y, (float)meas_smooth_alpha_);

//             // 像素误差（图像坐标）
//             double err_x = filt_cx_ - CAM_CX;
//             double err_y = filt_cy_ - CAM_CY;

//             // 死区
//             if (std::abs(err_x) < deadzone_px_) err_x = 0.0;
//             if (std::abs(err_y) < deadzone_px_) err_y = 0.0;

//             // PID
//             double cmd_pan  = pid_pan_.update(err_x, dt);
//             double cmd_tilt = pid_tilt_.update(err_y, dt);

//             // 命令 EMA
//             cmd_prev_pan_  = ema_filter((float)cmd_prev_pan_,  (float)cmd_pan,  (float)cmd_smooth_alpha_);
//             cmd_prev_tilt_ = ema_filter((float)cmd_prev_tilt_, (float)cmd_tilt, (float)cmd_smooth_alpha_);

//             // 方向（视需要调整）
//             dm_.control_vel(*m_horizontal_, -cmd_prev_pan_);
//             dm_.control_vel(*m_vertical_,    cmd_prev_tilt_);

//             publish_marker(err_x, err_y);
//             return;
//         }

//         // 未检测到：停止 + 重置
//         cmd_prev_pan_ = 0.0;
//         cmd_prev_tilt_ = 0.0;
//         dm_.control_vel(*m_horizontal_, 0.0);
//         dm_.control_vel(*m_vertical_,   0.0);
//         filt_cx_ = CAM_CX;
//         filt_cy_ = CAM_CY;
//         pid_pan_.reset();
//         pid_tilt_.reset();
//         publish_marker(NAN, NAN);
//     }

//     void publish_marker(double err_x, double err_y) {
//         visualization_msgs::msg::Marker m;
//         m.header.frame_id = "map";
//         m.header.stamp = now();
//         m.ns = "target";
//         m.id = 0;
//         m.type = m.SPHERE;
//         m.action = m.ADD;
//         if (std::isnan(err_x) || std::isnan(err_y)) {
//             m.pose.position.x = 0;
//             m.pose.position.y = 0;
//         } else {
//             m.pose.position.x = err_x;
//             m.pose.position.y = err_y;
//         }
//         m.pose.position.z = 0;
//         m.scale.x = m.scale.y = m.scale.z = 0.05;
//         m.color.a = 1.0;
//         m.color.g = 1.0;
//         marker_pub_->publish(m);
//     }

//     // 成员
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//     std::shared_ptr<SerialPort> serial_;
//     damiao::Motor_Control dm_;
//     std::shared_ptr<damiao::Motor> m_horizontal_, m_vertical_;

//     PIDController pid_pan_, pid_tilt_;
//     double max_vel_;
//     double cmd_prev_pan_, cmd_prev_tilt_;
//     double filt_cx_, filt_cy_;
//     double meas_smooth_alpha_, cmd_smooth_alpha_;
//     double deadzone_px_;
//     std::chrono::steady_clock::time_point last_seen_time_;
//     std::chrono::steady_clock::time_point last_control_time_;
// };

// // ====================== 主函数 ======================
// int main(int argc, char** argv) {
//     std::thread t(yolo_thread);
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<GimbalYoloNode>());
//     rclcpp::shutdown();
//     t.join();
//     return 0;
// }

//gpio
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "visualization_msgs/msg/marker.hpp"
// #include "damiao_ros2_control/damiao.h"
// #include "damiao_ros2_control/SerialPort.h"

// #include <opencv2/opencv.hpp>
// #include <opencv2/tracking.hpp>     // KCF
// #include <rknn_api.h>
// #include <mutex>
// #include <thread>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <chrono>
// #include <optional>
// #include <fcntl.h>
// #include <unistd.h>

// // ====================== 模型 & 输入参数配置 ======================
// const std::string RKNN_MODEL = "/home/orangepi/yolov11n_rknn/light-1.rknn";
// const int INPUT_W = 640, INPUT_H = 480;
// const float OBJ_THRESH = 0.4f;
// const float NMS_THRESH  = 0.45f;
// const std::vector<std::string> CLASSES = {"ball"};

// // ====================== 摄像头 & 控制参数配置 ======================
// const std::string SERIAL_DEV = "/dev/ttyACM0";
// const int SERIAL_BAUD = B921600;
// const double CAM_CX = INPUT_W  / 2.0;
// const double CAM_CY = INPUT_H / 2.0;

// // 目标区域阈值：在该范围内认为已对准（像素）
// const double TARGET_ZONE_THRESH = 10.0;  // 示例值，需根据实际调整
// // 平滑过渡系数
// const float SMOOTH_ALPHA = 0.4f;

// // ========== YOLO 与 Tracker 协同的调度/容错参数 ==========
// const int   YOLO_INTERVAL          = 3;     // 每隔几帧运行一次 YOLO
// const int   TRACKER_MAX_LOSS       = 15;    // 追踪器连续失败多少帧后判定丢失
// const float REINIT_IOU_THRESH      = 0.3f;  // YOLO 与 Tracker 框 IoU 过低则用 YOLO 重置
// const float SCORE_REINIT_THRESH    = 0.3f; // YOLO 置信度超过该阈值时优先以 YOLO 框为准
// const float POSITION_CHANGE_THRESH = 20.0f; // 目标位置突变平滑阈值（像素）

// // ====================== 全局共享变量（使用互斥锁保护） ======================
// struct TargetInfo {
//     bool detected = false;
//     float x = 0, y = 0;
//     cv::Rect bbox;     // 目标边界框
//     float score = 0.0f;
//     float fps = 0.0f;
//     enum Source { NONE=0, YOLO=1, TRACKER=2 } src = NONE;
// };
// TargetInfo g_target;
// std::mutex g_target_mutex;

// // ====================== 工具函数：加载RKNN模型 ======================
// static unsigned char* load_model(const char* fn, int* size) {
//     FILE* fp = fopen(fn, "rb");
//     if (!fp) return nullptr;
//     fseek(fp, 0, SEEK_END);
//     *size = ftell(fp);
//     unsigned char* buf = (unsigned char*)malloc(*size);
//     fseek(fp, 0, SEEK_SET);
//     fread(buf, 1, *size, fp);
//     fclose(fp);
//     return buf;
// }

// // ====================== 检测 & NMS ======================
// struct DetectBox {
//     int   classId;
//     float score;
//     float xmin, ymin, xmax, ymax;
//     DetectBox(int id, float s, float x1, float y1, float x2, float y2)
//         : classId(id), score(s), xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}
// };

// static float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

// static float IOU(float xmin1, float ymin1, float xmax1, float ymax1,
//                  float xmin2, float ymin2, float xmax2, float ymax2) {
//     float xmin = std::max(xmin1, xmin2);
//     float ymin = std::max(ymin1, ymin2);
//     float xmax = std::min(xmax1, xmax2);
//     float ymax = std::min(ymax1, ymax2);

//     float innerW = std::max(xmax - xmin, 0.f);
//     float innerH = std::max(ymax - ymin, 0.f);
//     float innerA = innerW * innerH;

//     float area1 = std::max(0.f, (xmax1 - xmin1)) * std::max(0.f, (ymax1 - ymin1));
//     float area2 = std::max(0.f, (xmax2 - xmin2)) * std::max(0.f, (ymax2 - ymin2));
//     return innerA / (area1 + area2 - innerA + 1e-6f);
// }

// static std::vector<DetectBox> NMS(const std::vector<DetectBox>& dets) {
//     if (dets.empty()) return {};
//     std::vector<cv::Rect> boxes;
//     std::vector<float>    scores;
//     for (const auto& b : dets) {
//         boxes.emplace_back(int(b.xmin), int(b.ymin), int(b.xmax - b.xmin), int(b.ymax - b.ymin));
//         scores.push_back(b.score);
//     }
//     std::vector<int> idxs(scores.size());
//     for (size_t i = 0; i < idxs.size(); ++i) idxs[i] = int(i);
//     std::sort(idxs.begin(), idxs.end(),
//               [&scores](int l, int r) { return scores[l] > scores[r]; });

//     std::vector<DetectBox> keep;
//     std::vector<bool>      suppressed(scores.size(), false);
//     for (size_t k = 0; k < idxs.size(); ++k) {
//         int i = idxs[k];
//         if (suppressed[i]) continue;
//         keep.push_back(dets[i]);
//         for (size_t t = k + 1; t < idxs.size(); ++t) {
//             int j = idxs[t];
//             if (IOU(boxes[i].x, boxes[i].y,
//                     boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height,
//                     boxes[j].x, boxes[j].y,
//                     boxes[j].x + boxes[j].width, boxes[j].y + boxes[j].height) > NMS_THRESH)
//                 suppressed[j] = true;
//         }
//     }
//     return keep;
// }

// static std::vector<DetectBox> postprocess(const std::vector<rknn_output>& out,
//                                           int img_h, int img_w) {
//     const int headNum = 3;
//     const std::vector<std::vector<int>> mapSize = {{60, 80}, {30, 40}, {15, 20}};
//     const std::vector<int> strides = {8, 16, 32};
//     const int class_num = 1;
//     const float objectThresh = OBJ_THRESH;

//     std::vector<std::vector<float>> outputs;
//     outputs.reserve(out.size());
//     for (const auto& o : out) {
//         size_t cnt = o.size / sizeof(float);
//         float* p = static_cast<float*>(o.buf);
//         outputs.emplace_back(p, p + cnt);
//     }

//     float scale_h = static_cast<float>(img_h) / INPUT_H;
//     float scale_w = static_cast<float>(img_w) / INPUT_W;

//     std::vector<DetectBox> detectResult;

//     for (int idx = 0; idx < headNum; ++idx) {
//         const auto& ms = mapSize[idx];
//         int map_h = ms[0];
//         int map_w = ms[1];
//         const std::vector<float>& reg = outputs[idx * 2 + 0];
//         const std::vector<float>& cls = outputs[idx * 2 + 1];

//         if ((int)cls.size() != class_num * map_h * map_w) {
//             RCLCPP_WARN(rclcpp::get_logger("yolo"), "cls size mismatch head=%d: got %zu expected %d", idx, cls.size(), class_num * map_h * map_w);
//             continue;
//         }
//         if ((int)reg.size() != 4 * 16 * map_h * map_w) {
//             RCLCPP_WARN(rclcpp::get_logger("yolo"), "reg size mismatch head=%d: got %zu expected %d", idx, reg.size(), 4 * 16 * map_h * map_w);
//             continue;
//         }

//         for (int h = 0; h < map_h; ++h) {
//             for (int w = 0; w < map_w; ++w) {
//                 int cls_index = 0 * map_h * map_w + h * map_w + w;
//                 float raw_conf = cls[cls_index];
//                 float conf = sigmoid(raw_conf);
//                 if (conf <= objectThresh) continue;

//                 float regdfl[4] = {0,0,0,0};
//                 for (int side = 0; side < 4; ++side) {
//                     int side_base = side * (16 * map_h * map_w);
//                     float sum_exp = 0.f;
//                     float exps[16];
//                     for (int k = 0; k < 16; ++k) {
//                         int idx_reg = side_base + k * (map_h * map_w) + h * map_w + w;
//                         float v = reg[idx_reg];
//                         float e = std::exp(v);
//                         exps[k] = e;
//                         sum_exp += e;
//                     }
//                     if (sum_exp <= 0.f) sum_exp = 1e-6f;
//                     float loc = 0.f;
//                     for (int k = 0; k < 16; ++k) loc += exps[k] * k;
//                     loc /= sum_exp;
//                     regdfl[side] = loc;
//                 }

//                 float grid_x = (w + 0.5f);
//                 float grid_y = (h + 0.5f);
//                 float stride = static_cast<float>(strides[idx]);

//                 float x1 = (grid_x - regdfl[0]) * stride;
//                 float y1 = (grid_y - regdfl[1]) * stride;
//                 float x2 = (grid_x + regdfl[2]) * stride;
//                 float y2 = (grid_y + regdfl[3]) * stride;

//                 float xmin = x1 * scale_w;
//                 float ymin = y1 * scale_h;
//                 float xmax = x2 * scale_w;
//                 float ymax = y2 * scale_h;

//                 xmin = std::max(0.f, xmin);
//                 ymin = std::max(0.f, ymin);
//                 xmax = std::min(static_cast<float>(img_w), xmax);
//                 ymax = std::min(static_cast<float>(img_h), ymax);

//                 detectResult.emplace_back(0, conf, xmin, ymin, xmax, ymax);
//             }
//         }
//     }

//     return NMS(detectResult);
// }

// // ====================== PID控制器类 -----------------------
// struct PIDController {
//     double kp, ki, kd;
//     double out_min, out_max;
//     double target_zone_thresh;

//     double integral;
//     double prev_error;
//     double derivative;

//     PIDController(double kp_val, double ki_val, double kd_val,
//                   double min_val, double max_val, double zone)
//         : kp(kp_val), ki(ki_val), kd(kd_val),
//           out_min(min_val), out_max(max_val),
//           target_zone_thresh(zone),
//           integral(0.0), prev_error(0.0), derivative(0.0) {}

//     double update(double err, double dt) {
//         if (std::abs(err) < target_zone_thresh) {
//             integral = 0.0;
//             prev_error = 0.0;
//             derivative = 0.0;
//             return 0.0;
//         }
//         derivative = (err - prev_error) / dt;
//         integral += err * dt;
//         if (ki > 0) {
//             double max_integral = (out_max * 0.5) / ki;
//             integral = std::clamp(integral, -max_integral, max_integral);
//         } else {
//             integral = 0.0;
//         }
//         double out = kp * err + ki * integral + kd * derivative;
//         prev_error = err;
//         return std::clamp(out, out_min, out_max);
//     }

//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         derivative = 0.0;
//     }
// };

// static inline float ema_filter(float prev, float val, float alpha) {
//     return prev * (1.0f - alpha) + val * alpha;
// }

// // 平滑过渡函数：当位置变化过大时进行平滑
// static inline cv::Point2f smooth_transition(const cv::Point2f& prev_pos,
//                                            const cv::Point2f& new_pos,
//                                            float max_change) {
//     float dx = new_pos.x - prev_pos.x;
//     float dy = new_pos.y - prev_pos.y;
//     float dist = std::sqrt(dx*dx + dy*dy);
//     if (dist <= max_change) return new_pos;
//     float ratio = max_change / dist;
//     return cv::Point2f(
//         prev_pos.x + dx * ratio * SMOOTH_ALPHA,
//         prev_pos.y + dy * ratio * SMOOTH_ALPHA
//     );
// }

// // 计算两个矩形的 IoU
// static inline float rect_iou(const cv::Rect& a, const cv::Rect& b) {
//     int x1 = std::max(a.x, b.x);
//     int y1 = std::max(a.y, b.y);
//     int x2 = std::min(a.x + a.width,  b.x + b.width);
//     int y2 = std::min(a.y + a.height, b.y + b.height);
//     int w = std::max(0, x2 - x1);
//     int h = std::max(0, y2 - y1);
//     int inter = w * h;
//     int areaA = a.width * a.height;
//     int areaB = b.width * b.height;
//     return inter > 0 ? float(inter) / float(areaA + areaB - inter + 1e-6f) : 0.f;
// }

// // ====================== 线程：YOLO + KCF 协同 ======================
// void yolo_thread() {
//     int model_len = 0;
//     unsigned char* model_buf = load_model(RKNN_MODEL.c_str(), &model_len);
//     if (!model_buf) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Load model failed");
//         return;
//     }

//     rknn_context ctx;
//     if (rknn_init(&ctx, model_buf, model_len, RKNN_FLAG_PRIOR_MEDIUM, nullptr) != RKNN_SUCC) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "rknn_init failed");
//         free(model_buf);
//         return;
//     }
//     free(model_buf);

//     // 帧率统计
//     int frame_count = 0;
//     float fps = 0.0f;
//     auto start_time = std::chrono::steady_clock::now();
//     const int FPS_UPDATE_INTERVAL = 10;

//     cv::VideoCapture cap(0, cv::CAP_V4L2);
//     cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
//     cap.set(cv::CAP_PROP_FRAME_WIDTH,  INPUT_W);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, INPUT_H);
//     cap.set(cv::CAP_PROP_FPS, 60);
//     if (!cap.isOpened()) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Camera open failed");
//         rknn_destroy(ctx);
//         return;
//     }

//     cv::Mat frame, rgb, show;
//     const int OUT_NUM = 6;
//     rknn_output outputs[OUT_NUM] = {};

//     // 位置平滑
//     cv::Point2f last_smoothed_pos(CAM_CX, CAM_CY);

//     // 追踪器
//     cv::Ptr<cv::Tracker> tracker;
//     bool tracker_inited = false;
//     cv::Rect tracker_bbox;
//     int tracker_lost_count = 0;

//     auto reset_tracker = [&]() {
//         tracker.release();
//         tracker_inited = false;
//         tracker_lost_count = 0;
//     };

//     auto init_tracker_with = [&](const cv::Mat& img, const cv::Rect& box) {
//         // 注意：部分OpenCV版本需用 legacy 命名空间
//         tracker = cv::TrackerKCF::create();
//         tracker->init(img, box);
//         tracker_inited = true;
//         tracker_bbox = box;
//         tracker_lost_count = 0;
//         if (!tracker_inited) {
//             RCLCPP_WARN(rclcpp::get_logger("yolo"), "Tracker init failed.");
//         }
//         return tracker_inited;
//     };

//     int last_yolo_frame_id = -1000;  // 上一次 YOLO 成功检测的帧号
//     float last_yolo_score = 0.0f;

//     while (true) {
//         cap >> frame;
//         if (frame.empty()) continue;
//         show = frame.clone();

//         // 帧率更新
//         frame_count++;
//         if (frame_count % FPS_UPDATE_INTERVAL == 0) {
//             auto end_time = std::chrono::steady_clock::now();
//             std::chrono::duration<float> elapsed = end_time - start_time;
//             fps = FPS_UPDATE_INTERVAL / elapsed.count();
//             start_time = end_time;
//         }

//         // 运行 YOLO 的判定
//         bool run_yolo_now = (frame_count % YOLO_INTERVAL == 0);

//         bool yolo_got = false;
//         cv::Rect yolo_bbox;
//         float yolo_score = 0.f;

//         if (run_yolo_now) {
//             // 预处理
//             cv::resize(frame, rgb, cv::Size(INPUT_W, INPUT_H));
//             cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);

//             // YOLO 推理
//             rknn_input inputs[1] = {};
//             inputs[0].index = 0;
//             inputs[0].type  = RKNN_TENSOR_UINT8;
//             inputs[0].fmt   = RKNN_TENSOR_NHWC;
//             inputs[0].buf   = rgb.data;
//             inputs[0].size  = rgb.total() * rgb.elemSize();

//             if (rknn_inputs_set(ctx, 1, inputs) == RKNN_SUCC) {
//                 if (rknn_run(ctx, nullptr) == RKNN_SUCC) {
//                     for (int i = 0; i < OUT_NUM; ++i) outputs[i].want_float = 1;
//                     if (rknn_outputs_get(ctx, OUT_NUM, outputs, nullptr) == RKNN_SUCC) {
//                         std::vector<rknn_output> out_vec(outputs, outputs + OUT_NUM);
//                         std::vector<DetectBox> keep = postprocess(out_vec, INPUT_H, INPUT_W);

//                         if (!keep.empty()) {
//                             // 取置信度最高的
//                             const auto& best = keep[0];
//                             int x1 = int(std::round(best.xmin));
//                             int y1 = int(std::round(best.ymin));
//                             int x2 = int(std::round(best.xmax));
//                             int y2 = int(std::round(best.ymax));
//                             yolo_bbox = cv::Rect(x1, y1, x2 - x1, y2 - y1);
//                             yolo_bbox &= cv::Rect(0,0,INPUT_W,INPUT_H);
//                             yolo_score = best.score;
//                             yolo_got = true;
//                             last_yolo_frame_id = frame_count;
//                             last_yolo_score = yolo_score;

//                             // 显示 YOLO 检测框（绿色）
//                             cv::rectangle(show, yolo_bbox, cv::Scalar(0,255,0), 2);
//                             cv::putText(show, cv::format("YOLO ball %.2f", yolo_score),
//                                         cv::Point(yolo_bbox.x, std::max(0, yolo_bbox.y - 5)),
//                                         cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);
//                         }
//                         rknn_outputs_release(ctx, OUT_NUM, outputs);
//                     }
//                 }
//             }
//         }

//         // Tracker 更新
//         bool tracker_ok = false;
//         if (tracker_inited) {
//             tracker_ok = tracker->update(frame, tracker_bbox);
//             if (!tracker_ok) {
//                 tracker_lost_count++;
//                 if (tracker_lost_count > TRACKER_MAX_LOSS) {
//                     reset_tracker();
//                 }
//             } else {
//                 tracker_lost_count = 0;
//                 // 画追踪框（蓝色）
//                 cv::rectangle(show, tracker_bbox, cv::Scalar(255,0,0), 2);
//                 cv::putText(show, "KCF", cv::Point(tracker_bbox.x, std::max(0, tracker_bbox.y - 5)),
//                             cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0), 2);
//             }
//         }

//         // 融合决策：优先 YOLO；当有 YOLO 时与 Tracker 一致性判断
//         TargetInfo current_target;
//         current_target.fps = fps;

//         if (yolo_got) {
//             // 若已有 Tracker，则检查 IoU；若低于阈值或 YOLO 置信度很高，则用 YOLO 重置 Tracker
//             bool need_reinit = (!tracker_inited) ||
//                                (rect_iou(tracker_bbox, yolo_bbox) < REINIT_IOU_THRESH) ||
//                                (yolo_score >= SCORE_REINIT_THRESH);

//             if (need_reinit) {
//                 init_tracker_with(frame, yolo_bbox);
//             }

//             // 用 YOLO 的中心点，并做位置平滑
//             cv::Point2f yolo_center(yolo_bbox.x + yolo_bbox.width * 0.5f,
//                                     yolo_bbox.y + yolo_bbox.height * 0.5f);
//             last_smoothed_pos = smooth_transition(last_smoothed_pos, yolo_center, POSITION_CHANGE_THRESH);

//             current_target.detected = true;
//             current_target.bbox     = yolo_bbox;
//             current_target.x        = last_smoothed_pos.x;
//             current_target.y        = last_smoothed_pos.y;
//             current_target.score    = yolo_score;
//             current_target.src      = TargetInfo::YOLO;

//         } else if (tracker_inited && tracker_ok) {
//             // 仅有追踪器成功
//             cv::Point2f trk_center(tracker_bbox.x + tracker_bbox.width * 0.5f,
//                                    tracker_bbox.y + tracker_bbox.height * 0.5f);
//             last_smoothed_pos = smooth_transition(last_smoothed_pos, trk_center, POSITION_CHANGE_THRESH);

//             current_target.detected = true;
//             current_target.bbox     = tracker_bbox;
//             current_target.x        = last_smoothed_pos.x;
//             current_target.y        = last_smoothed_pos.y;
//             current_target.score    = last_yolo_score * 0.9f; // 给个参考值
//             current_target.src      = TargetInfo::TRACKER;
//         } else {
//             // 都没有
//             current_target.detected = false;
//             current_target.src      = TargetInfo::NONE;
//             // 若长时间没有 YOLO/Tracker，重置
//             if (tracker_inited && tracker_lost_count > TRACKER_MAX_LOSS) {
//                 reset_tracker();
//             }
//         }

//         // 画中心点与目标区域
//         cv::circle(show, cv::Point((int)CAM_CX, (int)CAM_CY), (int)TARGET_ZONE_THRESH, cv::Scalar(0,255,255), 1);
//         if (current_target.detected) {
//             cv::circle(show, cv::Point((int)last_smoothed_pos.x, (int)last_smoothed_pos.y), 3, cv::Scalar(0,0,255), -1);
//             cv::putText(show,
//                         current_target.src == TargetInfo::YOLO ? "SRC: YOLO" : "SRC: KCF",
//                         cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
//         } else {
//             cv::putText(show, "SRC: NONE", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
//         }

//         // 显示帧率
//         cv::putText(show, cv::format("FPS: %.1f", fps),
//                     cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
//                     1.0, cv::Scalar(255, 0, 0), 2);

//         // 更新共享目标
//         {
//             std::lock_guard<std::mutex> lock(g_target_mutex);
//             g_target = current_target;
//         }

//         cv::imshow("YOLO+KCF", show);
//         cv::waitKey(1);
//     }

//     rknn_destroy(ctx);
// }

// // ====================== ROS 2 节点类：PID控制云台 + GPIO控制 ======================
// class GimbalYoloNode : public rclcpp::Node {
// public:
//     GimbalYoloNode()
//     : Node("gimbal_yolo_node"),
//       pid_pan_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
//       pid_tilt_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
//       GPIO3_B5(41), GPIO1_A2(34),  // GPIO编号
//       gpio_exported_(false),
//       in_target_zone_(false)
//     {
//         // PID 参数
//         declare_parameter<double>("kp_pan",  0.011);
//         declare_parameter<double>("ki_pan",  0.003);
//         declare_parameter<double>("kd_pan",  0.0005);
//         declare_parameter<double>("kp_tilt", 0.011);
//         declare_parameter<double>("ki_tilt", 0.00005);
//         declare_parameter<double>("kd_tilt", 0.0002);

//         double kp_pan = get_parameter("kp_pan").as_double();
//         double ki_pan = get_parameter("ki_pan").as_double();
//         double kd_pan = get_parameter("kd_pan").as_double();
//         double kp_tilt = get_parameter("kp_tilt").as_double();
//         double ki_tilt = get_parameter("ki_tilt").as_double();
//         double kd_tilt = get_parameter("kd_tilt").as_double();

//         // 串口与电机
//         serial_ = std::make_shared<SerialPort>(SERIAL_DEV, SERIAL_BAUD);
//         dm_ = damiao::Motor_Control(serial_);

//         m_horizontal_ = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x06, 0x16);
//         m_vertical_   = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x05, 0x15);

//         dm_.addMotor(m_horizontal_.get());
//         dm_.addMotor(m_vertical_.get());

//         dm_.switchControlMode(*m_horizontal_, damiao::VEL_MODE);
//         dm_.switchControlMode(*m_vertical_,   damiao::VEL_MODE);

//         dm_.set_zero_position(*m_horizontal_);
//         dm_.set_zero_position(*m_vertical_);

//         dm_.enable(*m_horizontal_);
//         dm_.enable(*m_vertical_);

//         // PID 初始化
//         max_vel_ = 100.0;
//         pid_pan_  = PIDController(kp_pan,  ki_pan,  kd_pan,  -max_vel_, max_vel_, TARGET_ZONE_THRESH);
//         pid_tilt_ = PIDController(kp_tilt, ki_tilt, kd_tilt, -max_vel_, max_vel_, TARGET_ZONE_THRESH);

//         // 过滤参数
//         meas_smooth_alpha_ = 0.7f;
//         cmd_smooth_alpha_  = 0.5f;
//         deadzone_px_       = 10.0;

//         cmd_prev_pan_  = 0.0;
//         cmd_prev_tilt_ = 0.0;
//         filt_cx_ = CAM_CX;
//         filt_cy_ = CAM_CY;
//         last_seen_time_ = std::chrono::steady_clock::now();
//         last_control_time_ = std::chrono::steady_clock::now();

//         // GPIO 初始化
//         initGpio();

//         // 控制循环 50Hz
//         timer_ = create_wall_timer(std::chrono::milliseconds(10),
//                                    std::bind(&GimbalYoloNode::control_loop, this));
//         marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
//         RCLCPP_INFO(get_logger(), "Gimbal + YOLO+KCF + GPIO control node started.");
//     }

//     ~GimbalYoloNode() {
//         releaseGpio();
//     }

// private:
//     // GPIO控制逻辑
//     const int GPIO3_B5, GPIO1_A2;
//     bool gpio_exported_;
//     bool in_target_zone_;
//     std::chrono::steady_clock::time_point zone_entry_time_;

//     bool exportGpio(int gpio) {
//         int fd = open("/sys/class/gpio/export", O_WRONLY);
//         if (fd < 0) {
//             RCLCPP_ERROR(get_logger(), "Open export failed for GPIO %d", gpio);
//             return false;
//         }
//         char buf[10];
//         snprintf(buf, sizeof(buf), "%d", gpio);
//         if (write(fd, buf, strlen(buf)) < 0) {
//             RCLCPP_ERROR(get_logger(), "Export GPIO %d failed", gpio);
//             close(fd);
//             return false;
//         }
//         close(fd);
//         return true;
//     }

//     bool setGpioDirection(int gpio, const std::string& dir) {
//         char path[64];
//         snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
//         int fd = open(path, O_WRONLY);
//         if (fd < 0) {
//             RCLCPP_ERROR(get_logger(), "Open direction failed for GPIO %d", gpio);
//             return false;
//         }
//         if (write(fd, dir.c_str(), dir.size()) < 0) {
//             RCLCPP_ERROR(get_logger(), "Set direction for GPIO %d failed", gpio);
//             close(fd);
//             return false;
//         }
//         close(fd);
//         return true;
//     }

//     bool writeGpio(int gpio, int value) {
//         char path[64];
//         snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
//         int fd = open(path, O_WRONLY);
//         if (fd < 0) {
//             RCLCPP_ERROR(get_logger(), "Open value failed for GPIO %d", gpio);
//             return false;
//         }
//         const char* val_str = value ? "1" : "0";
//         if (write(fd, val_str, strlen(val_str)) < 0) {
//             RCLCPP_ERROR(get_logger(), "Write value for GPIO %d failed", gpio);
//             close(fd);
//             return false;
//         }
//         close(fd);
//         return true;
//     }

//     void initGpio() {
//         if (exportGpio(GPIO3_B5) && exportGpio(GPIO1_A2)) {
//             if (setGpioDirection(GPIO3_B5, "out") && setGpioDirection(GPIO1_A2, "out")) {
//                 // 初始状态：B1高(1)，A2低(0)
//                 writeGpio(GPIO3_B5, 1);
//                 writeGpio(GPIO1_A2, 0);
//                 gpio_exported_ = true;
//             }
//         }
//     }

//     void releaseGpio() {
//         if (gpio_exported_) {
//             // 恢复初始状态
//             writeGpio(GPIO3_B5, 1);
//             writeGpio(GPIO1_A2, 0);
//             // 取消导出
//             int fd = open("/sys/class/gpio/unexport", O_WRONLY);
//             if (fd >= 0) {
//                 char buf[10];
//                 snprintf(buf, sizeof(buf), "%d", GPIO3_B5);
//                 write(fd, buf, strlen(buf));
//                 snprintf(buf, sizeof(buf), "%d", GPIO1_A2);
//                 write(fd, buf, strlen(buf));
//                 close(fd);
//             }
//         }
//     }

//     void control_loop() {
//         auto now_tp = std::chrono::steady_clock::now();
//         std::chrono::duration<double> dt_duration = now_tp - last_control_time_;
//         double dt = dt_duration.count();
//         last_control_time_ = now_tp;

//         TargetInfo info;
//         {
//             std::lock_guard<std::mutex> lock(g_target_mutex);
//             info = g_target;
//         }

//         if (info.detected) {
//             last_seen_time_ = now_tp;

//             // 测量 EMA
//             filt_cx_ = ema_filter(filt_cx_, info.x, (float)meas_smooth_alpha_);
//             filt_cy_ = ema_filter(filt_cy_, info.y, (float)meas_smooth_alpha_);

//             // 像素误差（图像坐标）
//             double err_x = filt_cx_ - CAM_CX;
//             double err_y = filt_cy_ - CAM_CY;

//             // 判断是否在目标区域内（误差 < 阈值）
//             bool in_zone = (std::abs(err_x) < TARGET_ZONE_THRESH) && (std::abs(err_y) < TARGET_ZONE_THRESH);

//             if (in_zone) {
//                 if (!in_target_zone_) {
//                     // 刚进入目标区域，记录时间
//                     in_target_zone_ = true;
//                     zone_entry_time_ = now_tp;
//                 } else {
//                     // 已在区域内，检查持续时间是否≥2.5秒
//                     std::chrono::duration<double> duration = now_tp - zone_entry_time_;
//                     if (duration.count() >= 2.5) {
//                         // 触发GPIO：B1低(0)，A2高(1)
//                         if (gpio_exported_) {
//                             writeGpio(GPIO3_B5, 0);
//                             writeGpio(GPIO1_A2, 1);
//                         }
//                     }
//                 }
//             } else {
//                 // 离开目标区域，重置状态
//                 in_target_zone_ = false;
//                 if (gpio_exported_) {
//                     writeGpio(GPIO3_B5, 1);  // 恢复高电平
//                     writeGpio(GPIO1_A2, 0);  // 恢复低电平
//                 }
//             }

//             // 死区处理
//             if (std::abs(err_x) < deadzone_px_) err_x = 0.0;
//             if (std::abs(err_y) < deadzone_px_) err_y = 0.0;

//             // PID计算
//             double cmd_pan  = pid_pan_.update(err_x, dt);
//             double cmd_tilt = pid_tilt_.update(err_y, dt);

//             // 命令平滑
//             cmd_prev_pan_  = ema_filter((float)cmd_prev_pan_,  (float)cmd_pan,  (float)cmd_smooth_alpha_);
//             cmd_prev_tilt_ = ema_filter((float)cmd_prev_tilt_, (float)cmd_tilt, (float)cmd_smooth_alpha_);

//             // 电机控制（方向根据实际机械结构调整）
//             dm_.control_vel(*m_horizontal_, -cmd_prev_pan_);
//             dm_.control_vel(*m_vertical_,    cmd_prev_tilt_);

//             publish_marker(err_x, err_y);
//             return;
//         }

//         // 未检测到目标：重置状态
//         in_target_zone_ = false;
//         if (gpio_exported_) {
//             writeGpio(GPIO3_B5, 1);
//             writeGpio(GPIO1_A2, 0);
//         }
//         cmd_prev_pan_ = 0.0;
//         cmd_prev_tilt_ = 0.0;
//         dm_.control_vel(*m_horizontal_, 0.0);
//         dm_.control_vel(*m_vertical_,   0.0);
//         filt_cx_ = CAM_CX;
//         filt_cy_ = CAM_CY;
//         pid_pan_.reset();
//         pid_tilt_.reset();
//         publish_marker(NAN, NAN);
//     }

//     void publish_marker(double err_x, double err_y) {
//         visualization_msgs::msg::Marker m;
//         m.header.frame_id = "map";
//         m.header.stamp = now();
//         m.ns = "target";
//         m.id = 0;
//         m.type = m.SPHERE;
//         m.action = m.ADD;
//         if (std::isnan(err_x) || std::isnan(err_y)) {
//             m.pose.position.x = 0;
//             m.pose.position.y = 0;
//         } else {
//             m.pose.position.x = err_x;
//             m.pose.position.y = err_y;
//         }
//         m.pose.position.z = 0;
//         m.scale.x = m.scale.y = m.scale.z = 0.05;
//         m.color.a = 1.0;
//         m.color.g = 1.0;
//         marker_pub_->publish(m);
//     }

//     // 成员变量
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//     std::shared_ptr<SerialPort> serial_;
//     damiao::Motor_Control dm_;
//     std::shared_ptr<damiao::Motor> m_horizontal_, m_vertical_;

//     PIDController pid_pan_, pid_tilt_;
//     double max_vel_;
//     double cmd_prev_pan_, cmd_prev_tilt_;
//     double filt_cx_, filt_cy_;
//     double meas_smooth_alpha_, cmd_smooth_alpha_;
//     double deadzone_px_;
//     std::chrono::steady_clock::time_point last_seen_time_;
//     std::chrono::steady_clock::time_point last_control_time_;
// };

// // ====================== 主函数 ======================
// int main(int argc, char** argv) {
//     std::thread t(yolo_thread);
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<GimbalYoloNode>());
//     rclcpp::shutdown();
//     t.join();
//     return 0;
// }

//检测标志位计时+视频实时显示

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "visualization_msgs/msg/marker.hpp"
// #include "damiao_ros2_control/damiao.h"
// #include "damiao_ros2_control/SerialPort.h"
// #include <opencv2/opencv.hpp>
// #include <opencv2/tracking.hpp>
// #include <rknn_api.h>
// #include <mutex>
// #include <thread>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <chrono>
// #include <optional>
// #include <fcntl.h>
// #include <unistd.h>
// #include <termios.h>

// // ====================== 模型 & 输入参数配置 ======================
// const std::string RKNN_MODEL = "/home/orangepi/yolov11n_rknn/light-1.rknn";
// const int INPUT_W = 640, INPUT_H = 480;
// const float OBJ_THRESH = 0.4f;
// const float NMS_THRESH  = 0.45f;
// const std::vector<std::string> CLASSES = {"ball"};

// // ====================== 摄像头 & 控制参数配置 ======================
// const std::string SERIAL_DEV = "/dev/ttyACM0";
// const int SERIAL_BAUD = B921600;
// const double CAM_CX = INPUT_W  / 2.0;
// const double CAM_CY = INPUT_H / 2.0;
// const double TARGET_ZONE_THRESH = 10.0;  // 像素
// const float SMOOTH_ALPHA = 0.4f;

// // ========== YOLO 与 Tracker 协同参数 ==========
// const int   YOLO_INTERVAL          = 3;
// const int   TRACKER_MAX_LOSS       = 15;
// const float REINIT_IOU_THRESH      = 0.3f;
// const float SCORE_REINIT_THRESH    = 0.3f;
// const float POSITION_CHANGE_THRESH = 20.0f;

// // ====================== 新增：/dev/ttyS3 串口 ======================
// const std::string UART3_DEV   = "/dev/ttyS3";
// const int         UART3_BAUD  = 115200;
// const int         READY_FRAME_LEN = 3;
// const uint8_t     READY_FRAME[READY_FRAME_LEN] = {0xAA, 0x01, 0x55};

// // ====================== 全局共享变量 ======================
// struct TargetInfo {
//     bool detected = false;
//     float x = 0, y = 0;
//     cv::Rect bbox;
//     float score = 0.0f;
//     float fps = 0.0f;
//     enum Source { NONE=0, YOLO=1, TRACKER=2 } src = NONE;
// };
// TargetInfo g_target;
// std::mutex g_target_mutex;

// // ====================== 工具函数：加载RKNN模型 ======================
// static unsigned char* load_model(const char* fn, int* size) {
//     FILE* fp = fopen(fn, "rb");
//     if (!fp) return nullptr;
//     fseek(fp, 0, SEEK_END);
//     *size = ftell(fp);
//     unsigned char* buf = (unsigned char*)malloc(*size);
//     fseek(fp, 0, SEEK_SET);
//     fread(buf, 1, *size, fp);
//     fclose(fp);
//     return buf;
// }

// // ====================== 检测 & NMS ======================
// struct DetectBox {
//     int   classId;
//     float score;
//     float xmin, ymin, xmax, ymax;
//     DetectBox(int id, float s, float x1, float y1, float x2, float y2)
//         : classId(id), score(s), xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}
// };

// static float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

// static float IOU(float xmin1, float ymin1, float xmax1, float ymax1,
//                  float xmin2, float ymin2, float xmax2, float ymax2) {
//     float xmin = std::max(xmin1, xmin2);
//     float ymin = std::max(ymin1, ymin2);
//     float xmax = std::min(xmax1, xmax2);
//     float ymax = std::min(ymax1, ymax2);

//     float innerW = std::max(xmax - xmin, 0.f);
//     float innerH = std::max(ymax - ymin, 0.f);
//     float innerA = innerW * innerH;

//     float area1 = std::max(0.f, (xmax1 - xmin1)) * std::max(0.f, (ymax1 - ymin1));
//     float area2 = std::max(0.f, (xmax2 - xmin2)) * std::max(0.f, (ymax2 - ymin2));
//     return innerA / (area1 + area2 - innerA + 1e-6f);
// }

// static std::vector<DetectBox> NMS(const std::vector<DetectBox>& dets) {
//     if (dets.empty()) return {};
//     std::vector<cv::Rect> boxes;
//     std::vector<float>    scores;
//     for (const auto& b : dets) {
//         boxes.emplace_back(int(b.xmin), int(b.ymin), int(b.xmax - b.xmin), int(b.ymax - b.ymin));
//         scores.push_back(b.score);
//     }
//     std::vector<int> idxs(scores.size());
//     for (size_t i = 0; i < idxs.size(); ++i) idxs[i] = int(i);
//     std::sort(idxs.begin(), idxs.end(),
//               [&scores](int l, int r) { return scores[l] > scores[r]; });

//     std::vector<DetectBox> keep;
//     std::vector<bool>      suppressed(scores.size(), false);
//     for (size_t k = 0; k < idxs.size(); ++k) {
//         int i = idxs[k];
//         if (suppressed[i]) continue;
//         keep.push_back(dets[i]);
//         for (size_t t = k + 1; t < idxs.size(); ++t) {
//             int j = idxs[t];
//             if (IOU(boxes[i].x, boxes[i].y,
//                     boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height,
//                     boxes[j].x, boxes[j].y,
//                     boxes[j].x + boxes[j].width, boxes[j].y + boxes[j].height) > NMS_THRESH)
//                 suppressed[j] = true;
//         }
//     }
//     return keep;
// }

// static std::vector<DetectBox> postprocess(const std::vector<rknn_output>& out,
//                                           int img_h, int img_w) {
//     const int headNum = 3;
//     const std::vector<std::vector<int>> mapSize = {{60, 80}, {30, 40}, {15, 20}};
//     const std::vector<int> strides = {8, 16, 32};
//     const int class_num = 1;
//     const float objectThresh = OBJ_THRESH;

//     std::vector<std::vector<float>> outputs;
//     outputs.reserve(out.size());
//     for (const auto& o : out) {
//         size_t cnt = o.size / sizeof(float);
//         float* p = static_cast<float*>(o.buf);
//         outputs.emplace_back(p, p + cnt);
//     }

//     float scale_h = static_cast<float>(img_h) / INPUT_H;
//     float scale_w = static_cast<float>(img_w) / INPUT_W;

//     std::vector<DetectBox> detectResult;

//     for (int idx = 0; idx < headNum; ++idx) {
//         const auto& ms = mapSize[idx];
//         int map_h = ms[0];
//         int map_w = ms[1];
//         const std::vector<float>& reg = outputs[idx * 2 + 0];
//         const std::vector<float>& cls = outputs[idx * 2 + 1];

//         if ((int)cls.size() != class_num * map_h * map_w) continue;
//         if ((int)reg.size() != 4 * 16 * map_h * map_w) continue;

//         for (int h = 0; h < map_h; ++h) {
//             for (int w = 0; w < map_w; ++w) {
//                 int cls_index = 0 * map_h * map_w + h * map_w + w;
//                 float raw_conf = cls[cls_index];
//                 float conf = sigmoid(raw_conf);
//                 if (conf <= objectThresh) continue;

//                 float regdfl[4] = {0,0,0,0};
//                 for (int side = 0; side < 4; ++side) {
//                     int side_base = side * (16 * map_h * map_w);
//                     float sum_exp = 0.f;
//                     float exps[16];
//                     for (int k = 0; k < 16; ++k) {
//                         int idx_reg = side_base + k * (map_h * map_w) + h * map_w + w;
//                         float v = reg[idx_reg];
//                         float e = std::exp(v);
//                         exps[k] = e;
//                         sum_exp += e;
//                     }
//                     if (sum_exp <= 0.f) sum_exp = 1e-6f;
//                     float loc = 0.f;
//                     for (int k = 0; k < 16; ++k) loc += exps[k] * k;
//                     loc /= sum_exp;
//                     regdfl[side] = loc;
//                 }

//                 float grid_x = (w + 0.5f);
//                 float grid_y = (h + 0.5f);
//                 float stride = static_cast<float>(strides[idx]);

//                 float x1 = (grid_x - regdfl[0]) * stride;
//                 float y1 = (grid_y - regdfl[1]) * stride;
//                 float x2 = (grid_x + regdfl[2]) * stride;
//                 float y2 = (grid_y + regdfl[3]) * stride;

//                 float xmin = x1 * scale_w;
//                 float ymin = y1 * scale_h;
//                 float xmax = x2 * scale_w;
//                 float ymax = y2 * scale_h;

//                 xmin = std::max(0.f, xmin);
//                 ymin = std::max(0.f, ymin);
//                 xmax = std::min(static_cast<float>(img_w), xmax);
//                 ymax = std::min(static_cast<float>(img_h), ymax);

//                 detectResult.emplace_back(0, conf, xmin, ymin, xmax, ymax);
//             }
//         }
//     }
//     return NMS(detectResult);
// }

// // ====================== PID控制器类 -----------------------
// struct PIDController {
//     double kp, ki, kd;
//     double out_min, out_max;
//     double target_zone_thresh;

//     double integral;
//     double prev_error;
//     double derivative;

//     PIDController(double kp_val, double ki_val, double kd_val,
//                   double min_val, double max_val, double zone)
//         : kp(kp_val), ki(ki_val), kd(kd_val),
//           out_min(min_val), out_max(max_val),
//           target_zone_thresh(zone),
//           integral(0.0), prev_error(0.0), derivative(0.0) {}

//     double update(double err, double dt) {
//         if (std::abs(err) < target_zone_thresh) {
//             integral = 0.0;
//             prev_error = 0.0;
//             derivative = 0.0;
//             return 0.0;
//         }
//         derivative = (err - prev_error) / dt;
//         integral += err * dt;
//         if (ki > 0) {
//             double max_integral = (out_max * 0.5) / ki;
//             integral = std::clamp(integral, -max_integral, max_integral);
//         } else {
//             integral = 0.0;
//         }
//         double out = kp * err + ki * integral + kd * derivative;
//         prev_error = err;
//         return std::clamp(out, out_min, out_max);
//     }

//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         derivative = 0.0;
//     }
// };

// static inline float ema_filter(float prev, float val, float alpha) {
//     return prev * (1.0f - alpha) + val * alpha;
// }

// static inline cv::Point2f smooth_transition(const cv::Point2f& prev_pos,
//                                            const cv::Point2f& new_pos,
//                                            float max_change) {
//     float dx = new_pos.x - prev_pos.x;
//     float dy = new_pos.y - prev_pos.y;
//     float dist = std::sqrt(dx*dx + dy*dy);
//     if (dist <= max_change) return new_pos;
//     float ratio = max_change / dist;
//     return cv::Point2f(
//         prev_pos.x + dx * ratio * SMOOTH_ALPHA,
//         prev_pos.y + dy * ratio * SMOOTH_ALPHA
//     );
// }

// static inline float rect_iou(const cv::Rect& a, const cv::Rect& b) {
//     int x1 = std::max(a.x, b.x);
//     int y1 = std::max(a.y, b.y);
//     int x2 = std::min(a.x + a.width,  b.x + b.width);
//     int y2 = std::min(a.y + a.height, b.y + b.height);
//     int w = std::max(0, x2 - x1);
//     int h = std::max(0, y2 - y1);
//     int inter = w * h;
//     int areaA = a.width * a.height;
//     int areaB = b.width * b.height;
//     return inter > 0 ? float(inter) / float(areaA + areaB - inter + 1e-6f) : 0.f;
// }

// // ====================== 线程：YOLO + KCF 协同 ======================
// // ====================== 线程：YOLO + KCF + 实时画面 ======================
// void yolo_thread() {
//     int model_len = 0;
//     unsigned char* model_buf = load_model(RKNN_MODEL.c_str(), &model_len);
//     if (!model_buf) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Load model failed");
//         return;
//     }

//     rknn_context ctx;
//     if (rknn_init(&ctx, model_buf, model_len, RKNN_FLAG_PRIOR_MEDIUM, nullptr) != RKNN_SUCC) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "rknn_init failed");
//         free(model_buf);
//         return;
//     }
//     free(model_buf);

//     int frame_count = 0;
//     float fps = 0.0f;
//     auto start_time = std::chrono::steady_clock::now();
//     const int FPS_UPDATE_INTERVAL = 10;

//     cv::VideoCapture cap(0, cv::CAP_V4L2);
//     cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
//     cap.set(cv::CAP_PROP_FRAME_WIDTH,  INPUT_W);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, INPUT_H);
//     cap.set(cv::CAP_PROP_FPS, 60);
//     if (!cap.isOpened()) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Camera open failed");
//         rknn_destroy(ctx);
//         return;
//     }

//     cv::Mat frame, rgb, show;
//     const int OUT_NUM = 6;
//     rknn_output outputs[OUT_NUM] = {};

//     cv::Point2f last_smoothed_pos(CAM_CX, CAM_CY);

//     cv::Ptr<cv::Tracker> tracker;
//     bool tracker_inited = false;
//     cv::Rect tracker_bbox;
//     int tracker_lost_count = 0;

//     auto reset_tracker = [&]() {
//         tracker.release();
//         tracker_inited = false;
//         tracker_lost_count = 0;
//     };

//     auto init_tracker_with = [&](const cv::Mat& img, const cv::Rect& box) {
//         tracker = cv::TrackerKCF::create();
//         tracker->init(img, box);
//         tracker_inited = true;
//         tracker_bbox = box;
//         tracker_lost_count = 0;
//     };

//     int last_yolo_frame_id = -1000;
//     float last_yolo_score = 0.0f;

//     cv::namedWindow("YOLO+KCF", cv::WINDOW_AUTOSIZE);

//     while (true) {
//         cap >> frame;
//         if (frame.empty()) continue;
//         show = frame.clone();

//         /* ---------- 帧率 ---------- */
//         frame_count++;
//         if (frame_count % FPS_UPDATE_INTERVAL == 0) {
//             auto end_time = std::chrono::steady_clock::now();
//             std::chrono::duration<float> elapsed = end_time - start_time;
//             fps = FPS_UPDATE_INTERVAL / elapsed.count();
//             start_time = end_time;
//         }

//         /* ---------- YOLO 推理 ---------- */
//         bool run_yolo_now = (frame_count % YOLO_INTERVAL == 0);
//         bool yolo_got = false;
//         cv::Rect yolo_bbox;
//         float yolo_score = 0.f;

//         if (run_yolo_now) {
//             cv::resize(frame, rgb, cv::Size(INPUT_W, INPUT_H));
//             cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);

//             rknn_input inputs[1] = {};
//             inputs[0].index = 0;
//             inputs[0].type  = RKNN_TENSOR_UINT8;
//             inputs[0].fmt   = RKNN_TENSOR_NHWC;
//             inputs[0].buf   = rgb.data;
//             inputs[0].size  = rgb.total() * rgb.elemSize();

//             if (rknn_inputs_set(ctx, 1, inputs) == RKNN_SUCC &&
//                 rknn_run(ctx, nullptr) == RKNN_SUCC) {
//                 for (int i = 0; i < OUT_NUM; ++i) outputs[i].want_float = 1;
//                 if (rknn_outputs_get(ctx, OUT_NUM, outputs, nullptr) == RKNN_SUCC) {
//                     std::vector<rknn_output> out_vec(outputs, outputs + OUT_NUM);
//                     std::vector<DetectBox> keep = postprocess(out_vec, INPUT_H, INPUT_W);
//                     if (!keep.empty()) {
//                         const auto& best = keep[0];
//                         yolo_bbox = cv::Rect(int(std::round(best.xmin)),
//                                              int(std::round(best.ymin)),
//                                              int(std::round(best.xmax - best.xmin)),
//                                              int(std::round(best.ymax - best.ymin)));
//                         yolo_bbox &= cv::Rect(0,0,INPUT_W,INPUT_H);
//                         yolo_score = best.score;
//                         yolo_got = true;
//                         last_yolo_frame_id = frame_count;
//                         last_yolo_score = yolo_score;
//                     }
//                     rknn_outputs_release(ctx, OUT_NUM, outputs);
//                 }
//             }
//         }

//         /* ---------- KCF 跟踪 ---------- */
//         bool tracker_ok = false;
//         if (tracker_inited) {
//             tracker_ok = tracker->update(frame, tracker_bbox);
//             if (!tracker_ok) {
//                 tracker_lost_count++;
//                 if (tracker_lost_count > TRACKER_MAX_LOSS) reset_tracker();
//             } else {
//                 tracker_lost_count = 0;
//             }
//         }

//         /* ---------- 融合决策 ---------- */
//         TargetInfo current_target;
//         current_target.fps = fps;

//         if (yolo_got) {
//             bool need_reinit = (!tracker_inited) ||
//                                (rect_iou(tracker_bbox, yolo_bbox) < REINIT_IOU_THRESH) ||
//                                (yolo_score >= SCORE_REINIT_THRESH);
//             if (need_reinit) init_tracker_with(frame, yolo_bbox);

//             cv::Point2f yolo_center(yolo_bbox.x + yolo_bbox.width * 0.5f,
//                                     yolo_bbox.y + yolo_bbox.height * 0.5f);
//             last_smoothed_pos = smooth_transition(last_smoothed_pos, yolo_center, POSITION_CHANGE_THRESH);

//             current_target.detected = true;
//             current_target.bbox = yolo_bbox;
//             current_target.x = last_smoothed_pos.x;
//             current_target.y = last_smoothed_pos.y;
//             current_target.score = yolo_score;
//             current_target.src = TargetInfo::YOLO;

//             cv::rectangle(show, yolo_bbox, cv::Scalar(0, 255, 0), 2);
//             cv::putText(show, cv::format("YOLO %.2f", yolo_score),
//                         cv::Point(yolo_bbox.x, std::max(0, yolo_bbox.y - 5)),
//                         cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

//         } else if (tracker_inited && tracker_ok) {
//             cv::Point2f trk_center(tracker_bbox.x + tracker_bbox.width * 0.5f,
//                                    tracker_bbox.y + tracker_bbox.height * 0.5f);
//             last_smoothed_pos = smooth_transition(last_smoothed_pos, trk_center, POSITION_CHANGE_THRESH);

//             current_target.detected = true;
//             current_target.bbox = tracker_bbox;
//             current_target.x = last_smoothed_pos.x;
//             current_target.y = last_smoothed_pos.y;
//             current_target.score = last_yolo_score * 0.9f;
//             current_target.src = TargetInfo::TRACKER;

//             cv::rectangle(show, tracker_bbox, cv::Scalar(255, 0, 0), 2);
//             cv::putText(show, "KCF",
//                         cv::Point(tracker_bbox.x, std::max(0, tracker_bbox.y - 5)),
//                         cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
//         } else {
//             current_target.detected = false;
//             current_target.src = TargetInfo::NONE;
//             if (tracker_inited && tracker_lost_count > TRACKER_MAX_LOSS) reset_tracker();
//         }

//         /* ---------- 画面叠加 ---------- */
//         cv::circle(show, cv::Point((int)CAM_CX, (int)CAM_CY), (int)TARGET_ZONE_THRESH, cv::Scalar(0, 255, 255), 2);
//         if (current_target.detected) {
//             cv::circle(show, cv::Point((int)last_smoothed_pos.x, (int)last_smoothed_pos.y), 5, cv::Scalar(0, 0, 255), -1);
//             cv::putText(show,
//                         current_target.src == TargetInfo::YOLO ? "SRC: YOLO" : "SRC: KCF",
//                         cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
//         } else {
//             cv::putText(show, "SRC: NONE", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
//         }

//         cv::putText(show, cv::format("FPS: %.1f", fps),
//                     cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 2);

//         /* ---------- 实时显示 ---------- */
//         cv::imshow("YOLO+KCF", show);
//         if (cv::waitKey(1) == 27) break;   // Esc 退出

//         /* ---------- 共享给控制线程 ---------- */
//         {
//             std::lock_guard<std::mutex> lock(g_target_mutex);
//             g_target = current_target;
//         }
//     }

//     cv::destroyAllWindows();
//     rknn_destroy(ctx);
// }
// // ====================== 新增：/dev/ttyS3 接收就绪帧 ======================
// static bool wait_for_ready_frame() {
//     int fd = open(UART3_DEV.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd < 0) {
//         RCLCPP_ERROR(rclcpp::get_logger("uart3"), "Cannot open %s", UART3_DEV.c_str());
//         return false;
//     }

//     struct termios tty;
//     memset(&tty, 0, sizeof(tty));
//     if (tcgetattr(fd, &tty) != 0) {
//         close(fd);
//         return false;
//     }

//     cfsetospeed(&tty, B115200);
//     cfsetispeed(&tty, B115200);
//     tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
//     tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
//     tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
//     tty.c_lflag = 0;
//     tty.c_oflag = 0;
//     tty.c_cc[VMIN]  = READY_FRAME_LEN;
//     tty.c_cc[VTIME] = 10; // 1 s timeout
//     tcflush(fd, TCIOFLUSH);
//     tcsetattr(fd, TCSANOW, &tty);

//     uint8_t buf[READY_FRAME_LEN];
//     while (true) {
//         ssize_t n = read(fd, buf, READY_FRAME_LEN);
//         if (n == READY_FRAME_LEN &&
//             memcmp(buf, READY_FRAME, READY_FRAME_LEN) == 0) {
//             RCLCPP_INFO(rclcpp::get_logger("uart3"), "Received READY frame from MCU");
//             close(fd);
//             return true;
//         }
//     }
// }

// // ====================== ROS 2 节点类 ======================
// class GimbalYoloNode : public rclcpp::Node {
// public:
//     GimbalYoloNode()
//     : Node("gimbal_yolo_node"),
//       pid_pan_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
//       pid_tilt_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
//       GPIO3_B5(41), GPIO1_A2(34),
//       gpio_exported_(false),
//       in_target_zone_(false),
//       ready_received_(false)
//     {
//         declare_parameter<double>("kp_pan",  0.011);
//         declare_parameter<double>("ki_pan",  0.003);
//         declare_parameter<double>("kd_pan",  0.0005);
//         declare_parameter<double>("kp_tilt", 0.011);
//         declare_parameter<double>("ki_tilt", 0.00005);
//         declare_parameter<double>("kd_tilt", 0.0002);

//         double kp_pan = get_parameter("kp_pan").as_double();
//         double ki_pan = get_parameter("ki_pan").as_double();
//         double kd_pan = get_parameter("kd_pan").as_double();
//         double kp_tilt = get_parameter("kp_tilt").as_double();
//         double ki_tilt = get_parameter("ki_tilt").as_double();
//         double kd_tilt = get_parameter("kd_tilt").as_double();

//         serial_ = std::make_shared<SerialPort>(SERIAL_DEV, SERIAL_BAUD);
//         dm_ = damiao::Motor_Control(serial_);

//         m_horizontal_ = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x06, 0x16);
//         m_vertical_   = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x05, 0x15);

//         dm_.addMotor(m_horizontal_.get());
//         dm_.addMotor(m_vertical_.get());

//         dm_.switchControlMode(*m_horizontal_, damiao::VEL_MODE);
//         dm_.switchControlMode(*m_vertical_,   damiao::VEL_MODE);

//         dm_.set_zero_position(*m_horizontal_);
//         dm_.set_zero_position(*m_vertical_);

//         dm_.enable(*m_horizontal_);
//         dm_.enable(*m_vertical_);

//         max_vel_ = 100.0;
//         pid_pan_  = PIDController(kp_pan,  ki_pan,  kd_pan,  -max_vel_, max_vel_, TARGET_ZONE_THRESH);
//         pid_tilt_ = PIDController(kp_tilt, ki_tilt, kd_tilt, -max_vel_, max_vel_, TARGET_ZONE_THRESH);

//         meas_smooth_alpha_ = 0.7f;
//         cmd_smooth_alpha_  = 0.5f;
//         deadzone_px_       = 10.0;

//         cmd_prev_pan_  = 0.0;
//         cmd_prev_tilt_ = 0.0;
//         filt_cx_ = CAM_CX;
//         filt_cy_ = CAM_CY;
//         last_seen_time_ = std::chrono::steady_clock::now();
//         last_control_time_ = std::chrono::steady_clock::now();

//         initGpio();

//         // 等待 MCU 就绪
//         std::thread([this]() {
//             if (wait_for_ready_frame()) {
//                 ready_received_ = true;
//             } else {
//                 RCLCPP_ERROR(get_logger(), "Failed to receive READY frame");
//             }
//         }).detach();

//         timer_ = create_wall_timer(std::chrono::milliseconds(10),
//                                    std::bind(&GimbalYoloNode::control_loop, this));
//         marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
//         RCLCPP_INFO(get_logger(), "Gimbal + YOLO+KCF + GPIO control node started.");
//     }

//     ~GimbalYoloNode() {
//         releaseGpio();
//     }

// private:
//     const int GPIO3_B5, GPIO1_A2;
//     bool gpio_exported_;
//     bool in_target_zone_;
//     std::chrono::steady_clock::time_point zone_entry_time_;
//     bool ready_received_;

//     bool exportGpio(int gpio) {
//         int fd = open("/sys/class/gpio/export", O_WRONLY);
//         if (fd < 0) return false;
//         char buf[10];
//         snprintf(buf, sizeof(buf), "%d", gpio);
//         write(fd, buf, strlen(buf));
//         close(fd);
//         return true;
//     }

//     bool setGpioDirection(int gpio, const std::string& dir) {
//         char path[64];
//         snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
//         int fd = open(path, O_WRONLY);
//         if (fd < 0) return false;
//         write(fd, dir.c_str(), dir.size());
//         close(fd);
//         return true;
//     }

//     bool writeGpio(int gpio, int value) {
//         char path[64];
//         snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
//         int fd = open(path, O_WRONLY);
//         if (fd < 0) return false;
//         const char* val_str = value ? "1" : "0";
//         write(fd, val_str, strlen(val_str));
//         close(fd);
//         return true;
//     }

//     void initGpio() {
//         if (exportGpio(GPIO3_B5) && exportGpio(GPIO1_A2)) {
//             setGpioDirection(GPIO3_B5, "out");
//             setGpioDirection(GPIO1_A2, "out");
//             writeGpio(GPIO3_B5, 1);
//             writeGpio(GPIO1_A2, 0);
//             gpio_exported_ = true;
//         }
//     }

//     void releaseGpio() {
//         if (gpio_exported_) {
//             writeGpio(GPIO3_B5, 1);
//             writeGpio(GPIO1_A2, 0);
//             int fd = open("/sys/class/gpio/unexport", O_WRONLY);
//             if (fd >= 0) {
//                 char buf[10];
//                 snprintf(buf, sizeof(buf), "%d", GPIO3_B5);
//                 write(fd, buf, strlen(buf));
//                 snprintf(buf, sizeof(buf), "%d", GPIO1_A2);
//                 write(fd, buf, strlen(buf));
//                 close(fd);
//             }
//         }
//     }

//     void control_loop() {
//         if (!ready_received_) return;

//         auto now_tp = std::chrono::steady_clock::now();
//         std::chrono::duration<double> dt_duration = now_tp - last_control_time_;
//         double dt = dt_duration.count();
//         last_control_time_ = now_tp;

//         TargetInfo info;
//         {
//             std::lock_guard<std::mutex> lock(g_target_mutex);
//             info = g_target;
//         }

//         if (info.detected) {
//             last_seen_time_ = now_tp;
//             filt_cx_ = ema_filter(filt_cx_, info.x, (float)meas_smooth_alpha_);
//             filt_cy_ = ema_filter(filt_cy_, info.y, (float)meas_smooth_alpha_);

//             double err_x = filt_cx_ - CAM_CX;
//             double err_y = filt_cy_ - CAM_CY;

//             bool in_zone = (std::abs(err_x) < TARGET_ZONE_THRESH) && (std::abs(err_y) < TARGET_ZONE_THRESH);

//             if (in_zone) {
//                 if (!in_target_zone_) {
//                     in_target_zone_ = true;
//                     zone_entry_time_ = now_tp;
//                 } else {
//                     std::chrono::duration<double> duration = now_tp - zone_entry_time_;
//                     if (duration.count() >= 2.5) {
//                         if (gpio_exported_) {
//                             writeGpio(GPIO3_B5, 0);
//                             writeGpio(GPIO1_A2, 1);
//                         }
//                     }
//                 }
//             } else {
//                 in_target_zone_ = false;
//                 if (gpio_exported_) {
//                     writeGpio(GPIO3_B5, 1);
//                     writeGpio(GPIO1_A2, 0);
//                 }
//             }

//             if (std::abs(err_x) < deadzone_px_) err_x = 0.0;
//             if (std::abs(err_y) < deadzone_px_) err_y = 0.0;

//             double cmd_pan  = pid_pan_.update(err_x, dt);
//             double cmd_tilt = pid_tilt_.update(err_y, dt);

//             cmd_prev_pan_  = ema_filter((float)cmd_prev_pan_,  (float)cmd_pan,  (float)cmd_smooth_alpha_);
//             cmd_prev_tilt_ = ema_filter((float)cmd_prev_tilt_, (float)cmd_tilt, (float)cmd_smooth_alpha_);

//             dm_.control_vel(*m_horizontal_, -cmd_prev_pan_);
//             dm_.control_vel(*m_vertical_,    cmd_prev_tilt_);

//             publish_marker(err_x, err_y);
//             return;
//         }

//         in_target_zone_ = false;
//         if (gpio_exported_) {
//             writeGpio(GPIO3_B5, 1);
//             writeGpio(GPIO1_A2, 0);
//         }
//         cmd_prev_pan_ = 0.0;
//         cmd_prev_tilt_ = 0.0;
//         dm_.control_vel(*m_horizontal_, 0.0);
//         dm_.control_vel(*m_vertical_,   0.0);
//         filt_cx_ = CAM_CX;
//         filt_cy_ = CAM_CY;
//         pid_pan_.reset();
//         pid_tilt_.reset();
//         publish_marker(NAN, NAN);
//     }

//     void publish_marker(double err_x, double err_y) {
//         visualization_msgs::msg::Marker m;
//         m.header.frame_id = "map";
//         m.header.stamp = now();
//         m.ns = "target";
//         m.id = 0;
//         m.type = m.SPHERE;
//         m.action = m.ADD;
//         if (std::isnan(err_x) || std::isnan(err_y)) {
//             m.pose.position.x = 0;
//             m.pose.position.y = 0;
//         } else {
//             m.pose.position.x = err_x;
//             m.pose.position.y = err_y;
//         }
//         m.pose.position.z = 0;
//         m.scale.x = m.scale.y = m.scale.z = 0.05;
//         m.color.a = 1.0;
//         m.color.g = 1.0;
//         marker_pub_->publish(m);
//     }

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//     std::shared_ptr<SerialPort> serial_;
//     damiao::Motor_Control dm_;
//     std::shared_ptr<damiao::Motor> m_horizontal_, m_vertical_;

//     PIDController pid_pan_, pid_tilt_;
//     double max_vel_;
//     double cmd_prev_pan_, cmd_prev_tilt_;
//     double filt_cx_, filt_cy_;
//     double meas_smooth_alpha_, cmd_smooth_alpha_;
//     double deadzone_px_;
//     std::chrono::steady_clock::time_point last_seen_time_;
//     std::chrono::steady_clock::time_point last_control_time_;
// };

// // ====================== 主函数 ======================
// int main(int argc, char** argv) {
//     std::thread t(yolo_thread);
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<GimbalYoloNode>());
//     rclcpp::shutdown();
//     t.join();
//     return 0;
// }

//计时逻辑优化 全程索敌 就绪帧重置计时

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "visualization_msgs/msg/marker.hpp"
// #include "damiao_ros2_control/damiao.h"
// #include "damiao_ros2_control/SerialPort.h"
// #include <opencv2/opencv.hpp>
// #include <opencv2/tracking.hpp>
// #include <rknn_api.h>
// #include <mutex>
// #include <thread>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <chrono>
// #include <optional>
// #include <fcntl.h>
// #include <unistd.h>
// #include <termios.h>
// #include <atomic>
// #include <fstream>  

// // ====================== 模型 & 输入参数配置 ======================
// const std::string RKNN_MODEL = "/home/orangepi/yolov11n_rknn/light-1.rknn";
// const int INPUT_W = 640, INPUT_H = 480;
// const float OBJ_THRESH = 0.4f;
// const float NMS_THRESH  = 0.45f;
// const std::vector<std::string> CLASSES = {"ball"};

// // ====================== 摄像头 & 控制参数配置 ======================
// const std::string SERIAL_DEV = "/dev/ttyACM0";
// const int SERIAL_BAUD = B921600;
// const double CAM_CX = 330;
// const double CAM_CY = 240;
// const double TARGET_ZONE_THRESH = 1.0;  // 像素
// const float SMOOTH_ALPHA = 0.4f;
// double g_inner_margin_px = 1.0;  // 默认值
// // ========== YOLO 与 Tracker 协同参数 ==========
// const int   YOLO_INTERVAL          = 3;
// const int   TRACKER_MAX_LOSS       = 15;
// const float REINIT_IOU_THRESH      = 0.3f;
// const float SCORE_REINIT_THRESH    = 0.3f;
// const float POSITION_CHANGE_THRESH = 20.0f;

// // ====================== 新增：/dev/ttyS3 串口 ======================
// const std::string UART3_DEV   = "/dev/ttyS3";
// const int         UART3_BAUD  = 115200;
// const int         READY_FRAME_LEN = 3;
// const uint8_t     READY_FRAME[READY_FRAME_LEN] = {0xAA, 0x01, 0x55};

// // ====================== 计时控制 ======================
// static std::atomic_bool g_first_entry_after_ready{true};   // true = 需要重新计时
// static std::atomic_bool g_timer_enabled{false};            // 计时功能是否开启
// static std::atomic<double> g_timer_duration{0.0};          // 当前计时时间(秒)

// // ======================重置状态变量 ======================
// int  gpio_reset_ = 99;          // GPIO3_A3 对应序号 99
// std::chrono::steady_clock::time_point last_gpio_read_;
// // ====================== 全局共享变量 ======================
// struct TargetInfo {
//     bool detected = false;
//     float x = 0, y = 0;
//     cv::Rect bbox;
//     float score = 0.0f;
//     float fps = 0.0f;
//     enum Source { NONE=0, YOLO=1, TRACKER=2 } src = NONE;
// };
// TargetInfo g_target;
// std::mutex g_target_mutex;

// // ====================== 工具函数：加载RKNN模型 ======================
// static unsigned char* load_model(const char* fn, int* size) {
//     FILE* fp = fopen(fn, "rb");
//     if (!fp) return nullptr;
//     fseek(fp, 0, SEEK_END);
//     *size = ftell(fp);
//     unsigned char* buf = (unsigned char*)malloc(*size);
//     fseek(fp, 0, SEEK_SET);
//     fread(buf, 1, *size, fp);
//     fclose(fp);
//     return buf;
// }

// // ====================== 检测 & NMS ======================
// struct DetectBox {
//     int   classId;
//     float score;
//     float xmin, ymin, xmax, ymax;
//     DetectBox(int id, float s, float x1, float y1, float x2, float y2)
//         : classId(id), score(s), xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}
// };

// static float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

// static float IOU(float xmin1, float ymin1, float xmax1, float ymax1,
//                  float xmin2, float ymin2, float xmax2, float ymax2) {
//     float xmin = std::max(xmin1, xmin2);
//     float ymin = std::max(ymin1, ymin2);
//     float xmax = std::min(xmax1, xmax2);
//     float ymax = std::min(ymax1, ymax2);

//     float innerW = std::max(xmax - xmin, 0.f);
//     float innerH = std::max(ymax - ymin, 0.f);
//     float innerA = innerW * innerH;

//     float area1 = std::max(0.f, (xmax1 - xmin1)) * std::max(0.f, (ymax1 - ymin1));
//     float area2 = std::max(0.f, (xmax2 - xmin2)) * std::max(0.f, (ymax2 - ymin2));
//     return innerA / (area1 + area2 - innerA + 1e-6f);
// }

// static std::vector<DetectBox> NMS(const std::vector<DetectBox>& dets) {
//     if (dets.empty()) return {};
//     std::vector<cv::Rect> boxes;
//     std::vector<float>    scores;
//     for (const auto& b : dets) {
//         boxes.emplace_back(int(b.xmin), int(b.ymin), int(b.xmax - b.xmin), int(b.ymax - b.ymin));
//         scores.push_back(b.score);
//     }
//     std::vector<int> idxs(scores.size());
//     for (size_t i = 0; i < idxs.size(); ++i) idxs[i] = int(i);
//     std::sort(idxs.begin(), idxs.end(),
//               [&scores](int l, int r) { return scores[l] > scores[r]; });

//     std::vector<DetectBox> keep;
//     std::vector<bool>      suppressed(scores.size(), false);
//     for (size_t k = 0; k < idxs.size(); ++k) {
//         int i = idxs[k];
//         if (suppressed[i]) continue;
//         keep.push_back(dets[i]);
//         for (size_t t = k + 1; t < idxs.size(); ++t) {
//             int j = idxs[t];
//             if (IOU(boxes[i].x, boxes[i].y,
//                     boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height,
//                     boxes[j].x, boxes[j].y,
//                     boxes[j].x + boxes[j].width, boxes[j].y + boxes[j].height) > NMS_THRESH)
//                 suppressed[j] = true;
//         }
//     }
//     return keep;
// }

// static std::vector<DetectBox> postprocess(const std::vector<rknn_output>& out,
//                                           int img_h, int img_w) {
//     const int headNum = 3;
//     const std::vector<std::vector<int>> mapSize = {{60, 80}, {30, 40}, {15, 20}};
//     const std::vector<int> strides = {8, 16, 32};
//     const int class_num = 1;
//     const float objectThresh = OBJ_THRESH;

//     std::vector<std::vector<float>> outputs;
//     outputs.reserve(out.size());
//     for (const auto& o : out) {
//         size_t cnt = o.size / sizeof(float);
//         float* p = static_cast<float*>(o.buf);
//         outputs.emplace_back(p, p + cnt);
//     }

//     float scale_h = static_cast<float>(img_h) / INPUT_H;
//     float scale_w = static_cast<float>(img_w) / INPUT_W;

//     std::vector<DetectBox> detectResult;

//     for (int idx = 0; idx < headNum; ++idx) {
//         const auto& ms = mapSize[idx];
//         int map_h = ms[0];
//         int map_w = ms[1];
//         const std::vector<float>& reg = outputs[idx * 2 + 0];
//         const std::vector<float>& cls = outputs[idx * 2 + 1];

//         if ((int)cls.size() != class_num * map_h * map_w) continue;
//         if ((int)reg.size() != 4 * 16 * map_h * map_w) continue;

//         for (int h = 0; h < map_h; ++h) {
//             for (int w = 0; w < map_w; ++w) {
//                 int cls_index = 0 * map_h * map_w + h * map_w + w;
//                 float raw_conf = cls[cls_index];
//                 float conf = sigmoid(raw_conf);
//                 if (conf <= objectThresh) continue;

//                 float regdfl[4] = {0,0,0,0};
//                 for (int side = 0; side < 4; ++side) {
//                     int side_base = side * (16 * map_h * map_w);
//                     float sum_exp = 0.f;
//                     float exps[16];
//                     for (int k = 0; k < 16; ++k) {
//                         int idx_reg = side_base + k * (map_h * map_w) + h * map_w + w;
//                         float v = reg[idx_reg];
//                         float e = std::exp(v);
//                         exps[k] = e;
//                         sum_exp += e;
//                     }
//                     if (sum_exp <= 0.f) sum_exp = 1e-6f;
//                     float loc = 0.f;
//                     for (int k = 0; k < 16; ++k) loc += exps[k] * k;
//                     loc /= sum_exp;
//                     regdfl[side] = loc;
//                 }

//                 float grid_x = (w + 0.5f);
//                 float grid_y = (h + 0.5f);
//                 float stride = static_cast<float>(strides[idx]);

//                 float x1 = (grid_x - regdfl[0]) * stride;
//                 float y1 = (grid_y - regdfl[1]) * stride;
//                 float x2 = (grid_x + regdfl[2]) * stride;
//                 float y2 = (grid_y + regdfl[3]) * stride;

//                 float xmin = x1 * scale_w;
//                 float ymin = y1 * scale_h;
//                 float xmax = x2 * scale_w;
//                 float ymax = y2 * scale_h;

//                 xmin = std::max(0.f, xmin);
//                 ymin = std::max(0.f, ymin);
//                 xmax = std::min(static_cast<float>(img_w), xmax);
//                 ymax = std::min(static_cast<float>(img_h), ymax);

//                 detectResult.emplace_back(0, conf, xmin, ymin, xmax, ymax);
//             }
//         }
//     }
//     return NMS(detectResult);
// }

// // ====================== PID控制器类 -----------------------
// struct PIDController {
//     double kp, ki, kd;
//     double out_min, out_max;
//     double target_zone_thresh;

//     double integral;
//     double prev_error;
//     double derivative;

//     PIDController(double kp_val, double ki_val, double kd_val,
//                   double min_val, double max_val, double zone)
//         : kp(kp_val), ki(ki_val), kd(kd_val),
//           out_min(min_val), out_max(max_val),
//           target_zone_thresh(zone),
//           integral(0.0), prev_error(0.0), derivative(0.0) {}

//     double update(double err, double dt) {
//         if (std::abs(err) < target_zone_thresh) {
//             integral = 0.0;
//             prev_error = 0.0;
//             derivative = 0.0;
//             return 0.0;
//         }
//         derivative = (err - prev_error) / dt;
//         integral += err * dt;
//         if (ki > 0) {
//             double max_integral = (out_max * 0.5) / ki;
//             integral = std::clamp(integral, -max_integral, max_integral);
//         } else {
//             integral = 0.0;
//         }
//         double out = kp * err + ki * integral + kd * derivative;
//         prev_error = err;
//         return std::clamp(out, out_min, out_max);
//     }

//     void reset() {
//         integral = 0.0;
//         prev_error = 0.0;
//         derivative = 0.0;
//     }
// };

// static inline float ema_filter(float prev, float val, float alpha) {
//     return prev * (1.0f - alpha) + val * alpha;
// }

// static inline cv::Point2f smooth_transition(const cv::Point2f& prev_pos,
//                                            const cv::Point2f& new_pos,
//                                            float max_change) {
//     float dx = new_pos.x - prev_pos.x;
//     float dy = new_pos.y - prev_pos.y;
//     float dist = std::sqrt(dx*dx + dy*dy);
//     if (dist <= max_change) return new_pos;
//     float ratio = max_change / dist;
//     return cv::Point2f(
//         prev_pos.x + dx * ratio * SMOOTH_ALPHA,
//         prev_pos.y + dy * ratio * SMOOTH_ALPHA
//     );
// }

// static inline float rect_iou(const cv::Rect& a, const cv::Rect& b) {
//     int x1 = std::max(a.x, b.x);
//     int y1 = std::max(a.y, b.y);
//     int x2 = std::min(a.x + a.width,  b.x + b.width);
//     int y2 = std::min(a.y + a.height, b.y + b.height);
//     int w = std::max(0, x2 - x1);
//     int h = std::max(0, y2 - y1);
//     int inter = w * h;
//     int areaA = a.width * a.height;
//     int areaB = b.width * b.height;
//     return inter > 0 ? float(inter) / float(areaA + areaB - inter + 1e-6f) : 0.f;
// }

// // ====================== 线程：YOLO + KCF + 实时画面 ======================
// void yolo_thread() {
//     int model_len = 0;
//     unsigned char* model_buf = load_model(RKNN_MODEL.c_str(), &model_len);
//     if (!model_buf) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Load model failed");
//         return;
//     }

//     rknn_context ctx;
//     if (rknn_init(&ctx, model_buf, model_len, RKNN_FLAG_PRIOR_MEDIUM, nullptr) != RKNN_SUCC) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "rknn_init failed");
//         free(model_buf);
//         return;
//     }
//     free(model_buf);

//     int frame_count = 0;
//     float fps = 0.0f;
//     auto start_time = std::chrono::steady_clock::now();
//     const int FPS_UPDATE_INTERVAL = 10;

//     cv::VideoCapture cap(0, cv::CAP_V4L2);
//     cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
//     cap.set(cv::CAP_PROP_FRAME_WIDTH,  INPUT_W);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, INPUT_H);
//     cap.set(cv::CAP_PROP_FPS, 60);
//     if (!cap.isOpened()) {
//         RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Camera open failed");
//         rknn_destroy(ctx);
//         return;
//     }

//     cv::Mat frame, rgb, show;
//     const int OUT_NUM = 6;
//     rknn_output outputs[OUT_NUM] = {};

//     cv::Point2f last_smoothed_pos(CAM_CX, CAM_CY);

//     cv::Ptr<cv::Tracker> tracker;
//     bool tracker_inited = false;
//     cv::Rect tracker_bbox;
//     int tracker_lost_count = 0;

//     auto reset_tracker = [&]() {
//         tracker.release();
//         tracker_inited = false;
//         tracker_lost_count = 0;
//     };

//     auto init_tracker_with = [&](const cv::Mat& img, const cv::Rect& box) {
//         tracker = cv::TrackerKCF::create();
//         tracker->init(img, box);
//         tracker_inited = true;
//         tracker_bbox = box;
//         tracker_lost_count = 0;
//     };

//     int last_yolo_frame_id = -1000;
//     float last_yolo_score = 0.0f;

//     cv::namedWindow("YOLO+KCF", cv::WINDOW_AUTOSIZE);

//     while (true) {
//         cap >> frame;
//         if (frame.empty()) continue;
//         show = frame.clone();

//         frame_count++;
//         if (frame_count % FPS_UPDATE_INTERVAL == 0) {
//             auto end_time = std::chrono::steady_clock::now();
//             std::chrono::duration<float> elapsed = end_time - start_time;
//             fps = FPS_UPDATE_INTERVAL / elapsed.count();
//             start_time = end_time;
//         }

//         bool run_yolo_now = (frame_count % YOLO_INTERVAL == 0);
//         bool yolo_got = false;
//         cv::Rect yolo_bbox;
//         float yolo_score = 0.f;

//         if (run_yolo_now) {
//             cv::resize(frame, rgb, cv::Size(INPUT_W, INPUT_H));
//             cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);

//             rknn_input inputs[1] = {};
//             inputs[0].index = 0;
//             inputs[0].type  = RKNN_TENSOR_UINT8;
//             inputs[0].fmt   = RKNN_TENSOR_NHWC;
//             inputs[0].buf   = rgb.data;
//             inputs[0].size  = rgb.total() * rgb.elemSize();

//             if (rknn_inputs_set(ctx, 1, inputs) == RKNN_SUCC &&
//                 rknn_run(ctx, nullptr) == RKNN_SUCC) {
//                 for (int i = 0; i < OUT_NUM; ++i) outputs[i].want_float = 1;
//                 if (rknn_outputs_get(ctx, OUT_NUM, outputs, nullptr) == RKNN_SUCC) {
//                     std::vector<rknn_output> out_vec(outputs, outputs + OUT_NUM);
//                     std::vector<DetectBox> keep = postprocess(out_vec, INPUT_H, INPUT_W);
//                     if (!keep.empty()) {
//                         const auto& best = keep[0];
//                         yolo_bbox = cv::Rect(int(std::round(best.xmin)),
//                                              int(std::round(best.ymin)),
//                                              int(std::round(best.xmax - best.xmin)),
//                                              int(std::round(best.ymax - best.ymin)));
//                         yolo_bbox &= cv::Rect(0,0,INPUT_W,INPUT_H);
//                         yolo_score = best.score;
//                         yolo_got = true;
//                         last_yolo_frame_id = frame_count;
//                         last_yolo_score = yolo_score;
//                     }
//                     rknn_outputs_release(ctx, OUT_NUM, outputs);
//                 }
//             }
//         }

//         bool tracker_ok = false;
//         if (tracker_inited) {
//             tracker_ok = tracker->update(frame, tracker_bbox);
//             if (!tracker_ok) {
//                 tracker_lost_count++;
//                 if (tracker_lost_count > TRACKER_MAX_LOSS) reset_tracker();
//             } else {
//                 tracker_lost_count = 0;
//             }
//         }

//         TargetInfo current_target;
//         current_target.fps = fps;

//         if (yolo_got) {
//             bool need_reinit = (!tracker_inited) ||
//                                (rect_iou(tracker_bbox, yolo_bbox) < REINIT_IOU_THRESH) ||
//                                (yolo_score >= SCORE_REINIT_THRESH);
//             if (need_reinit) init_tracker_with(frame, yolo_bbox);

//             cv::Point2f yolo_center(yolo_bbox.x + yolo_bbox.width * 0.5f,
//                                     yolo_bbox.y + yolo_bbox.height * 0.5f);
//             last_smoothed_pos = smooth_transition(last_smoothed_pos, yolo_center, POSITION_CHANGE_THRESH);

//             current_target.detected = true;
//             current_target.bbox = yolo_bbox;
//             current_target.x = last_smoothed_pos.x;
//             current_target.y = last_smoothed_pos.y;
//             current_target.score = yolo_score;
//             current_target.src = TargetInfo::YOLO;

//         } else if (tracker_inited && tracker_ok) {
//             cv::Point2f trk_center(tracker_bbox.x + tracker_bbox.width * 0.5f,
//                                    tracker_bbox.y + tracker_bbox.height * 0.5f);
//             last_smoothed_pos = smooth_transition(last_smoothed_pos, trk_center, POSITION_CHANGE_THRESH);

//             current_target.detected = true;
//             current_target.bbox = tracker_bbox;
//             current_target.x = last_smoothed_pos.x;
//             current_target.y = last_smoothed_pos.y;
//             current_target.score = last_yolo_score * 0.9f;
//             current_target.src = TargetInfo::TRACKER;
//         } else {
//             current_target.detected = false;
//             current_target.src = TargetInfo::NONE;
//             if (tracker_inited && tracker_lost_count > TRACKER_MAX_LOSS) reset_tracker();
//         }

//         cv::circle(show, cv::Point((int)CAM_CX, (int)CAM_CY), (int)TARGET_ZONE_THRESH, cv::Scalar(0, 255, 255), 2);
        
//         // ---------- 画“缩小后的计时框” ----------
//         if (current_target.detected && !current_target.bbox.empty()) {
//             int m = static_cast<int>(g_inner_margin_px);
//             cv::Rect inner = current_target.bbox;
//             inner.x      += m;
//             inner.y      += m;
//             inner.width  = std::max(inner.width  - 2 * m, 0);
//             inner.height = std::max(inner.height - 2 * m, 0);

//             cv::Scalar color = (current_target.src == TargetInfo::YOLO)
//                                ? cv::Scalar(0, 255, 0)   // 绿色
//                                : cv::Scalar(255, 0, 0);  // 蓝色
//             cv::rectangle(show, inner, color, 2);
//         }

//         // ---------- 显示目标跟踪信息 ----------
//         if (current_target.detected) {
//             cv::circle(show, cv::Point((int)last_smoothed_pos.x, (int)last_smoothed_pos.y), 2, cv::Scalar(0, 0, 255), -1);
//             cv::putText(show,
//                         current_target.src == TargetInfo::YOLO ? "SRC: YOLO" : "SRC: KCF",
//                         cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
//         } else {
//             cv::putText(show, "SRC: NONE", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
//         }

//         // ---------- 显示FPS ----------
//         cv::putText(show, cv::format("FPS: %.1f", fps),
//                     cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 2);

//         // ---------- 显示计时状态和时间 ----------
//         bool timer_enabled = g_timer_enabled.load();
//         double timer_duration = g_timer_duration.load();
        
//         cv::Scalar timer_color = timer_enabled ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
//         cv::putText(show, cv::format("Timer Enabled: %s", timer_enabled ? "YES" : "NO"),
//                     cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, timer_color, 2);
        
//         if (timer_enabled) {
//             cv::putText(show, cv::format("Timer: %.1f s", timer_duration),
//                         cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
//         } else {
//             cv::putText(show, "Waiting for GPIO trigger...",
//                         cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
//         }

//         cv::imshow("YOLO+KCF", show);
//         if (cv::waitKey(1) == 27) break;   // Esc 退出

//         {
//             std::lock_guard<std::mutex> lock(g_target_mutex);
//             g_target = current_target;
//         }
//     }

//     cv::destroyAllWindows();
//     rknn_destroy(ctx);
// }

// // ====================== 接收就绪帧并复位计时 ======================
// static bool wait_for_ready_frame() {
//     int fd = open(UART3_DEV.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd < 0) {
//         RCLCPP_ERROR(rclcpp::get_logger("uart3"), "Cannot open %s", UART3_DEV.c_str());
//         return false;
//     }

//     struct termios tty{};
//     cfsetospeed(&tty, B115200);
//     cfsetispeed(&tty, B115200);
//     tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
//     tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
//     tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
//     tty.c_lflag = 0;
//     tty.c_oflag = 0;
//     tty.c_cc[VMIN]  = READY_FRAME_LEN;
//     tty.c_cc[VTIME] = 10;
//     tcflush(fd, TCIOFLUSH);
//     tcsetattr(fd, TCSANOW, &tty);

//     uint8_t buf[READY_FRAME_LEN];
//     while (true) {
//         ssize_t n = read(fd, buf, READY_FRAME_LEN);
//         if (n == READY_FRAME_LEN && memcmp(buf, READY_FRAME, READY_FRAME_LEN) == 0) {
//             RCLCPP_INFO(rclcpp::get_logger("uart3"), "Received READY frame, reset timer");
//             g_first_entry_after_ready.store(true);   // 触发计时清零
//             close(fd);
//             return true;
//         }
//     }
// }

// // ====================== ROS 2 节点类 ======================
// class GimbalYoloNode : public rclcpp::Node {
// public:
//     GimbalYoloNode()
//     : Node("gimbal_yolo_node"),
//       pid_pan_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
//       pid_tilt_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
//       GPIO3_B5(97), GPIO1_A2(34),
//       gpio_exported_(false),
//       in_target_zone_(false),
//       ready_received_(false),
//       gpio_triggered_(false)  // 新增：GPIO是否已触发
//     {
//         declare_parameter<double>("kp_pan",  0.022);
//         declare_parameter<double>("ki_pan",  0.003);
//         declare_parameter<double>("kd_pan",  0.0005);
//         declare_parameter<double>("kp_tilt", 0.009);
//         declare_parameter<double>("ki_tilt", 0.0005);
//         declare_parameter<double>("kd_tilt", 0.00005);

//         declare_parameter<double>("inner_margin_px", 3.0);

//         double kp_pan  = get_parameter("kp_pan").as_double();
//         double ki_pan  = get_parameter("ki_pan").as_double();
//         double kd_pan  = get_parameter("kd_pan").as_double();
//         double kp_tilt = get_parameter("kp_tilt").as_double();
//         double ki_tilt = get_parameter("ki_tilt").as_double();
//         double kd_tilt = get_parameter("kd_tilt").as_double();

//         g_inner_margin_px = get_parameter("inner_margin_px").as_double();

//         serial_ = std::make_shared<SerialPort>(SERIAL_DEV, SERIAL_BAUD);
//         dm_ = damiao::Motor_Control(serial_);

//         m_horizontal_ = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x06, 0x16);
//         m_vertical_   = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x05, 0x15);

//         dm_.addMotor(m_horizontal_.get());
//         dm_.addMotor(m_vertical_.get());

//         dm_.switchControlMode(*m_horizontal_, damiao::VEL_MODE);
//         dm_.switchControlMode(*m_vertical_,   damiao::VEL_MODE);

//         dm_.set_zero_position(*m_horizontal_);
//         dm_.set_zero_position(*m_vertical_);

//         dm_.enable(*m_horizontal_);
//         dm_.enable(*m_vertical_);

//         max_vel_ = 100.0;
//         pid_pan_  = PIDController(kp_pan,  ki_pan,  kd_pan,  -max_vel_, max_vel_, TARGET_ZONE_THRESH);
//         pid_tilt_ = PIDController(kp_tilt, ki_tilt, kd_tilt, -max_vel_, max_vel_, TARGET_ZONE_THRESH);

//         meas_smooth_alpha_ = 0.7f;
//         cmd_smooth_alpha_  = 0.5f;
//         deadzone_px_       = 1.0;

//         cmd_prev_pan_  = 0.0;
//         cmd_prev_tilt_ = 0.0;
//         filt_cx_ = CAM_CX;
//         filt_cy_ = CAM_CY;
//         last_seen_time_ = std::chrono::steady_clock::now();
//         last_control_time_ = std::chrono::steady_clock::now();

//         initGpio();

//         std::thread([this]() {
//             if (wait_for_ready_frame()) {
//             RCLCPP_INFO(get_logger(), "Received READY frame from MCU, resetting zone timer.");
//             zone_entry_time_ = std::chrono::steady_clock::now();
//             in_target_zone_ = false;
//             } else {
//                 RCLCPP_ERROR(get_logger(), "Failed to receive READY frame");
//             }
//         }).detach();

//         timer_ = create_wall_timer(std::chrono::milliseconds(10),
//                                    std::bind(&GimbalYoloNode::control_loop, this));
//         marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
//         RCLCPP_INFO(get_logger(), "Gimbal + YOLO+KCF + GPIO control node started.");
//     }

//     ~GimbalYoloNode() {
//         releaseGpio();
//     }

// private:
//     const int GPIO3_B5, GPIO1_A2;
//     bool gpio_exported_;
//     bool in_target_zone_;
//     std::chrono::steady_clock::time_point zone_entry_time_;
//     bool ready_received_;
//     bool gpio_triggered_;  // 新增：GPIO是否已触发
//     double inner_margin_px_;

//     bool exportGpio(int gpio) {
//         int fd = open("/sys/class/gpio/export", O_WRONLY);
//         if (fd < 0) {
//             RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to open export for gpio%d", gpio);
//             return false;
//         }
//         char buf[10];
//         snprintf(buf, sizeof(buf), "%d", gpio);
//         if (write(fd, buf, strlen(buf)) < 0) {
//             RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to export gpio%d", gpio);
//             close(fd);
//             return false;
//         }
//         close(fd);
//         // 等待导出完成
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         return true;
//     }

//     bool setGpioDirection(int gpio, const std::string& dir) {
//         char path[64];
//         snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
//         int fd = open(path, O_WRONLY);
//         if (fd < 0) {
//             RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to open direction for gpio%d", gpio);
//             return false;
//         }
//         write(fd, dir.c_str(), dir.size());
//         close(fd);
//         return true;
//     }

//     bool writeGpio(int gpio, int value) {
//         char path[64];
//         snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
//         int fd = open(path, O_WRONLY);
//         if (fd < 0) {
//             RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to open value for gpio%d", gpio);
//             return false;
//         }
//         const char* val_str = value ? "1" : "0";
//         write(fd, val_str, strlen(val_str));
//         close(fd);
//         return true;
//     }

//     void initGpio() {
//         if (exportGpio(GPIO3_B5) && exportGpio(GPIO1_A2)) {
//             setGpioDirection(GPIO3_B5, "out");
//             setGpioDirection(GPIO1_A2, "out");
//             writeGpio(GPIO3_B5, 1);
//             writeGpio(GPIO1_A2, 0);
//             gpio_exported_ = true;
//             RCLCPP_INFO(rclcpp::get_logger("gpio"), "GPIO B1 and A2 initialized");
//         }
        
//         // 初始化重置GPIO
//         if (exportGpio(gpio_reset_)) {
//             if (setGpioDirection(gpio_reset_, "in")) {
//                 RCLCPP_INFO(rclcpp::get_logger("gpio"), "GPIO%d initialized as input", gpio_reset_);
//             } else {
//                 RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to set direction for gpio%d", gpio_reset_);
//             }
//         } else {
//             RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to export gpio%d", gpio_reset_);
//         }
//     }

//     void releaseGpio() {
//         if (gpio_exported_) {
//             writeGpio(GPIO3_B5, 1);
//             writeGpio(GPIO1_A2, 0);
//             int fd = open("/sys/class/gpio/unexport", O_WRONLY);
//             if (fd >= 0) {
//                 char buf[10];
//                 snprintf(buf, sizeof(buf), "%d", GPIO3_B5);
//                 write(fd, buf, strlen(buf));
//                 snprintf(buf, sizeof(buf), "%d", GPIO1_A2);
//                 write(fd, buf, strlen(buf));
//                 close(fd);
//             }
//         }
        
//         // 释放重置GPIO
//         int fd = open("/sys/class/gpio/unexport", O_WRONLY);
//         if (fd >= 0) {
//             char buf[10];
//             snprintf(buf, sizeof(buf), "%d", gpio_reset_);
//             write(fd, buf, strlen(buf));
//             close(fd);
//         }
//     }

//     void control_loop() {
//         auto now_tp = std::chrono::steady_clock::now();
//         // 检测GPIO99低电平，触发计时功能开启
//         char path[64];
//         snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_reset_);
//         std::ifstream fs(path);
//         int val = 1;
        
//         if (fs.is_open()) {
//             fs >> val;
//             // 低电平触发计时功能开启
//             if (val == 0 && !gpio_triggered_) {
//                 gpio_triggered_ = true;
//                 g_timer_enabled.store(true);
//                 RCLCPP_INFO(get_logger(), "GPIO%d low level detected, timer enabled", gpio_reset_);
//                 zone_entry_time_ = now_tp;  // 初始化计时起点
//             }
//         } else {
//             RCLCPP_WARN(rclcpp::get_logger("gpio"), "Failed to open gpio%d value file", gpio_reset_);
//         }

//         // 更新全局计时状态
//         g_timer_enabled.store(gpio_triggered_);

//         std::chrono::duration<double> dt_duration = now_tp - last_control_time_;
//         double dt = dt_duration.count();
//         last_control_time_ = now_tp;

//         TargetInfo info;
//         {
//             std::lock_guard<std::mutex> lock(g_target_mutex);
//             info = g_target;
//         }

//         if (info.detected) {
//             last_seen_time_ = now_tp;
//             filt_cx_ = ema_filter(filt_cx_, info.x, (float)meas_smooth_alpha_);
//             filt_cy_ = ema_filter(filt_cy_, info.y, (float)meas_smooth_alpha_);

//             // 判断画面中心是否在目标框内
//             cv::Point2i center(static_cast<int>(CAM_CX), static_cast<int>(CAM_CY));
//             bool center_in_bbox = false;
            
//             if (!info.bbox.empty()) {
//                 int m = static_cast<int>(g_inner_margin_px);
//                 cv::Rect inner = info.bbox;
//                 inner.x += m;
//                 inner.y += m;
//                 inner.width  = std::max(inner.width  - 2*m, 0);
//                 inner.height = std::max(inner.height - 2*m, 0);
//                 center_in_bbox = inner.contains(center); 
//             }

//             // 计时逻辑：只有GPIO触发后且中心在框内才计时
//             if (gpio_triggered_) {  // 确保GPIO已触发
//                 if (center_in_bbox) {
//                     if (!in_target_zone_ || g_first_entry_after_ready.load()) {
//                         in_target_zone_ = true;
//                         zone_entry_time_ = now_tp;
//                         g_first_entry_after_ready.store(false);
//                     } else {
//                         // 计算并更新计时时间
//                         std::chrono::duration<double> duration = now_tp - zone_entry_time_;
//                         g_timer_duration.store(duration.count());
                        
//                         // 超过2.1秒触发输出
//                         if (duration.count() >= 2.1) {
//                             if (gpio_exported_) {
//                                 writeGpio(GPIO3_B5, 0);
//                                 writeGpio(GPIO1_A2, 1);
//                             }
//                         }
//                     }
//                 } else {
//                     in_target_zone_ = false;
//                     g_first_entry_after_ready.store(false);
//                     if (gpio_exported_) {
//                         writeGpio(GPIO3_B5, 1);
//                         writeGpio(GPIO1_A2, 0);
//                     }
//                 }
//             } else {
//                 // GPIO未触发，不计时
//                 in_target_zone_ = false;
//                 g_timer_duration.store(0.0);
//             }

//             // 云台控制逻辑
//             double err_x = filt_cx_ - CAM_CX;
//             double err_y = filt_cy_ - CAM_CY;

//             if (std::abs(err_x) < deadzone_px_) err_x = 0.0;
//             if (std::abs(err_y) < deadzone_px_) err_y = 0.0;

//             double cmd_pan  = pid_pan_.update(err_x, dt);
//             double cmd_tilt = pid_tilt_.update(err_y, dt);

//             cmd_prev_pan_  = ema_filter((float)cmd_prev_pan_,  (float)cmd_pan,  (float)cmd_smooth_alpha_);
//             cmd_prev_tilt_ = ema_filter((float)cmd_prev_tilt_, (float)cmd_tilt, (float)cmd_smooth_alpha_);

//             dm_.control_vel(*m_horizontal_, -cmd_prev_pan_);
//             dm_.control_vel(*m_vertical_,    cmd_prev_tilt_);

//             publish_marker(err_x, err_y);
//             return;
//         }

//         // 未检测到目标
//         in_target_zone_ = false;
//         g_first_entry_after_ready.store(false);
//         if (gpio_exported_) {
//             writeGpio(GPIO3_B5, 1);
//             writeGpio(GPIO1_A2, 0);
//         }
//         cmd_prev_pan_ = 0.0;
//         cmd_prev_tilt_ = 0.0;
//         dm_.control_vel(*m_horizontal_, 0.0);
//         dm_.control_vel(*m_vertical_,   0.0);
//         filt_cx_ = CAM_CX;
//         filt_cy_ = CAM_CY;
//         pid_pan_.reset();
//         pid_tilt_.reset();
//         publish_marker(NAN, NAN);
//     }

//     void publish_marker(double err_x, double err_y) {
//         visualization_msgs::msg::Marker m;
//         m.header.frame_id = "map";
//         m.header.stamp = now();
//         m.ns = "target";
//         m.id = 0;
//         m.type = m.SPHERE;
//         m.action = m.ADD;
//         if (std::isnan(err_x) || std::isnan(err_y)) {
//             m.pose.position.x = 0;
//             m.pose.position.y = 0;
//         } else {
//             m.pose.position.x = err_x;
//             m.pose.position.y = err_y;
//         }
//         m.pose.position.z = 0;
//         m.scale.x = m.scale.y = m.scale.z = 0.05;
//         m.color.a = 1.0;
//         m.color.g = 1.0;
//         marker_pub_->publish(m);
//     }

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//     std::shared_ptr<SerialPort> serial_;
//     damiao::Motor_Control dm_;
//     std::shared_ptr<damiao::Motor> m_horizontal_, m_vertical_;

//     PIDController pid_pan_, pid_tilt_;
//     double max_vel_;
//     double cmd_prev_pan_, cmd_prev_tilt_;
//     double filt_cx_, filt_cy_;
//     double meas_smooth_alpha_, cmd_smooth_alpha_;
//     double deadzone_px_;
//     std::chrono::steady_clock::time_point last_seen_time_;
//     std::chrono::steady_clock::time_point last_control_time_;
// };

// // ====================== 主函数 ======================
// int main(int argc, char** argv) {
//     std::thread t(yolo_thread);
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<GimbalYoloNode>());
//     rclcpp::shutdown();
//     t.join();
//     return 0;
// }

//动态pid

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "damiao_ros2_control/damiao.h"
#include "damiao_ros2_control/SerialPort.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <rknn_api.h>
#include <mutex>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <optional>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <atomic>
#include <fstream>

// ====================== 模型 & 输入参数配置 ======================
const std::string RKNN_MODEL = "/home/orangepi/yolov11n_rknn/light-1.rknn";
const int INPUT_W = 640, INPUT_H = 480;
const float OBJ_THRESH = 0.5f;
const float NMS_THRESH  = 0.45f;
const std::vector<std::string> CLASSES = {"ball"};

// ====================== 摄像头 & 控制参数配置 ======================
const std::string SERIAL_DEV = "/dev/ttyACM0";
const int SERIAL_BAUD = B921600;
const double CAM_CX = 330;
const double CAM_CY = 240;
const double TARGET_ZONE_THRESH = 1.0;  // 像素
const float SMOOTH_ALPHA = 0.4f;
double g_inner_margin_px = 1.0;  // 默认值
// ========== YOLO 与 Tracker 协同参数 ==========
const int   YOLO_INTERVAL          = 3;
const int   TRACKER_MAX_LOSS       = 15;
const float REINIT_IOU_THRESH      = 0.3f;
const float SCORE_REINIT_THRESH    = 0.3f;
const float POSITION_CHANGE_THRESH = 20.0f;

// ====================== 新增：/dev/ttyS3 串口 ======================
const std::string UART3_DEV   = "/dev/ttyS3";
const int         UART3_BAUD  = 115200;
const int         READY_FRAME_LEN = 3;
const uint8_t     READY_FRAME[READY_FRAME_LEN] = {0xAA, 0x01, 0x55};

// ====================== 计时控制 ======================
static std::atomic_bool g_first_entry_after_ready{true};   // true = 需要重新计时
static std::atomic_bool g_timer_enabled{false};            // 计时功能是否开启
static std::atomic<double> g_timer_duration{0.0};          // 当前计时时间(秒)

// ======================重置状态变量 ======================
int  gpio_reset_ = 99;          // GPIO3_A3 对应序号 99
std::chrono::steady_clock::time_point last_gpio_read_;
// ====================== 全局共享变量 ======================
struct TargetInfo {
    bool detected = false;
    float x = 0, y = 0;
    cv::Rect bbox;
    float score = 0.0f;
    float fps = 0.0f;
    enum Source { NONE=0, YOLO=1, TRACKER=2 } src = NONE;
};
TargetInfo g_target;
std::mutex g_target_mutex;

// ====================== 工具函数：加载RKNN模型 ======================
static unsigned char* load_model(const char* fn, int* size) {
    FILE* fp = fopen(fn, "rb");
    if (!fp) return nullptr;
    fseek(fp, 0, SEEK_END);
    *size = ftell(fp);
    unsigned char* buf = (unsigned char*)malloc(*size);
    fseek(fp, 0, SEEK_SET);
    fread(buf, 1, *size, fp);
    fclose(fp);
    return buf;
}

// ====================== 检测 & NMS ======================
struct DetectBox {
    int   classId;
    float score;
    float xmin, ymin, xmax, ymax;
    DetectBox(int id, float s, float x1, float y1, float x2, float y2)
        : classId(id), score(s), xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}
};

static float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

static float IOU(float xmin1, float ymin1, float xmax1, float ymax1,
                 float xmin2, float ymin2, float xmax2, float ymax2) {
    float xmin = std::max(xmin1, xmin2);
    float ymin = std::max(ymin1, ymin2);
    float xmax = std::min(xmax1, xmax2);
    float ymax = std::min(ymax1, ymax2);

    float innerW = std::max(xmax - xmin, 0.f);
    float innerH = std::max(ymax - ymin, 0.f);
    float innerA = innerW * innerH;

    float area1 = std::max(0.f, (xmax1 - xmin1)) * std::max(0.f, (ymax1 - ymin1));
    float area2 = std::max(0.f, (xmax2 - xmin2)) * std::max(0.f, (ymax2 - ymin2));
    return innerA / (area1 + area2 - innerA + 1e-6f);
}

static std::vector<DetectBox> NMS(const std::vector<DetectBox>& dets) {
    if (dets.empty()) return {};
    std::vector<cv::Rect> boxes;
    std::vector<float>    scores;
    for (const auto& b : dets) {
        boxes.emplace_back(int(b.xmin), int(b.ymin), int(b.xmax - b.xmin), int(b.ymax - b.ymin));
        scores.push_back(b.score);
    }
    std::vector<int> idxs(scores.size());
    for (size_t i = 0; i < idxs.size(); ++i) idxs[i] = int(i);
    std::sort(idxs.begin(), idxs.end(),
              [&scores](int l, int r) { return scores[l] > scores[r]; });

    std::vector<DetectBox> keep;
    std::vector<bool>      suppressed(scores.size(), false);
    for (size_t k = 0; k < idxs.size(); ++k) {
        int i = idxs[k];
        if (suppressed[i]) continue;
        keep.push_back(dets[i]);
        for (size_t t = k + 1; t < idxs.size(); ++t) {
            int j = idxs[t];
            if (IOU(boxes[i].x, boxes[i].y,
                    boxes[i].x + boxes[i].width, boxes[i].y + boxes[i].height,
                    boxes[j].x, boxes[j].y,
                    boxes[j].x + boxes[j].width, boxes[j].y + boxes[j].height) > NMS_THRESH)
                suppressed[j] = true;
        }
    }
    return keep;
}

static std::vector<DetectBox> postprocess(const std::vector<rknn_output>& out,
                                          int img_h, int img_w) {
    const int headNum = 3;
    const std::vector<std::vector<int>> mapSize = {{60, 80}, {30, 40}, {15, 20}};
    const std::vector<int> strides = {8, 16, 32};
    const int class_num = 1;
    const float objectThresh = OBJ_THRESH;

    std::vector<std::vector<float>> outputs;
    outputs.reserve(out.size());
    for (const auto& o : out) {
        size_t cnt = o.size / sizeof(float);
        float* p = static_cast<float*>(o.buf);
        outputs.emplace_back(p, p + cnt);
    }

    float scale_h = static_cast<float>(img_h) / INPUT_H;
    float scale_w = static_cast<float>(img_w) / INPUT_W;

    std::vector<DetectBox> detectResult;

    for (int idx = 0; idx < headNum; ++idx) {
        const auto& ms = mapSize[idx];
        int map_h = ms[0];
        int map_w = ms[1];
        const std::vector<float>& reg = outputs[idx * 2 + 0];
        const std::vector<float>& cls = outputs[idx * 2 + 1];

        if ((int)cls.size() != class_num * map_h * map_w) continue;
        if ((int)reg.size() != 4 * 16 * map_h * map_w) continue;

        for (int h = 0; h < map_h; ++h) {
            for (int w = 0; w < map_w; ++w) {
                int cls_index = 0 * map_h * map_w + h * map_w + w;
                float raw_conf = cls[cls_index];
                float conf = sigmoid(raw_conf);
                if (conf <= objectThresh) continue;

                float regdfl[4] = {0,0,0,0};
                for (int side = 0; side < 4; ++side) {
                    int side_base = side * (16 * map_h * map_w);
                    float sum_exp = 0.f;
                    float exps[16];
                    for (int k = 0; k < 16; ++k) {
                        int idx_reg = side_base + k * (map_h * map_w) + h * map_w + w;
                        float v = reg[idx_reg];
                        float e = std::exp(v);
                        exps[k] = e;
                        sum_exp += e;
                    }
                    if (sum_exp <= 0.f) sum_exp = 1e-6f;
                    float loc = 0.f;
                    for (int k = 0; k < 16; ++k) loc += exps[k] * k;
                    loc /= sum_exp;
                    regdfl[side] = loc;
                }

                float grid_x = (w + 0.5f);
                float grid_y = (h + 0.5f);
                float stride = static_cast<float>(strides[idx]);

                float x1 = (grid_x - regdfl[0]) * stride;
                float y1 = (grid_y - regdfl[1]) * stride;
                float x2 = (grid_x + regdfl[2]) * stride;
                float y2 = (grid_y + regdfl[3]) * stride;

                float xmin = x1 * scale_w;
                float ymin = y1 * scale_h;
                float xmax = x2 * scale_w;
                float ymax = y2 * scale_h;

                xmin = std::max(0.f, xmin);
                ymin = std::max(0.f, ymin);
                xmax = std::min(static_cast<float>(img_w), xmax);
                ymax = std::min(static_cast<float>(img_h), ymax);

                detectResult.emplace_back(0, conf, xmin, ymin, xmax, ymax);
            }
        }
    }
    return NMS(detectResult);
}

// ====================== PID控制器类 -----------------------
struct PIDController {
    double kp, ki, kd;
    double out_min, out_max;
    double target_zone_thresh;

    double integral;
    double prev_error;
    double derivative;

    PIDController(double kp_val, double ki_val, double kd_val,
                  double min_val, double max_val, double zone)
        : kp(kp_val), ki(ki_val), kd(kd_val),
          out_min(min_val), out_max(max_val),
          target_zone_thresh(zone),
          integral(0.0), prev_error(0.0), derivative(0.0) {}

    double update(double err, double dt) {
        if (std::abs(err) < target_zone_thresh) {
            integral = 0.0;
            prev_error = 0.0;
            derivative = 0.0;
            return 0.0;
        }
        derivative = (err - prev_error) / dt;
        integral += err * dt;
        if (ki > 0) {
            double max_integral = (out_max * 0.5) / ki;
            integral = std::clamp(integral, -max_integral, max_integral);
        } else {
            integral = 0.0;
        }
        double out = kp * err + ki * integral + kd * derivative;
        prev_error = err;
        return std::clamp(out, out_min, out_max);
    }

    void reset() {
        integral = 0.0;
        prev_error = 0.0;
        derivative = 0.0;
    }

    // 新增：动态设置kp
    void set_kp(double kp_new) { kp = kp_new; }
};

static inline float ema_filter(float prev, float val, float alpha) {
    return prev * (1.0f - alpha) + val * alpha;
}

static inline cv::Point2f smooth_transition(const cv::Point2f& prev_pos,
                                           const cv::Point2f& new_pos,
                                           float max_change) {
    float dx = new_pos.x - prev_pos.x;
    float dy = new_pos.y - prev_pos.y;
    float dist = std::sqrt(dx*dx + dy*dy);
    if (dist <= max_change) return new_pos;
    float ratio = max_change / dist;
    return cv::Point2f(
        prev_pos.x + dx * ratio * SMOOTH_ALPHA,
        prev_pos.y + dy * ratio * SMOOTH_ALPHA
    );
}

static inline float rect_iou(const cv::Rect& a, const cv::Rect& b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width,  b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    int w = std::max(0, x2 - x1);
    int h = std::max(0, y2 - y1);
    int inter = w * h;
    int areaA = a.width * a.height;
    int areaB = b.width * b.height;
    return inter > 0 ? float(inter) / float(areaA + areaB - inter + 1e-6f) : 0.f;
}

// ====================== 线程：YOLO + KCF + 实时画面 ======================
void yolo_thread() {
    int model_len = 0;
    unsigned char* model_buf = load_model(RKNN_MODEL.c_str(), &model_len);
    if (!model_buf) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Load model failed");
        return;
    }

    rknn_context ctx;
    if (rknn_init(&ctx, model_buf, model_len, RKNN_FLAG_PRIOR_MEDIUM, nullptr) != RKNN_SUCC) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo"), "rknn_init failed");
        free(model_buf);
        return;
    }
    free(model_buf);

    int frame_count = 0;
    float fps = 0.0f;
    auto start_time = std::chrono::steady_clock::now();
    const int FPS_UPDATE_INTERVAL = 10;

    cv::VideoCapture cap(0, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  INPUT_W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, INPUT_H);
    cap.set(cv::CAP_PROP_FPS, 60);

    cap.set(cv::CAP_PROP_BRIGHTNESS, 15);      // 轻微提亮

    if (!cap.isOpened()) {
        RCLCPP_ERROR(rclcpp::get_logger("yolo"), "Camera open failed");
        rknn_destroy(ctx);
        return;
    }

    cv::Mat frame, rgb, show;
    const int OUT_NUM = 6;
    rknn_output outputs[OUT_NUM] = {};

    cv::Point2f last_smoothed_pos(CAM_CX, CAM_CY);

    cv::Ptr<cv::Tracker> tracker;
    bool tracker_inited = false;
    cv::Rect tracker_bbox;
    int tracker_lost_count = 0;

    auto reset_tracker = [&]() {
        tracker.release();
        tracker_inited = false;
        tracker_lost_count = 0;
    };

    auto init_tracker_with = [&](const cv::Mat& img, const cv::Rect& box) {
        tracker = cv::TrackerKCF::create();
        tracker->init(img, box);
        tracker_inited = true;
        tracker_bbox = box;
        tracker_lost_count = 0;
    };

    int last_yolo_frame_id = -1000;
    float last_yolo_score = 0.0f;

    // cv::namedWindow("YOLO+KCF", cv::WINDOW_AUTOSIZE);

    while (true) {
        cap >> frame;
        if (frame.empty()) continue;
        show = frame.clone();

        frame_count++;
        if (frame_count % FPS_UPDATE_INTERVAL == 0) {
            auto end_time = std::chrono::steady_clock::now();
            std::chrono::duration<float> elapsed = end_time - start_time;
            fps = FPS_UPDATE_INTERVAL / elapsed.count();
            start_time = end_time;
        }

        bool run_yolo_now = (frame_count % YOLO_INTERVAL == 0);
        bool yolo_got = false;
        cv::Rect yolo_bbox;
        float yolo_score = 0.f;

        if (run_yolo_now) {
            cv::resize(frame, rgb, cv::Size(INPUT_W, INPUT_H));
            cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);

            rknn_input inputs[1] = {};
            inputs[0].index = 0;
            inputs[0].type  = RKNN_TENSOR_UINT8;
            inputs[0].fmt   = RKNN_TENSOR_NHWC;
            inputs[0].buf   = rgb.data;
            inputs[0].size  = rgb.total() * rgb.elemSize();

            if (rknn_inputs_set(ctx, 1, inputs) == RKNN_SUCC &&
                rknn_run(ctx, nullptr) == RKNN_SUCC) {
                for (int i = 0; i < OUT_NUM; ++i) outputs[i].want_float = 1;
                if (rknn_outputs_get(ctx, OUT_NUM, outputs, nullptr) == RKNN_SUCC) {
                    std::vector<rknn_output> out_vec(outputs, outputs + OUT_NUM);
                    std::vector<DetectBox> keep = postprocess(out_vec, INPUT_H, INPUT_W);
                    if (!keep.empty()) {
                        const auto& best = keep[0];
                        yolo_bbox = cv::Rect(int(std::round(best.xmin)),
                                             int(std::round(best.ymin)),
                                             int(std::round(best.xmax - best.xmin)),
                                             int(std::round(best.ymax - best.ymin)));
                        yolo_bbox &= cv::Rect(0,0,INPUT_W,INPUT_H);
                        yolo_score = best.score;
                        yolo_got = true;
                        last_yolo_frame_id = frame_count;
                        last_yolo_score = yolo_score;
                    }
                    rknn_outputs_release(ctx, OUT_NUM, outputs);
                }
            }
        }

        bool tracker_ok = false;
        if (tracker_inited) {
            tracker_ok = tracker->update(frame, tracker_bbox);
            if (!tracker_ok) {
                tracker_lost_count++;
                if (tracker_lost_count > TRACKER_MAX_LOSS) reset_tracker();
            } else {
                tracker_lost_count = 0;
            }
        }

        TargetInfo current_target;
        current_target.fps = fps;

        int area_px = 0;
        if (yolo_got) {
            bool need_reinit = (!tracker_inited) ||
                               (rect_iou(tracker_bbox, yolo_bbox) < REINIT_IOU_THRESH) ||
                               (yolo_score >= SCORE_REINIT_THRESH);
            if (need_reinit) init_tracker_with(frame, yolo_bbox);

            cv::Point2f yolo_center(yolo_bbox.x + yolo_bbox.width * 0.5f,
                                    yolo_bbox.y + yolo_bbox.height * 0.5f);
            last_smoothed_pos = smooth_transition(last_smoothed_pos, yolo_center, POSITION_CHANGE_THRESH);

            current_target.detected = true;
            current_target.bbox = yolo_bbox;
            current_target.x = last_smoothed_pos.x;
            current_target.y = last_smoothed_pos.y;
            current_target.score = yolo_score;
            current_target.src = TargetInfo::YOLO;
            area_px = yolo_bbox.area();

        } else if (tracker_inited && tracker_ok) {
            cv::Point2f trk_center(tracker_bbox.x + tracker_bbox.width * 0.5f,
                                   tracker_bbox.y + tracker_bbox.height * 0.5f);
            last_smoothed_pos = smooth_transition(last_smoothed_pos, trk_center, POSITION_CHANGE_THRESH);

            current_target.detected = true;
            current_target.bbox = tracker_bbox;
            current_target.x = last_smoothed_pos.x;
            current_target.y = last_smoothed_pos.y;
            current_target.score = last_yolo_score * 0.9f;
            current_target.src = TargetInfo::TRACKER;
            area_px = tracker_bbox.area();
        } else {
            current_target.detected = false;
            current_target.src = TargetInfo::NONE;
            if (tracker_inited && tracker_lost_count > TRACKER_MAX_LOSS) reset_tracker();
        }

        cv::circle(show, cv::Point((int)CAM_CX, (int)CAM_CY), (int)TARGET_ZONE_THRESH, cv::Scalar(0, 255, 255), 2);

        // ---------- 画“缩小后的计时框” ----------
        if (current_target.detected && !current_target.bbox.empty()) {
            int m = static_cast<int>(g_inner_margin_px);
            cv::Rect inner = current_target.bbox;
            inner.x      += m;
            inner.y      += m;
            inner.width  = std::max(inner.width  - 2 * m, 0);
            inner.height = std::max(inner.height - 2 * m, 0);

            cv::Scalar color = (current_target.src == TargetInfo::YOLO)
                               ? cv::Scalar(0, 255, 0)   // 绿色
                               : cv::Scalar(255, 0, 0);  // 蓝色
            cv::rectangle(show, inner, color, 2);
        }

        // ---------- 显示目标跟踪信息 ----------
        if (current_target.detected) {
            cv::circle(show, cv::Point((int)last_smoothed_pos.x, (int)last_smoothed_pos.y), 2, cv::Scalar(0, 0, 255), -1);
            cv::putText(show,
                        current_target.src == TargetInfo::YOLO ? "SRC: YOLO" : "SRC: KCF",
                        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        } else {
            cv::putText(show, "SRC: NONE", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        // ---------- 显示FPS ----------
        cv::putText(show, cv::format("FPS: %.1f", fps),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 2);

        // ---------- 显示面积 & 当前kp ----------
        if (current_target.detected) {
            cv::putText(show, cv::format("Area: %d", area_px),
                        cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        }

        // ---------- 显示计时状态和时间 ----------
        bool timer_enabled = g_timer_enabled.load();
        double timer_duration = g_timer_duration.load();

        cv::Scalar timer_color = timer_enabled ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(show, cv::format("Timer Enabled: %s", timer_enabled ? "YES" : "NO"),
                    cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, timer_color, 2);

        if (timer_enabled) {
            cv::putText(show, cv::format("Timer: %.1f s", timer_duration),
                        cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
        } else {
            cv::putText(show, "Waiting for GPIO trigger...",
                        cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        // cv::imshow("YOLO+KCF", show);
        // if (cv::waitKey(1) == 27) break;   // Esc 退出

        {
            std::lock_guard<std::mutex> lock(g_target_mutex);
            g_target = current_target;
        }
    }

    cv::destroyAllWindows();
    rknn_destroy(ctx);
}

// ====================== 接收就绪帧并复位计时 ======================
static bool wait_for_ready_frame() {
    int fd = open(UART3_DEV.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("uart3"), "Cannot open %s", UART3_DEV.c_str());
        return false;
    }

    struct termios tty{};
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = READY_FRAME_LEN;
    tty.c_cc[VTIME] = 10;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &tty);

    uint8_t buf[READY_FRAME_LEN];
    while (true) {
        ssize_t n = read(fd, buf, READY_FRAME_LEN);
        if (n == READY_FRAME_LEN && memcmp(buf, READY_FRAME, READY_FRAME_LEN) == 0) {
            RCLCPP_INFO(rclcpp::get_logger("uart3"), "Received READY frame, reset timer");
            g_first_entry_after_ready.store(true);   // 触发计时清零
            close(fd);
            return true;
        }
    }
}

// ====================== ROS 2 节点类 ======================
class GimbalYoloNode : public rclcpp::Node {
public:
    GimbalYoloNode()
    : Node("gimbal_yolo_node"),
      pid_pan_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      pid_tilt_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      GPIO3_B5(97), GPIO1_A2(34),
      gpio_exported_(false),
      in_target_zone_(false),
      ready_received_(false),
      gpio_triggered_(false),
      inner_margin_px_(3.0)
    {
        declare_parameter<double>("kp_pan_base",  0.022);
        declare_parameter<double>("ki_pan",  0.003);
        declare_parameter<double>("kd_pan",  0.0005);
        declare_parameter<double>("kp_tilt_base", 0.005);//0.009
        declare_parameter<double>("ki_tilt", 0.0005);
        declare_parameter<double>("kd_tilt", 0.00005);

        declare_parameter<double>("inner_margin_px", 3.0);

        kp_pan_base_  = get_parameter("kp_pan_base").as_double();
        ki_pan_  = get_parameter("ki_pan").as_double();
        kd_pan_  = get_parameter("kd_pan").as_double();
        kp_tilt_base_ = get_parameter("kp_tilt_base").as_double();
        ki_tilt_ = get_parameter("ki_tilt").as_double();
        kd_tilt_ = get_parameter("kd_tilt").as_double();

        g_inner_margin_px = get_parameter("inner_margin_px").as_double();

        serial_ = std::make_shared<SerialPort>(SERIAL_DEV, SERIAL_BAUD);
        dm_ = damiao::Motor_Control(serial_);

        m_horizontal_ = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x06, 0x16);
        m_vertical_   = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x05, 0x15);

        dm_.addMotor(m_horizontal_.get());
        dm_.addMotor(m_vertical_.get());

        dm_.switchControlMode(*m_horizontal_, damiao::VEL_MODE);
        dm_.switchControlMode(*m_vertical_,   damiao::VEL_MODE);

        dm_.set_zero_position(*m_horizontal_);
        dm_.set_zero_position(*m_vertical_);

        dm_.enable(*m_horizontal_);
        dm_.enable(*m_vertical_);

        max_vel_ = 100.0;

        pid_pan_  = PIDController(kp_pan_base_,  ki_pan_,  kd_pan_,  -max_vel_, max_vel_, TARGET_ZONE_THRESH);
        pid_tilt_ = PIDController(kp_tilt_base_, ki_tilt_, kd_tilt_, -max_vel_, max_vel_, TARGET_ZONE_THRESH);

        meas_smooth_alpha_ = 0.7f;
        cmd_smooth_alpha_  = 0.5f;
        deadzone_px_       = 1.0;

        cmd_prev_pan_  = 0.0;
        cmd_prev_tilt_ = 0.0;
        filt_cx_ = CAM_CX;
        filt_cy_ = CAM_CY;
        last_seen_time_ = std::chrono::steady_clock::now();
        last_control_time_ = std::chrono::steady_clock::now();

        initGpio();

        std::thread([this]() {
            if (wait_for_ready_frame()) {
                RCLCPP_INFO(get_logger(), "Received READY frame from MCU, resetting zone timer.");
                zone_entry_time_ = std::chrono::steady_clock::now();
                in_target_zone_ = false;
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to receive READY frame");
            }
        }).detach();

        timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&GimbalYoloNode::control_loop, this));
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
        RCLCPP_INFO(get_logger(), "Gimbal + YOLO+KCF + GPIO control node started.");
    }

    ~GimbalYoloNode() {
        releaseGpio();
    }

private:
    const int GPIO3_B5, GPIO1_A2;
    bool gpio_exported_;
    bool in_target_zone_;
    std::chrono::steady_clock::time_point zone_entry_time_;
    bool ready_received_;
    bool gpio_triggered_;
    double inner_margin_px_;

    double kp_pan_base_, ki_pan_, kd_pan_;
    double kp_tilt_base_, ki_tilt_, kd_tilt_;

    bool exportGpio(int gpio) {
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to open export for gpio%d", gpio);
            return false;
        }
        char buf[10];
        snprintf(buf, sizeof(buf), "%d", gpio);
        if (write(fd, buf, strlen(buf)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to export gpio%d", gpio);
            close(fd);
            return false;
        }
        close(fd);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return true;
    }

    bool setGpioDirection(int gpio, const std::string& dir) {
        char path[64];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
        int fd = open(path, O_WRONLY);
        if (fd < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to open direction for gpio%d", gpio);
            return false;
        }
        write(fd, dir.c_str(), dir.size());
        close(fd);
        return true;
    }

    bool writeGpio(int gpio, int value) {
        char path[64];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
        int fd = open(path, O_WRONLY);
        if (fd < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to open value for gpio%d", gpio);
            return false;
        }
        const char* val_str = value ? "1" : "0";
        write(fd, val_str, strlen(val_str));
        close(fd);
        return true;
    }

    void initGpio() {
        if (exportGpio(GPIO3_B5) && exportGpio(GPIO1_A2)) {
            setGpioDirection(GPIO3_B5, "out");
            setGpioDirection(GPIO1_A2, "out");
            writeGpio(GPIO3_B5, 1);
            writeGpio(GPIO1_A2, 0);
            gpio_exported_ = true;
            RCLCPP_INFO(rclcpp::get_logger("gpio"), "GPIO B1 and A2 initialized");
        }

        if (exportGpio(gpio_reset_)) {
            if (setGpioDirection(gpio_reset_, "in")) {
                RCLCPP_INFO(rclcpp::get_logger("gpio"), "GPIO%d initialized as input", gpio_reset_);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to set direction for gpio%d", gpio_reset_);
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("gpio"), "Failed to export gpio%d", gpio_reset_);
        }
    }

    void releaseGpio() {
        if (gpio_exported_) {
            writeGpio(GPIO3_B5, 1);
            writeGpio(GPIO1_A2, 0);
            int fd = open("/sys/class/gpio/unexport", O_WRONLY);
            if (fd >= 0) {
                char buf[10];
                snprintf(buf, sizeof(buf), "%d", GPIO3_B5);
                write(fd, buf, strlen(buf));
                snprintf(buf, sizeof(buf), "%d", GPIO1_A2);
                write(fd, buf, strlen(buf));
                close(fd);
            }
        }

        int fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd >= 0) {
            char buf[10];
            snprintf(buf, sizeof(buf), "%d", gpio_reset_);
            write(fd, buf, strlen(buf));
            close(fd);
        }
    }

    void control_loop() {
        auto now_tp = std::chrono::steady_clock::now();
        char path[64];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_reset_);
        std::ifstream fs(path);
        int val = 1;

        if (fs.is_open()) {
            fs >> val;
            if (val == 0 && !gpio_triggered_) {
                gpio_triggered_ = true;
                g_timer_enabled.store(true);
                RCLCPP_INFO(get_logger(), "GPIO%d low level detected, timer enabled", gpio_reset_);
                zone_entry_time_ = now_tp;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("gpio"), "Failed to open gpio%d value file", gpio_reset_);
        }

        g_timer_enabled.store(gpio_triggered_);

        std::chrono::duration<double> dt_duration = now_tp - last_control_time_;
        double dt = dt_duration.count();
        last_control_time_ = now_tp;

        TargetInfo info;
        {
            std::lock_guard<std::mutex> lock(g_target_mutex);
            info = g_target;
        }

        if (info.detected) {
            last_seen_time_ = now_tp;
            filt_cx_ = ema_filter(filt_cx_, info.x, (float)meas_smooth_alpha_);
            filt_cy_ = ema_filter(filt_cy_, info.y, (float)meas_smooth_alpha_);

            int area = info.bbox.area();
            double kp_pan  = kp_pan_base_  * (1.0 - std::min(std::max((area - 200) / 10000.0, 0.0), 0.9));
            double kp_tilt = kp_tilt_base_ * (1.0 - std::min(std::max((area - 200) / 10000.0, 0.0), 0.9));
            pid_pan_.set_kp(kp_pan);
            pid_tilt_.set_kp(kp_tilt);

            cv::Point2i center(static_cast<int>(CAM_CX), static_cast<int>(CAM_CY));
            bool center_in_bbox = false;

            if (!info.bbox.empty()) {
                int m = static_cast<int>(g_inner_margin_px);
                cv::Rect inner = info.bbox;
                inner.x += m;
                inner.y += m;
                inner.width  = std::max(inner.width  - 2*m, 0);
                inner.height = std::max(inner.height - 2*m, 0);
                center_in_bbox = inner.contains(center);
            }

            if (gpio_triggered_) {
                if (center_in_bbox) {
                    if (!in_target_zone_ || g_first_entry_after_ready.load()) {
                        in_target_zone_ = true;
                        zone_entry_time_ = now_tp;
                        g_first_entry_after_ready.store(false);
                    } else {
                        std::chrono::duration<double> duration = now_tp - zone_entry_time_;
                        g_timer_duration.store(duration.count());
                        if (duration.count() >= 2.1) {
                            if (gpio_exported_) {
                                writeGpio(GPIO3_B5, 0);
                                writeGpio(GPIO1_A2, 1);
                            }
                        }
                    }
                } else {
                    in_target_zone_ = false;
                    g_first_entry_after_ready.store(false);
                    if (gpio_exported_) {
                        writeGpio(GPIO3_B5, 1);
                        writeGpio(GPIO1_A2, 0);
                    }
                }
            } else {
                in_target_zone_ = false;
                g_timer_duration.store(0.0);
            }

            double err_x = filt_cx_ - CAM_CX;
            double err_y = filt_cy_ - CAM_CY;

            if (std::abs(err_x) < deadzone_px_) err_x = 0.0;
            if (std::abs(err_y) < deadzone_px_) err_y = 0.0;

            double cmd_pan  = pid_pan_.update(err_x, dt);
            double cmd_tilt = pid_tilt_.update(err_y, dt);

            cmd_prev_pan_  = ema_filter((float)cmd_prev_pan_,  (float)cmd_pan,  (float)cmd_smooth_alpha_);
            cmd_prev_tilt_ = ema_filter((float)cmd_prev_tilt_, (float)cmd_tilt, (float)cmd_smooth_alpha_);

            dm_.control_vel(*m_horizontal_, -cmd_prev_pan_);
            dm_.control_vel(*m_vertical_,    cmd_prev_tilt_);

            publish_marker(err_x, err_y);
            return;
        }

        in_target_zone_ = false;
        g_first_entry_after_ready.store(false);
        if (gpio_exported_) {
            writeGpio(GPIO3_B5, 1);
            writeGpio(GPIO1_A2, 0);
        }
        cmd_prev_pan_ = 0.0;
        cmd_prev_tilt_ = 0.0;
        dm_.control_vel(*m_horizontal_, 0.0);
        dm_.control_vel(*m_vertical_,   0.0);
        filt_cx_ = CAM_CX;
        filt_cy_ = CAM_CY;
        pid_pan_.reset();
        pid_tilt_.reset();
        publish_marker(NAN, NAN);
    }

    void publish_marker(double err_x, double err_y) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = now();
        m.ns = "target";
        m.id = 0;
        m.type = m.SPHERE;
        m.action = m.ADD;
        if (std::isnan(err_x) || std::isnan(err_y)) {
            m.pose.position.x = 0;
            m.pose.position.y = 0;
        } else {
            m.pose.position.x = err_x;
            m.pose.position.y = err_y;
        }
        m.pose.position.z = 0;
        m.scale.x = m.scale.y = m.scale.z = 0.05;
        m.color.a = 1.0;
        m.color.g = 1.0;
        marker_pub_->publish(m);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<SerialPort> serial_;
    damiao::Motor_Control dm_;
    std::shared_ptr<damiao::Motor> m_horizontal_, m_vertical_;

    PIDController pid_pan_, pid_tilt_;
    double max_vel_;
    double cmd_prev_pan_, cmd_prev_tilt_;
    double filt_cx_, filt_cy_;
    double meas_smooth_alpha_, cmd_smooth_alpha_;
    double deadzone_px_;
    std::chrono::steady_clock::time_point last_seen_time_;
    std::chrono::steady_clock::time_point last_control_time_;
};

// ====================== 主函数 ======================
int main(int argc, char** argv) {
    std::thread t(yolo_thread);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalYoloNode>());
    rclcpp::shutdown();
    t.join();
    return 0;
}