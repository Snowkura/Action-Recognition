// Command-line user interface
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include<Windows.h>
#include<process.h>
#include<stdio.h>
#include<jwvehicle_wrapper.h>
#include "yolo.h"
#include<opencv2//opencv.hpp>
#include<math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

using namespace std;
using namespace cv;
using namespace dnn;


class UserOutputClass : public op::Worker<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
private:
    JWVehicle vehicle;
    bool top, mid, down, front, side;
    //action是长度为2的数组，第一位数字的可能取值为0，1和2，第二位数字的可能取值为0和1
    //第一位数字为0表示左手处于自然放下，1表示左手处于和地面平行的举平状态，2表示高举左手超过头顶
    //第二位数字为0表示正面朝向摄像头，1表示侧面朝向摄像头
    vector<int> action;
    vector<int> lastAction;
    Yolov5 yoloDetector;
    Net yoloNet;
    vector<Scalar> colors;

public:
    UserOutputClass(const std::string& comPort) : vehicle("\\\\.\\COM5"), top(false), mid(false), down(false),
        front(false), side(false), action({ 0, 0 }), lastAction({ 0, 0 }) {
        // 加载YOLO模型
        string modelPath = "./models/best.onnx";
        if (yoloDetector.readModel(yoloNet, modelPath, false)) {
            cout << "Model loaded successfully!" << endl;
        }
        else {
            cerr << "Failed to load the model!" << endl;
            exit(-1);
        }

        // 生成随机颜色
        srand(time(0));
        for (int i = 0; i < 80; i++) {
            int b = rand() % 256;
            int g = rand() % 256;
            int r = rand() % 256;
            colors.push_back(Scalar(b, g, r));
        }
    }

    void initializationOnThread() override {
        vehicle.move(0, 0); // 初始化机器人的位置
        lastAction = action;
    }

    void work(shared_ptr<vector<shared_ptr<op::Datum>>>& datumsPtr) override {
             
        if (datumsPtr != nullptr && !datumsPtr->empty()) {
            
            for (const auto& datum : *datumsPtr) {

                Mat inputImage = OP_OP2CVCONSTMAT(datum->cvInputData);
                vector<Output> result;
                if (yoloDetector.Detect(inputImage, yoloNet, result)) {
                    Rect boxes = result[0].box;
                    //矩形边界框的左下角和右下角的坐标
                    int xLeftDown = boxes.x;
                    int yLeftDown = (boxes.y + boxes.height) * -1;
                    int xRightDown = boxes.x + boxes.width;
                    int yRightDown = (boxes.y + boxes.height) * -1;
                    //矩形边界框的左上角和右上角的坐标
                    int xLeftUp = boxes.x;
                    int yLeftUp = boxes.y * -1;
                    int xRightUp = boxes.x + boxes.width;
                    int yRightUp = boxes.y * -1;
                    const auto& poseKeypoints = datum->poseKeypoints;
                    vector<double> findMin;
                    //遍历当前帧中openpose检测到的所有人
                    for (int person = 0; person < poseKeypoints.getSize(0); person++)
                    {
                        const auto xRightHand = poseKeypoints[{person, 4, 0}];
                        const auto yRightHand = poseKeypoints[{person, 4, 1}] * -1;
                        const auto xLeftHand = poseKeypoints[{person, 7, 0}];
                        const auto yLeftHand = poseKeypoints[{person, 7, 1}] * -1;

                        vector<double> vec;
                        //边界框左下角的距离到人的左手的距离
                        auto distance1 = sqrt((xLeftDown - xLeftHand) * (xLeftDown - xLeftHand) + (yLeftDown - yLeftHand) * (yLeftDown - yLeftHand));
                        vec.push_back(distance1);
                        //边界框右下角的距离到人的左手的距离
                        auto distance2 = sqrt((xRightDown - xLeftHand) * (xRightDown - xLeftHand) + (yRightDown - yLeftHand) * (yRightDown - yLeftHand));
                        vec.push_back(distance2);
                        //边界框左上角的距离到人的左手的距离
                        auto distance3 = sqrt((xLeftUp - xLeftHand) * (xLeftUp - xLeftHand) + (yLeftUp - yLeftHand) * (yLeftUp - yLeftHand));
                        vec.push_back(distance3);
                        //边界框右上角的距离到人的左手的距离
                        auto distance4 = sqrt((xRightUp - xLeftHand) * (xRightUp - xLeftHand) + (yRightUp - yLeftHand) * (yRightUp - yLeftHand));
                        vec.push_back(distance4);

                        //边界框右下角的距离到人的右手的距离
                        auto distance5 = sqrt((xRightDown - xRightHand) * (xRightDown - xRightHand) + (yRightDown - yRightHand) * (yRightDown - yRightHand));
                        vec.push_back(distance5);
                        //边界框左下角的距离到人的右手的距离
                        auto distance6 = sqrt((xLeftDown - xRightHand) * (xLeftDown - xRightHand) + (yLeftDown - yRightHand) * (yLeftDown - yRightHand));
                        vec.push_back(distance6);
                        //边界框右上角的距离到人的右手的距离
                        auto distance7 = sqrt((xRightUp - xRightHand) * (xRightUp - xRightHand) + (yRightUp - yRightHand) * (yRightUp - yRightHand));
                        vec.push_back(distance7);
                        //边界框左上角的距离到人的右手的距离
                        auto distance8 = sqrt((xLeftUp - xRightHand) * (xLeftUp - xRightHand) + (yLeftUp - yRightHand) * (yLeftUp - yRightHand));
                        vec.push_back(distance8);


                        //找出上述距离中的最小值
                        auto min = min_element(vec.begin(), vec.end());

                        findMin.push_back(*min);
                    }
                    if (!findMin.empty()) {
                        auto min = min_element(findMin.begin(), findMin.end());
                        int minIndex = distance(findMin.begin(), min);
                        const auto xNose = poseKeypoints[{minIndex, 0, 0}];
                        const auto yNose = poseKeypoints[{minIndex, 0, 1}];
                        const auto xNeck = poseKeypoints[{minIndex, 1, 0}];
                        const auto yNeck = poseKeypoints[{minIndex, 1, 1}];
                        auto outputImage = OP_OP2CVCONSTMAT(datum->cvOutputData);
                        int sideLength = sqrt((xNose - xNeck) * (xNose - xNeck) + (yNose - yNeck) * (yNose - yNeck));
                        Point topLeft(xNose - sideLength / 2, yNose - sideLength / 2);
                        Point bottomRight(xNose + sideLength / 2, yNose + sideLength / 2);
                        //在指挥员脸上输出矩形
                        rectangle(outputImage, topLeft, bottomRight, Scalar(0, 255, 255), 3);
                        putText(outputImage, "Traffic Guard", Point(xNose - sideLength / 2, yNose - sideLength / 2 - 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);

                        //对指挥员的动作进行判定                        
                        const auto xLS = poseKeypoints[{minIndex, 5, 0}];
                        const auto yLS = poseKeypoints[{minIndex, 5, 1}] * -1;
                        const auto xRS = poseKeypoints[{minIndex, 2, 0}];
                        const auto yRS = poseKeypoints[{minIndex, 2, 1}] * -1;
                        const auto xRightHand = poseKeypoints[{minIndex, 4, 0}];
                        const auto yRightHand = poseKeypoints[{minIndex, 4, 1}];
                        const auto xLeftHand = poseKeypoints[{minIndex, 7, 0}];
                        const auto yLeftHand = poseKeypoints[{minIndex, 7, 1}];
                        const auto yLeftElbow = poseKeypoints[(minIndex, 6, 1)];
                        const auto yHip = poseKeypoints[{minIndex, 8, 1}];
                        //距离因子，等于腰部到胸部的距离乘以0.2，用于判定手臂是否水平
                        const auto normalized = (yHip - yNeck) * 0.2;
                        //距离因子，等于腰部到胸部的距离乘以0.3，用于判定是否侧身，如果两个肩膀之间的距离小于该值，则表示处于侧身状态
                        const auto sideNormalized = (yHip - yNeck) * 0.3;

                        //双手在胸前交叉则退出程序
                        if (xLeftHand < xRightHand && (yLeftHand >= yNeck - normalized && yLeftHand <= yNeck + normalized) &&
                            (yRightHand >= yNeck - normalized && yRightHand <= yNeck + normalized)) {
                            exit(0);
                        }

                        front = xRightHand != 0 && xLeftHand != 0;
                        if (front) {
                            action[1] = 0;
                        }
                        side = sqrt((xLS - xRS) * (xLS - xRS) + (yLS - yRS) * (yLS - yRS)) < sideNormalized;
                        if (side) {
                            action[1] = 1;
                        }
                        top = yLeftHand < yNose && yLeftHand != 0;
                        if (top) {
                            action[0] = 2;
                        }
                        mid = (yLeftHand >= yNeck - normalized) && (yLeftHand <= yNeck + normalized);
                        if (mid) {
                            action[0] = 1;
                        }
                        down = yLeftHand > yNeck + normalized;
                        if (down) {
                            action[0] = 0;
                        }

                        if (action != lastAction) {
                            //{1，0}表示停止
                            if (action[0] == 1 && action[1] == 0) {
                                vehicle.stop();
                            }
                            //{2, 0}表示减速
                            else if (action[0] == 2 && action[1] == 0) {
                                vehicle.move(30, 0);
                            }
                            //侧面朝向表示前进
                            else if (action[1] == 1) {
                                vehicle.move(50, 0);
                            }
                        }
                        lastAction = action;

                    }
                    Mat outputImage = OP_OP2CVCONSTMAT(datum->cvOutputData);
                    yoloDetector.drawPred(outputImage, result, colors); // 将YOLO检测结果绘制到OpenPose的输出图像上
                }                
            }
        }
    }
};


void configureWrapper(op::Wrapper& opWrapper)
{
    try
    {
        // Configuring OpenPose

        // logging_level
        op::checkBool(
            0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
            __LINE__, __FUNCTION__, __FILE__);
        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

        // Applying user defined configuration - GFlags to program variables
        // producerType
        op::ProducerType producerType;
        op::String producerString;
        std::tie(producerType, producerString) = op::flagsToProducer(
            op::String(FLAGS_image_dir), op::String(FLAGS_video), op::String(FLAGS_ip_camera), FLAGS_camera,
            FLAGS_flir_camera, FLAGS_flir_camera_index);
        // cameraSize
        const auto cameraSize = op::flagsToPoint(op::String(FLAGS_camera_resolution), "-1x-1");
        // outputSize
        const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
        // netInputSize
        const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
        // faceNetInputSize
        const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
        // handNetInputSize
        const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
        // poseMode
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);
        // poseModel
        const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            op::opLog(
                "Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
                " instead.", op::Priority::Max);
        // keypointScaleMode
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
            FLAGS_heatmaps_add_PAFs);
        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
        // >1 camera view?
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
        const op::WrapperStructPose wrapperStructPose{
            poseMode, netInputSize, FLAGS_net_resolution_dynamic, outputSize, keypointScaleMode, FLAGS_num_gpu,
            FLAGS_num_gpu_start, FLAGS_scale_number, (float)FLAGS_scale_gap,
            op::flagsToRenderMode(FLAGS_render_pose, multipleView), poseModel, !FLAGS_disable_blending,
            (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, op::String(FLAGS_model_folder),
            heatMapTypes, heatMapScaleMode, FLAGS_part_candidates, (float)FLAGS_render_threshold,
            FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max, op::String(FLAGS_prototxt_path),
            op::String(FLAGS_caffemodel_path), (float)FLAGS_upsampling_ratio, enableGoogleLogging };
        opWrapper.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };
        opWrapper.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
        opWrapper.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };
        opWrapper.configure(wrapperStructExtra);
        // Producer (use default to disable any input)
        const op::WrapperStructInput wrapperStructInput{
            producerType, producerString, FLAGS_frame_first, FLAGS_frame_step, FLAGS_frame_last,
            FLAGS_process_real_time, FLAGS_frame_flip, FLAGS_frame_rotate, FLAGS_frames_repeat,
            cameraSize, op::String(FLAGS_camera_parameter_path), FLAGS_frame_undistort, FLAGS_3d_views };
        opWrapper.configure(wrapperStructInput);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port) };
        opWrapper.configure(wrapperStructOutput);
        // GUI (comment or use default argument to disable any visual output)
        const op::WrapperStructGui wrapperStructGui{
            op::flagsToDisplayMode(FLAGS_display, FLAGS_3d), !FLAGS_no_gui_verbose, FLAGS_fullscreen };
        opWrapper.configure(wrapperStructGui);
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}


int openPoseDemo()
{
    try
    {
        op::opLog("Starting WheelChair...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configure OpenPose
        op::opLog("Configuring OpenPose...", op::Priority::High);
        op::Wrapper opWrapper;
        configureWrapper(opWrapper);

        // 使用UserOutput回调
        auto userOutput = std::make_shared<UserOutputClass>("\\\\.\\COM5");
        opWrapper.setWorker(op::WorkerType::Output, userOutput);


        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapper.exec();

        // Measuring total time
        op::printTime(opTimer, "WheelChair-OpenPose successfully finished. Total time: ", " seconds.", op::Priority::High);

        // Return successful message
        return 0;
    }
    catch (const std::exception&)
    {
        return -1;
    }
}

int main(int argc, char* argv[])
{
    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Running openPoseDemo
    return openPoseDemo();

}
