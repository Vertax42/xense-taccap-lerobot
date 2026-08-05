#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <thread>
#include <iostream>
#include <mutex>
#include <sstream>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include "PXREARobotSDK.h"

using json = nlohmann::json;
namespace py = pybind11;

std::array<double, 7> LeftControllerPose;
std::array<double, 7> RightControllerPose;
std::array<double, 7> HeadsetPose;

std::array<std::array<double, 7>, 26> LeftHandTrackingState;
double LeftHandScale = 1.0;
int LeftHandIsActive = 0;
std::array<std::array<double, 7>, 26> RightHandTrackingState;
double RightHandScale = 1.0;
int RightHandIsActive = 0;

// Whole body motion data - 24 joints for body tracking
std::array<std::array<double, 7>, 24> BodyJointsPose;  // Position and rotation for each joint
std::array<std::array<double, 6>, 24> BodyJointsVelocity;  // Velocity and angular velocity for each joint
std::array<std::array<double, 6>, 24> BodyJointsAcceleration;  // Acceleration and angular acceleration for each joint
std::array<int64_t, 24> BodyJointsTimestamp;  // IMU timestamp for each joint
int64_t BodyTimeStampNs = 0;  // Body data timestamp
bool BodyDataAvailable = false;  // Flag to indicate if body data is available

std::array<std::array<double, 7>, 3> MotionTrackerPose;  // Position and rotation for each joint
std::array<std::array<double, 6>, 3> MotionTrackerVelocity;  // Velocity and angular velocity for each joint
std::array<std::array<double, 6>, 3> MotionTrackerAcceleration;  // Acceleration and angular acceleration for each joint
std::array<std::string, 3> MotionTrackerSerialNumbers;  // Serial numbers of the motion trackers
int64_t MotionTimeStampNs = 0;  // Motion data timestamp
int NumMotionDataAvailable = 0;  // number of motion trackers


bool LeftMenuButton;
double LeftTrigger;
double LeftGrip;
std::array<double, 2> LeftAxis{0.0, 0.0};
bool LeftAxisClick;
bool LeftPrimaryButton;
bool LeftSecondaryButton;

bool RightMenuButton;
double RightTrigger;
double RightGrip;
std::array<double, 2> RightAxis{0.0, 0.0};
bool RightAxisClick;
bool RightPrimaryButton;
bool RightSecondaryButton;

int64_t TimeStampNs;

std::mutex leftMutex;
std::mutex rightMutex;
std::mutex headsetPoseMutex;
std::mutex timestampMutex;
std::mutex leftHandMutex;
std::mutex rightHandMutex;
std::mutex bodyMutex;  // Mutex for body tracking data
std::mutex motionMutex;
std::mutex cameraMutex;

struct PicoCameraFrame {
    bool valid = false;
    std::string deviceId;
    int eyeIndex = -1;
    int width = 0;
    int height = 0;
    uint32_t frameSequence = 0;
    uint64_t timestampNs = 0;
    std::vector<uint8_t> jpegBytes;
};

std::array<PicoCameraFrame, 2> PicoCameraFrames;

template <typename T>
T readUnalignedValue(const uint8_t* data) {
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

bool cachePicoCameraFrame(const char* data, uint64_t size) {
    if (data == nullptr || size < 1 + 1 + 2 + 2 + 4 + 8 + 2) {
        return false;
    }

    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    uint64_t offset = 0;

    const uint8_t deviceIdLen = bytes[offset++];
    if (deviceIdLen == 0 || offset + deviceIdLen + 1 + 2 + 2 + 4 + 8 + 2 > size) {
        return false;
    }

    std::string deviceId(reinterpret_cast<const char*>(bytes + offset), deviceIdLen);
    offset += deviceIdLen;

    const uint8_t eyeIndex = bytes[offset++];
    if (eyeIndex > 1) {
        return false;
    }

    const uint16_t width = readUnalignedValue<uint16_t>(bytes + offset);
    offset += sizeof(uint16_t);
    const uint16_t height = readUnalignedValue<uint16_t>(bytes + offset);
    offset += sizeof(uint16_t);
    const uint32_t frameSequence = readUnalignedValue<uint32_t>(bytes + offset);
    offset += sizeof(uint32_t);
    const uint64_t timestampNs = readUnalignedValue<uint64_t>(bytes + offset);
    offset += sizeof(uint64_t);

    const uint64_t jpegSize = size - offset;
    if (width == 0 || height == 0 || jpegSize < 2 || bytes[offset] != 0xFF || bytes[offset + 1] != 0xD8) {
        return false;
    }

    PicoCameraFrame frame;
    frame.valid = true;
    frame.deviceId = std::move(deviceId);
    frame.eyeIndex = eyeIndex;
    frame.width = width;
    frame.height = height;
    frame.frameSequence = frameSequence;
    frame.timestampNs = timestampNs;
    frame.jpegBytes.assign(bytes + offset, bytes + size);

    std::lock_guard<std::mutex> lock(cameraMutex);
    PicoCameraFrames[eyeIndex] = std::move(frame);
    return true;
}

std::array<double, 7> stringToPoseArray(const std::string& poseStr) {
    std::array<double, 7> result{0};
    std::stringstream ss(poseStr);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < 7) {
        result[i++] = std::stod(value);
    }
    return result;
}

std::array<double, 6> stringToVelocityArray(const std::string& velocityStr) {
    std::array<double, 6> result{0};
    std::stringstream ss(velocityStr);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < 6) {
        result[i++] = std::stod(value);
    }
    return result;
}

void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData)
{
    switch (type)
    {
    case PXREAServerConnect:
        std::cout << "server connect\n" << std::endl;
        break;
    case PXREAServerDisconnect:
        std::cout << "server disconnect\n" << std::endl;
        break;
    case PXREADeviceFind:
        std::cout << "device found\n" << (const char*)userData << std::endl;
        break;
    case PXREADeviceMissing:
        std::cout << "device missing\n" << (const char*)userData << std::endl;
        break;
    case PXREADeviceConnect:
        std::cout << "device connect\n" << (const char*)userData << status << std::endl;
        break;
    case PXREADeviceStateJson:
    {
        auto& dsj = *((PXREADevStateJson*)userData);
        try {
            json data = json::parse(dsj.stateJson);
            if (data.contains("value")) {
                auto value = json::parse(data["value"].get<std::string>());
                if (value["Controller"].contains("left")) {
                    auto& left = value["Controller"]["left"];
                    {
                        std::lock_guard<std::mutex> lock(leftMutex);
                        LeftControllerPose = stringToPoseArray(left["pose"].get<std::string>());
                        LeftTrigger = left["trigger"].get<double>();
                        LeftGrip = left["grip"].get<double>();
                        LeftMenuButton = left["menuButton"].get<bool>();
                        LeftAxis[0] = left["axisX"].get<double>();
                        LeftAxis[1] = left["axisY"].get<double>();
                        LeftAxisClick = left["axisClick"].get<bool>();
                        LeftPrimaryButton = left["primaryButton"].get<bool>();
                        LeftSecondaryButton = left["secondaryButton"].get<bool>();
                    }
                }
                if (value["Controller"].contains("right")) {
                    auto& right = value["Controller"]["right"];
                    {
                        std::lock_guard<std::mutex> lock(rightMutex);
                        RightControllerPose = stringToPoseArray(right["pose"].get<std::string>());
                        RightTrigger = right["trigger"].get<double>();
                        RightGrip = right["grip"].get<double>();
                        RightMenuButton = right["menuButton"].get<bool>();
                        RightAxis[0] = right["axisX"].get<double>();
                        RightAxis[1] = right["axisY"].get<double>();
                        RightAxisClick = right["axisClick"].get<bool>();
                        RightPrimaryButton = right["primaryButton"].get<bool>();
                        RightSecondaryButton = right["secondaryButton"].get<bool>();
                    }
                }
                if (value.contains("Head")) {
                    auto& headset = value["Head"];
                    {
                        std::lock_guard<std::mutex> lock(headsetPoseMutex);
                        HeadsetPose = stringToPoseArray(headset["pose"].get<std::string>());
                    }
                }
                if (value.contains("timeStampNs")) {
                    std::lock_guard<std::mutex> lock(timestampMutex);
                    TimeStampNs = value["timeStampNs"].get<int64_t>();
                }
                if (value["Hand"].contains("leftHand")) {
                    auto& leftHand = value["Hand"]["leftHand"];
                    {
                        std::lock_guard<std::mutex> lock(leftHandMutex);

                        LeftHandScale = leftHand["scale"].get<double>();
                        LeftHandIsActive = leftHand["isActive"].get<int>();
                        for (int i = 0; i < 26; i++) {
                            LeftHandTrackingState[i] = stringToPoseArray(leftHand["HandJointLocations"][i]["p"].get<std::string>());
                        }
                    }
                }
                if (value["Hand"].contains("rightHand")) {
                    auto& rightHand = value["Hand"]["rightHand"];
                    {
                        std::lock_guard<std::mutex> lock(rightHandMutex);
                        RightHandScale = rightHand["scale"].get<double>();
                        RightHandIsActive = rightHand["isActive"].get<int>();
                        for (int i = 0; i < 26; i++) {
                            RightHandTrackingState[i] = stringToPoseArray(rightHand["HandJointLocations"][i]["p"].get<std::string>());
                        }
                    }
                }
                // Parse Body data for whole body motion capture
                if (value.contains("Body")) {
                    auto& body = value["Body"];
                    {
                        std::lock_guard<std::mutex> lock(bodyMutex);

                        if (body.contains("timeStampNs")) {
                            BodyTimeStampNs = body["timeStampNs"].get<int64_t>();
                        }

                        if (body.contains("joints") && body["joints"].is_array()) {
                            auto joints = body["joints"];
                            int jointCount = std::min(static_cast<int>(joints.size()), 24);

                            for (int i = 0; i < jointCount; i++) {
                                auto& joint = joints[i];

                                // Parse pose (position and rotation)
                                if (joint.contains("p")) {
                                    BodyJointsPose[i] = stringToPoseArray(joint["p"].get<std::string>());
                                }

                                // Parse velocity and angular velocity
                                if (joint.contains("va")) {
                                    BodyJointsVelocity[i] = stringToVelocityArray(joint["va"].get<std::string>());
                                }

                                // Parse acceleration and angular acceleration
                                if (joint.contains("wva")) {
                                    BodyJointsAcceleration[i] = stringToVelocityArray(joint["wva"].get<std::string>());
                                }

                                // Parse IMU timestamp
                                if (joint.contains("t")) {
                                    BodyJointsTimestamp[i] = joint["t"].get<int64_t>();
                                }
                            }

                            BodyDataAvailable = true;
                        }
                    }
                }
                //parse individual tracker data
                if (value.contains("Motion")) {
                    auto& motion = value["Motion"];
                    {
                        std::lock_guard<std::mutex> lock(motionMutex);
                        if (motion.contains("timeStampNs")) {
                            MotionTimeStampNs = motion["timeStampNs"].get<int64_t>();
                        }
                        if (motion.contains("joints") && motion["joints"].is_array()) {
                            auto joints = motion["joints"];
                            NumMotionDataAvailable = std::min(static_cast<int>(joints.size()), 3);

                            for (int i = 0; i < NumMotionDataAvailable; i++) {
                                auto& joint = joints[i];

                                // Parse pose (position and rotation)
                                if (joint.contains("p")) {
                                    MotionTrackerPose[i] = stringToPoseArray(joint["p"].get<std::string>());
                                }

                                // Parse velocity and angular velocity
                                if (joint.contains("va")) {
                                    MotionTrackerVelocity[i] = stringToVelocityArray(joint["va"].get<std::string>());
                                }

                                // Parse acceleration and angular acceleration
                                if (joint.contains("wva")) {
                                    MotionTrackerAcceleration[i] = stringToVelocityArray(joint["wva"].get<std::string>());
                                }

                                if (joint.contains("sn")) {
                                    MotionTrackerSerialNumbers[i] = joint["sn"].get<std::string>();
                                }
                            }

                        }
                    }
                }
            }
        } catch (const json::exception& e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
        }
            break;
    }
    case PXREADeviceCustomMessage:
    {
        auto& dcm = *((PXREADevCustomMessage*)userData);
        cachePicoCameraFrame(dcm.dataPtr, dcm.dataSize);
        break;
    }
    }
}

void init() {
    if (PXREAInit(NULL, OnPXREAClientCallback, PXREAFullMask) != 0) {
        throw std::runtime_error("PXREAInit failed");
    }
}

void deinit() {
    PXREADeinit();
}

std::array<double, 7> getLeftControllerPose() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftControllerPose;
}

std::array<double, 7> getRightControllerPose() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightControllerPose;
}

std::array<double, 7> getHeadsetPose() {
    std::lock_guard<std::mutex> lock(headsetPoseMutex);
    return HeadsetPose;
}

double getLeftTrigger() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftTrigger;
}

double getLeftGrip() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftGrip;
}

double getRightTrigger() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightTrigger;
}

double getRightGrip() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightGrip;
}

bool getLeftMenuButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftMenuButton;
}

bool getRightMenuButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightMenuButton;
}

bool getLeftAxisClick() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftAxisClick;
}

bool getRightAxisClick() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightAxisClick;
}

std::array<double, 2> getLeftAxis() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftAxis;
}


std::array<double, 2> getRightAxis() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightAxis;
}

bool getLeftPrimaryButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftPrimaryButton;
}

bool getRightPrimaryButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightPrimaryButton;
}

bool getLeftSecondaryButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftSecondaryButton;
}

bool getRightSecondaryButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightSecondaryButton;
}

int64_t getTimeStampNs() {
    std::lock_guard<std::mutex> lock(timestampMutex);
    return TimeStampNs;
}

std::array<std::array<double, 7>, 26> getLeftHandTrackingState() {
    std::lock_guard<std::mutex> lock(leftHandMutex);
    return LeftHandTrackingState;
}

int getLeftHandScale() {
    std::lock_guard<std::mutex> lock(leftHandMutex);
    return LeftHandScale;
}

int getLeftHandIsActive() {
    std::lock_guard<std::mutex> lock(leftHandMutex);
    return LeftHandIsActive;
}

std::array<std::array<double, 7>, 26> getRightHandTrackingState() {
    std::lock_guard<std::mutex> lock(rightHandMutex);
    return RightHandTrackingState;
}

int getRightHandScale() {
    std::lock_guard<std::mutex> lock(rightHandMutex);
    return RightHandScale;
}

int getRightHandIsActive() {
    std::lock_guard<std::mutex> lock(rightHandMutex);
    return RightHandIsActive;
}

// Body tracking functions
bool isBodyDataAvailable() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyDataAvailable;
}

std::array<std::array<double, 7>, 24> getBodyJointsPose() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyJointsPose;
}

std::array<std::array<double, 6>, 24> getBodyJointsVelocity() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyJointsVelocity;
}

std::array<std::array<double, 6>, 24> getBodyJointsAcceleration() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyJointsAcceleration;
}

std::array<int64_t, 24> getBodyJointsTimestamp() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyJointsTimestamp;
}

int64_t getBodyTimeStampNs() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyTimeStampNs;
}

int numMotionDataAvailable() {
    std::lock_guard<std::mutex> lock(motionMutex);
    return NumMotionDataAvailable;
}

std::vector<std::array<double, 7>> getMotionTrackerPose() {
    std::lock_guard<std::mutex> lock(motionMutex);
    std::vector<std::array<double, 7>> result;
    for (int i = 0; i < NumMotionDataAvailable; i++) {
        result.push_back(MotionTrackerPose[i]);
    }
    return result;
}

std::vector<std::array<double, 6>> getMotionTrackerVelocity() {
    std::lock_guard<std::mutex> lock(motionMutex);
    std::vector<std::array<double, 6>> result;
    for (int i = 0; i < NumMotionDataAvailable; i++) {
        result.push_back(MotionTrackerVelocity[i]);
    }
    return result;
}

std::vector<std::array<double, 6>> getMotionTrackerAcceleration() {
    std::lock_guard<std::mutex> lock(motionMutex);
    std::vector<std::array<double, 6>> result;
    for (int i = 0; i < NumMotionDataAvailable; i++) {
        result.push_back(MotionTrackerAcceleration[i]);
    }
    return result;
}

std::vector<std::string> getMotionTrackerSerialNumbers() {
    std::lock_guard<std::mutex> lock(motionMutex);
    std::vector<std::string> result;
    for (int i = 0; i < NumMotionDataAvailable; i++) {
        result.push_back(MotionTrackerSerialNumbers[i]);
    }
    return result;
}

int64_t getMotionTimeStampNs() {
    std::lock_guard<std::mutex> lock(motionMutex);
    return MotionTimeStampNs;
}

bool hasPicoCameraFrame(int eyeIndex) {
    if (eyeIndex < 0 || eyeIndex >= static_cast<int>(PicoCameraFrames.size())) {
        return false;
    }

    std::lock_guard<std::mutex> lock(cameraMutex);
    return PicoCameraFrames[eyeIndex].valid;
}

py::dict getPicoCameraFrameMetadata(int eyeIndex) {
    if (eyeIndex < 0 || eyeIndex >= static_cast<int>(PicoCameraFrames.size())) {
        throw std::out_of_range("Pico camera eye index must be 0 (left) or 1 (right).");
    }

    std::lock_guard<std::mutex> lock(cameraMutex);
    const PicoCameraFrame& frame = PicoCameraFrames[eyeIndex];
    if (!frame.valid) {
        throw std::runtime_error("No Pico camera frame has been received for the requested eye.");
    }

    py::dict metadata;
    metadata["device_id"] = frame.deviceId;
    metadata["eye_index"] = frame.eyeIndex;
    metadata["width"] = frame.width;
    metadata["height"] = frame.height;
    metadata["frame_sequence"] = frame.frameSequence;
    metadata["timestamp_ns"] = frame.timestampNs;
    metadata["jpeg_size"] = frame.jpegBytes.size();
    return metadata;
}

py::bytes getPicoCameraFrameJpeg(int eyeIndex) {
    if (eyeIndex < 0 || eyeIndex >= static_cast<int>(PicoCameraFrames.size())) {
        throw std::out_of_range("Pico camera eye index must be 0 (left) or 1 (right).");
    }

    std::lock_guard<std::mutex> lock(cameraMutex);
    const PicoCameraFrame& frame = PicoCameraFrames[eyeIndex];
    if (!frame.valid || frame.jpegBytes.empty()) {
        throw std::runtime_error("No Pico camera JPEG has been received for the requested eye.");
    }

    return py::bytes(reinterpret_cast<const char*>(frame.jpegBytes.data()), frame.jpegBytes.size());
}

py::dict getPicoCameraFrame(int eyeIndex) {
    if (eyeIndex < 0 || eyeIndex >= static_cast<int>(PicoCameraFrames.size())) {
        throw std::out_of_range("Pico camera eye index must be 0 (left) or 1 (right).");
    }

    std::lock_guard<std::mutex> lock(cameraMutex);
    const PicoCameraFrame& frame = PicoCameraFrames[eyeIndex];
    if (!frame.valid || frame.jpegBytes.empty()) {
        throw std::runtime_error("No Pico camera frame has been received for the requested eye.");
    }

    py::dict result;
    result["device_id"] = frame.deviceId;
    result["eye_index"] = frame.eyeIndex;
    result["width"] = frame.width;
    result["height"] = frame.height;
    result["frame_sequence"] = frame.frameSequence;
    result["timestamp_ns"] = frame.timestampNs;
    result["jpeg"] = py::bytes(reinterpret_cast<const char*>(frame.jpegBytes.data()), frame.jpegBytes.size());
    return result;
}

py::dict getLeftPicoCameraFrame() {
    return getPicoCameraFrame(0);
}

py::dict getRightPicoCameraFrame() {
    return getPicoCameraFrame(1);
}


PYBIND11_MODULE(xensevr_pc_service_sdk, m) {
    m.def("init", &init, "Initialize the PXREARobot SDK.");
    m.def("close", &deinit, "Deinitialize the PXREARobot SDK.");
    m.def("get_left_controller_pose", &getLeftControllerPose, "Get the left controller pose.");
    m.def("get_right_controller_pose", &getRightControllerPose, "Get the right controller pose.");
    m.def("get_headset_pose", &getHeadsetPose, "Get the headset pose.");
    m.def("get_left_trigger", &getLeftTrigger, "Get the left trigger value.");
    m.def("get_left_grip", &getLeftGrip, "Get the left grip value.");
    m.def("get_right_trigger", &getRightTrigger, "Get the right trigger value.");
    m.def("get_right_grip", &getRightGrip, "Get the right grip value.");
    m.def("get_left_menu_button", &getLeftMenuButton, "Get the left menu button state.");
    m.def("get_right_menu_button", &getRightMenuButton, "Get the right menu button state.");
    m.def("get_left_axis_click", &getLeftAxisClick, "Get the left axis click state.");
    m.def("get_right_axis_click", &getRightAxisClick, "Get the right axis click state.");
    m.def("get_left_axis", &getLeftAxis, "Get the left axis values (x, y).");
    m.def("get_right_axis", &getRightAxis, "Get the right axis values (x, y).");
    m.def("get_X_button", &getLeftPrimaryButton, "Get the left primary button state.");
    m.def("get_A_button", &getRightPrimaryButton, "Get the right primary button state.");
    m.def("get_Y_button", &getLeftSecondaryButton, "Get the left secondary button state.");
    m.def("get_B_button", &getRightSecondaryButton, "Get the right secondary button state.");
    m.def("get_time_stamp_ns", &getTimeStampNs, "Get the timestamp in nanoseconds.");
    m.def("get_left_hand_tracking_state", &getLeftHandTrackingState, "Get the left hand state.");
    m.def("get_right_hand_tracking_state", &getRightHandTrackingState, "Get the right hand state.");
    m.def("get_left_hand_is_active", &getLeftHandIsActive, "Get the left hand tracking quality (0 = low, 1 = high).");
    m.def("get_right_hand_is_active", &getRightHandIsActive, "Get the right hand tracking quality (0 = low, 1 = high).");

    // Body tracking functions
    m.def("is_body_data_available", &isBodyDataAvailable, "Check if body tracking data is available.");
    m.def("get_body_joints_pose", &getBodyJointsPose, "Get the body joints pose data (24 joints, 7 values each: x,y,z,qx,qy,qz,qw).");
    m.def("get_body_joints_velocity", &getBodyJointsVelocity, "Get the body joints velocity data (24 joints, 6 values each: vx,vy,vz,wx,wy,wz).");
    m.def("get_body_joints_acceleration", &getBodyJointsAcceleration, "Get the body joints acceleration data (24 joints, 6 values each: ax,ay,az,wax,way,waz).");
    m.def("get_body_joints_timestamp", &getBodyJointsTimestamp, "Get the body joints IMU timestamp data (24 joints).");
    m.def("get_body_timestamp_ns", &getBodyTimeStampNs, "Get the body data timestamp in nanoseconds.");

    // Motion tracker functions
    m.def("num_motion_data_available", &numMotionDataAvailable, "Check if motion tracker data is available.");
    m.def("get_motion_tracker_pose", &getMotionTrackerPose, "Get the motion tracker pose data (3 trackers, 7 values each: x,y,z,qx,qy,qz,qw).");
    m.def("get_motion_tracker_velocity", &getMotionTrackerVelocity, "Get the motion tracker velocity data (3 trackers, 6 values each: vx,vy,vz,wx,wy,wz).");
    m.def("get_motion_tracker_acceleration", &getMotionTrackerAcceleration, "Get the motion tracker acceleration data (3 trackers, 6 values each: ax,ay,az,wax,way,waz).");
    m.def("get_motion_tracker_serial_numbers", &getMotionTrackerSerialNumbers, "Get the serial numbers of the motion trackers.");
    m.def("get_motion_timestamp_ns", &getMotionTimeStampNs, "Get the motion data timestamp in nanoseconds.");

    // Pico eye camera functions. eye_index: 0 = left, 1 = right.
    m.def("has_pico_camera_frame", &hasPicoCameraFrame, "Check whether a Pico camera frame has been received for an eye.");
    m.def("get_pico_camera_frame_metadata", &getPicoCameraFrameMetadata, "Get metadata for the latest Pico camera frame for an eye.");
    m.def("get_pico_camera_frame_jpeg", &getPicoCameraFrameJpeg, "Get the latest Pico camera JPEG bytes for an eye.");
    m.def("get_pico_camera_frame", &getPicoCameraFrame, "Get metadata and JPEG bytes for the latest Pico camera frame for an eye.");
    m.def("get_left_pico_camera_frame", &getLeftPicoCameraFrame, "Get metadata and JPEG bytes for the latest left-eye Pico camera frame.");
    m.def("get_right_pico_camera_frame", &getRightPicoCameraFrame, "Get metadata and JPEG bytes for the latest right-eye Pico camera frame.");

    m.doc() = "Python bindings for PXREARobot SDK using pybind11.";
}
