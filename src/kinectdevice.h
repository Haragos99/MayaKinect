#pragma once
#include <Windows.h>    // Required for NuiApi.h
#include <NuiApi.h>
#include <maya/MApiNamespace.h>
#include <maya/MVector.h>
#include <maya/MQuaternion.h>
#include <maya/MMatrix.h>
#include <array>
#include <vector>
#include <map>
#include <math.h>
#include <maya/MEulerRotation.h>
#include <maya/MAngle.h>
#define IDC_VIDEOVIEW           1003


struct JointData
{
	Vector4 positionKinect; // Original Kinect position (homogeneous coordinates)
    MVector position;
    MQuaternion orientation;
    MQuaternion localQuaternion;
    MMatrix rotation;
    MAngle eulerX;
    MAngle eulerY;
    MAngle eulerZ;
    Vector4 quat;
};

struct KinectFrame
{
    std::array<JointData, 20> joints;
    std::vector<unsigned char> colorBuffer;
    int width;
    int height;
    double timestamp;
};


const double PI = 3.14159265358979323846;

class KinectDevice
{
public:
    KinectDevice() = default;
    HRESULT initialize();
    void shutdownKinect();
    std::map<int, std::string>& getKinectToMayaMap() { return kinectToMayaMap; }
    void processSkeleton();
    void creatSkeleton(NUI_SKELETON_DATA& skel);
    std::vector<JointData>& getLatestJoints();
    bool getFrame(unsigned char* outBuffer);
    MEulerRotation getEulers(int idx);
    void calculateQuaternion();
    void initoffsets();
    static const int parent_joint_map[NUI_SKELETON_POSITION_COUNT];
    std::vector<MVector>& getOffsets() { return offsets; }
    ~KinectDevice();

private:
    // Kinect SDK handles
    INuiSensor* NuiSensor;
    HANDLE NextSkeletonEvent;
    HANDLE NextColorEvent;
    HANDLE hColorStream;
    HANDLE SkeletonStreamHandle;
    HWND  m_hWnd;
    std::vector<JointData> joints;
    void drawLine(unsigned char* buffer, int x1, int y1, int x2, int y2, int thickness);
    void drawPixel(unsigned char* buffer, int x, int y, unsigned char r, unsigned char g, unsigned char b, int radius);
    MMatrix convertToMayaMatrix(const Matrix4& kMat);
    // Maps each joint index to its parent joint index
    // Matches Kinect's skeleton hierarchy exactly
    
    // Stores last stable cross product per joint to prevent gimbal/instability
    MVector last_stable_vx[NUI_SKELETON_POSITION_COUNT];

    std::vector<MVector> offsets;

    // helpers
    MVector   buildAxis(const MVector& a, const MVector& b);
    MMatrix   matFromAxes(const MVector& vx, const MVector& vy, const MVector& vz);
    MMatrix   bindPoseInverse(float rx, float ry, float rz);
    MQuaternion matToQuat(const MMatrix& m);


    // Map Kinect v1 Indices to your Maya joint names
    std::map<int, std::string> kinectToMayaMap = {
        { NUI_SKELETON_POSITION_HIP_CENTER,      "SpineBase" },
        { NUI_SKELETON_POSITION_SPINE,           "SpineMid" },
        { NUI_SKELETON_POSITION_SHOULDER_CENTER, "SpineShoulder" },
        { NUI_SKELETON_POSITION_HEAD,            "Head" },
        { NUI_SKELETON_POSITION_SHOULDER_LEFT,   "ShoulderLeft" },
        { NUI_SKELETON_POSITION_ELBOW_LEFT,      "ElbowLeft" },
        { NUI_SKELETON_POSITION_WRIST_LEFT,      "WristLeft" },
        { NUI_SKELETON_POSITION_HAND_LEFT,       "HandLeft" },
        { NUI_SKELETON_POSITION_SHOULDER_RIGHT,  "ShoulderRight" },
        { NUI_SKELETON_POSITION_ELBOW_RIGHT,     "ElbowRight" },
        { NUI_SKELETON_POSITION_WRIST_RIGHT,     "WristRight" },
        { NUI_SKELETON_POSITION_HAND_RIGHT,      "HandRight" },
        { NUI_SKELETON_POSITION_HIP_LEFT,        "HipLeft" },
        { NUI_SKELETON_POSITION_KNEE_LEFT,       "KneeLeft" },
        { NUI_SKELETON_POSITION_ANKLE_LEFT,      "AnkleLeft" },
        { NUI_SKELETON_POSITION_FOOT_LEFT,       "FootLeft" },
        { NUI_SKELETON_POSITION_HIP_RIGHT,       "HipRight" },
        { NUI_SKELETON_POSITION_KNEE_RIGHT,      "KneeRight" },
        { NUI_SKELETON_POSITION_ANKLE_RIGHT,     "AnkleRight" },
        { NUI_SKELETON_POSITION_FOOT_RIGHT,      "FootRight" }
    };

};
