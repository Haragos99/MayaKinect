#include "kinectdevice.h"


// Static parent hierarchy — mirrors Kinect's skeleton tree
const int KinectDevice::parent_joint_map[NUI_SKELETON_POSITION_COUNT] =
{
    // Joint                              Parent
    NUI_SKELETON_POSITION_HIP_CENTER,    // 0  HIP_CENTER    -> self (root)
    NUI_SKELETON_POSITION_HIP_CENTER,    // 1  SPINE         -> HIP_CENTER
    NUI_SKELETON_POSITION_SPINE,         // 2  SHOULDER_CENTER -> SPINE
    NUI_SKELETON_POSITION_SHOULDER_CENTER,// 3  HEAD          -> SHOULDER_CENTER
    NUI_SKELETON_POSITION_SHOULDER_CENTER,// 4  SHOULDER_LEFT -> SHOULDER_CENTER
    NUI_SKELETON_POSITION_SHOULDER_LEFT, // 5  ELBOW_LEFT    -> SHOULDER_LEFT
    NUI_SKELETON_POSITION_ELBOW_LEFT,    // 6  WRIST_LEFT    -> ELBOW_LEFT
    NUI_SKELETON_POSITION_WRIST_LEFT,    // 7  HAND_LEFT     -> WRIST_LEFT
    NUI_SKELETON_POSITION_SHOULDER_CENTER,// 8  SHOULDER_RIGHT-> SHOULDER_CENTER
    NUI_SKELETON_POSITION_SHOULDER_RIGHT,// 9  ELBOW_RIGHT   -> SHOULDER_RIGHT
    NUI_SKELETON_POSITION_ELBOW_RIGHT,   // 10 WRIST_RIGHT   -> ELBOW_RIGHT
    NUI_SKELETON_POSITION_WRIST_RIGHT,   // 11 HAND_RIGHT    -> WRIST_RIGHT
    NUI_SKELETON_POSITION_HIP_CENTER,    // 12 HIP_LEFT      -> HIP_CENTER
    NUI_SKELETON_POSITION_HIP_LEFT,      // 13 KNEE_LEFT     -> HIP_LEFT
    NUI_SKELETON_POSITION_KNEE_LEFT,     // 14 ANKLE_LEFT    -> KNEE_LEFT
    NUI_SKELETON_POSITION_ANKLE_LEFT,    // 15 FOOT_LEFT     -> ANKLE_LEFT
    NUI_SKELETON_POSITION_HIP_CENTER,    // 16 HIP_RIGHT     -> HIP_CENTER
    NUI_SKELETON_POSITION_HIP_RIGHT,     // 17 KNEE_RIGHT    -> HIP_RIGHT
    NUI_SKELETON_POSITION_KNEE_RIGHT,    // 18 ANKLE_RIGHT   -> KNEE_RIGHT
    NUI_SKELETON_POSITION_ANKLE_RIGHT,   // 19 FOOT_RIGHT    -> ANKLE_RIGHT
};




bool checkindex(_NUI_SKELETON_POSITION_INDEX index)
{
    switch (index)
    {
    case NUI_SKELETON_POSITION_HIP_CENTER:
        return false;
    case NUI_SKELETON_POSITION_SPINE:
        return false;
    case NUI_SKELETON_POSITION_SHOULDER_CENTER:
        return false;
    case NUI_SKELETON_POSITION_HEAD:
        return true;
    case NUI_SKELETON_POSITION_SHOULDER_LEFT:
        return true;
    case NUI_SKELETON_POSITION_ELBOW_LEFT:
        return true;
    case NUI_SKELETON_POSITION_WRIST_LEFT:
        return true;
    case NUI_SKELETON_POSITION_HAND_LEFT:
        return false;
    case NUI_SKELETON_POSITION_SHOULDER_RIGHT:
        return true;
    case NUI_SKELETON_POSITION_ELBOW_RIGHT:
        return true;
    case NUI_SKELETON_POSITION_WRIST_RIGHT:
        return true;
    case NUI_SKELETON_POSITION_HAND_RIGHT:
        return true;
    case NUI_SKELETON_POSITION_HIP_LEFT:
        return true;
    case NUI_SKELETON_POSITION_KNEE_LEFT:
        return true;
    case NUI_SKELETON_POSITION_ANKLE_LEFT:
        return true;
    case NUI_SKELETON_POSITION_FOOT_LEFT:
        return true;
    case NUI_SKELETON_POSITION_HIP_RIGHT:
        return true;
    case NUI_SKELETON_POSITION_KNEE_RIGHT:
        return true;
    case NUI_SKELETON_POSITION_ANKLE_RIGHT:
        return true;
    case NUI_SKELETON_POSITION_FOOT_RIGHT:
        return true;
    case NUI_SKELETON_POSITION_COUNT:
        return false;
    default:
        return false;
    }
}



HRESULT KinectDevice::initialize()
{
    INuiSensor* pNuiSensor;
    int iSensorCount = 0;
    HRESULT hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr)) return hr;

    for (int i = 0; i < iSensorCount; ++i)
    {
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr)) continue;

        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            NuiSensor = pNuiSensor;
            break;
        }
        pNuiSensor->Release();
    }

    if (NULL != NuiSensor)
    {
        // 1. ADDED: NUI_INITIALIZE_FLAG_USES_COLOR to the flags
        hr = NuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR);

        if (SUCCEEDED(hr))
        {
            // --- SKELETON SETUP ---
            NextSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
            hr = NuiSensor->NuiSkeletonTrackingEnable(NextSkeletonEvent, 0);

            // --- COLOR IMAGE SETUP ---
            // 2. ADDED: Create an event for color data
            NextColorEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

            // 3. ADDED: Open the color stream
            // hColorStream is a HANDLE you should define in your KinectDevice class header
            hr = NuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_COLOR,            // Stream type
                NUI_IMAGE_RESOLUTION_640x480,    // Resolution
                0,                               // Image stream flags
                2,                               // Number of frames to buffer
                NextColorEvent,                  // Event handle
                &hColorStream                    // Pointer to stream handle
            );
        }
    }

    if (NULL == NuiSensor || FAILED(hr))
    {
        return E_FAIL;
    }

    initoffsets();

    return hr;
}

void KinectDevice::shutdownKinect()
{
    if (NuiSensor)
    {
        NuiSensor->NuiShutdown();
    }

    if (NextSkeletonEvent && (NextSkeletonEvent != INVALID_HANDLE_VALUE))
    {
        CloseHandle(NextSkeletonEvent);
    }
}

void KinectDevice::processSkeleton()
{
    NUI_SKELETON_FRAME skeletonFrame = { 0 };
    joints.clear();
    HRESULT hr = NuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
    if (FAILED(hr))
    {
        return;
    }
    const NUI_TRANSFORM_SMOOTH_PARAMETERS defaultParams = { 0.5f, 0.5f, 0.5f, 0.05f, 0.04f };

    NuiSensor->NuiTransformSmooth(&skeletonFrame, &defaultParams);

    // smooth out the skeleton data
    NuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);

    RECT rct;
    GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
    int width = rct.right;
    int height = rct.bottom;


    for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
    {
        auto trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            // We're tracking the skeleton, draw it
            creatSkeleton(skeletonFrame.SkeletonData[i]);
        }

    }
}

MMatrix KinectDevice::convertToMayaMatrix(const Matrix4& kMat)
{
    MMatrix mayaMat;

    mayaMat[0][0] = kMat.M11;
    mayaMat[0][1] = kMat.M12;
    mayaMat[0][2] = kMat.M13;
    mayaMat[0][3] = kMat.M14;

    mayaMat[1][0] = kMat.M21;
    mayaMat[1][1] = kMat.M22;
    mayaMat[1][2] = -kMat.M23;
    mayaMat[1][3] = kMat.M24;

    mayaMat[2][0] = -kMat.M31;
    mayaMat[2][1] = -kMat.M32;
    mayaMat[2][2] = kMat.M33;
    mayaMat[2][3] = kMat.M34;

    mayaMat[3][0] = kMat.M41;
    mayaMat[3][1] = kMat.M42;
    mayaMat[3][2] = kMat.M43;
    mayaMat[3][3] = kMat.M44;

    return mayaMat;
}

// Build a direction vector from two joint positions
MVector KinectDevice::buildAxis(const MVector& a, const MVector& b)
{
    return MVector(b.x - a.x, b.y - a.y, b.z - a.z);
}

// Build rotation matrix from X and Y axes (Z computed via cross)
// Mirrors mat3_from_axis logic from your reference code
MMatrix KinectDevice::matFromAxes(const MVector& vx, const MVector& vy, const MVector& vz)
{
    MVector r0, r1, r2;

    // YZ plane (vx is zero) 
    if (vx.x == 0 && vx.y == 0 && vx.z == 0)
    {
        r1 = vy.normal();
        r0 = (vy ^ vz).normal();        // cross(vy, vz)
        r2 = (r0 ^ r1).normal();        // cross(r0, r1)
    }
    // XZ plane (vy is zero)
    else if (vy.x == 0 && vy.y == 0 && vy.z == 0)
    {
        r0 = vx.normal();
        r1 = (vz ^ vx).normal();        // cross(vz, vx)
        r2 = (r0 ^ r1).normal();        // cross(r0, r1)
    }
    // XY plane (vz is zero)  original code's most used path 
    else if (vz.x == 0 && vz.y == 0 && vz.z == 0)
    {
        r1 = vy.normal();
        r2 = (vx.normal() ^ r1).normal(); // cross(normalize(vx), r1)
        r0 = (r1 ^ r2).normal();          // cross(r1, r2)
    }
    else
    {
        // All axes provided  just normalize
        r0 = vx.normal();
        r1 = vy.normal();
        r2 = vz.normal();
    }

    double m[4][4] =
    {
        { r0.x, r0.y, r0.z, 0.0 },
        { r1.x, r1.y, r1.z, 0.0 },
        { r2.x, r2.y, r2.z, 0.0 },
        { 0.0,  0.0,  0.0,  1.0 }
    };
    return MMatrix(m);
}

// Build an inverse bind pose matrix from euler angles (degrees)
// Mirrors mat3_inverse(mat3_rotation_*(angle)) from reference
MMatrix KinectDevice::bindPoseInverse(float rx, float ry, float rz)
{
    MTransformationMatrix tm;
    double rot[3] = {
        rx * 3.1415 / 180.0,
        ry * 3.1415 / 180.0,
        rz * 3.1415 / 180.0
    };
    tm.setRotation(rot, MTransformationMatrix::kXYZ);
    return tm.asMatrix().inverse();
}

// Extract quaternion from matrix
MQuaternion KinectDevice::matToQuat(const MMatrix& m)
{
    MTransformationMatrix tm(m);
    MQuaternion q = tm.rotation();
    q.normalizeIt();
    return q;
}



void KinectDevice::initoffsets()
{
    offsets.resize(NUI_SKELETON_POSITION_COUNT);

    offsets[NUI_SKELETON_POSITION_HIP_CENTER] = MVector(0, 0, 0);
    offsets[NUI_SKELETON_POSITION_SPINE] = MVector(0, 6.81f, 0);
    offsets[NUI_SKELETON_POSITION_SHOULDER_CENTER] = MVector(0, 36.82f, 0);
    offsets[NUI_SKELETON_POSITION_HEAD] = MVector(0, 18.49f, 0);

    offsets[NUI_SKELETON_POSITION_SHOULDER_LEFT] = MVector(-14.f, 0, 0);
    offsets[NUI_SKELETON_POSITION_ELBOW_LEFT] = MVector(-25.f, 0, 0);
    offsets[NUI_SKELETON_POSITION_WRIST_LEFT] = MVector(-23.f, 0, 0);
    offsets[NUI_SKELETON_POSITION_HAND_LEFT] = MVector(-8.32f, 0, 0);

    offsets[NUI_SKELETON_POSITION_SHOULDER_RIGHT] = MVector(14.f, 0, 0);
    offsets[NUI_SKELETON_POSITION_ELBOW_RIGHT] = MVector(25.f, 0, 0);
    offsets[NUI_SKELETON_POSITION_WRIST_RIGHT] = MVector(23.f, 0, 0);
    offsets[NUI_SKELETON_POSITION_HAND_RIGHT] = MVector(8.32f, 0, 0);

    offsets[NUI_SKELETON_POSITION_HIP_LEFT] = MVector(-9.52f, 0, 0);
    offsets[NUI_SKELETON_POSITION_KNEE_LEFT] = MVector(0, -37.32f, 0);
    offsets[NUI_SKELETON_POSITION_ANKLE_LEFT] = MVector(0, -34.6f, 0);
    offsets[NUI_SKELETON_POSITION_FOOT_LEFT] = MVector(0, 0, 8.91f);

    offsets[NUI_SKELETON_POSITION_HIP_RIGHT] = MVector(9.52f, 0, 0);
    offsets[NUI_SKELETON_POSITION_KNEE_RIGHT] = MVector(0, -37.32f, 0);
    offsets[NUI_SKELETON_POSITION_ANKLE_RIGHT] = MVector(0, -34.6f, 0);
    offsets[NUI_SKELETON_POSITION_FOOT_RIGHT] = MVector(0, 0, 8.91f);
}

MEulerRotation KinectDevice::getEulers(int idx)
{
    // Get parent quaternion (identity if root joint)
    MQuaternion q_parent = MQuaternion::identity;
    if (idx != NUI_SKELETON_POSITION_HIP_CENTER)
    {
        const Vector4& pq = joints[parent_joint_map[idx]].quat;
        q_parent = MQuaternion(pq.x, pq.y, pq.z, pq.w);
    }

    // Get current joint quaternion
    const Vector4& cq = joints[idx].quat;
    MQuaternion q_current(cq.x, cq.y, cq.z, cq.w);

    // Delta = q_current * inverse(q_parent)
    MQuaternion q_delta = q_current * q_parent.inverse();

    // Convert to XYZ Euler
    return q_delta.asEulerRotation().reorder(MEulerRotation::kXYZ);
}

void KinectDevice::creatSkeleton(NUI_SKELETON_DATA& skel)
{
    NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT];
    HRESULT hr = NuiSkeletonCalculateBoneOrientations(&skel, boneOrientations);
    if (FAILED(hr))
    {
        return;
    }
    for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
    {
        const NUI_SKELETON_BONE_ORIENTATION& boneOrientation = boneOrientations[i];
        auto q = boneOrientation.hierarchicalRotation.rotationQuaternion;
        auto matrix = boneOrientation.hierarchicalRotation.rotationMatrix;
        // 1. Defensive programming: prevent divide-by-zero crashes
        float w = skel.SkeletonPositions[i].w;
        if (w == 0.0f)
        {
            w = 1.0f; // Fallback to avoid NaN
        }
        JointData jd;
        jd.position = MVector(
            -skel.SkeletonPositions[i].x ,
            skel.SkeletonPositions[i].y ,
            -skel.SkeletonPositions[i].z 
        );
        jd.orientation = MQuaternion(q.x, q.y, -q.z, -q.w);
        jd.rotation = convertToMayaMatrix(matrix);
		jd.positionKinect = skel.SkeletonPositions[i];


        joints.push_back(jd);
    }
    calculateQuaternion();
}

std::vector<JointData>& KinectDevice::getLatestJoints()
{
    return joints;
}

bool KinectDevice::getFrame(unsigned char* outBuffer)
{
    if (!NuiSensor || hColorStream == NULL) return false;


    // Process Color Frame
    if (WaitForSingleObject(NextColorEvent, 0) == WAIT_OBJECT_0)
    {
        NUI_IMAGE_FRAME imageFrame;
        if (SUCCEEDED(NuiSensor->NuiImageStreamGetNextFrame(hColorStream, 0, &imageFrame)))
        {
            INuiFrameTexture* texture = imageFrame.pFrameTexture;
            NUI_LOCKED_RECT lockedRect;
            texture->LockRect(0, &lockedRect, NULL, 0);

            if (lockedRect.Pitch != 0) 
            {
                memcpy(outBuffer, lockedRect.pBits, lockedRect.Pitch * 480);
            }
            texture->UnlockRect(0);
            NuiSensor->NuiImageStreamReleaseFrame(hColorStream, &imageFrame);

            // Draw using cached skeleton (always available)
            POINT screenPoints[NUI_SKELETON_POSITION_COUNT];

            for (int j = 0; j < joints.size(); ++j)
            {
                float fx, fy;
				Vector4 pos = Vector4(joints[j].position.x, joints[j].position.y, joints[j].position.z, 1.0f);
                NuiTransformSkeletonToDepthImage(joints[j].positionKinect, &fx, &fy, NUI_IMAGE_RESOLUTION_640x480);
                screenPoints[j].x = static_cast<long>(fx + 0.5f);
                screenPoints[j].y = static_cast<long>(fy + 0.5f);
                drawPixel(outBuffer, screenPoints[j].x, screenPoints[j].y, 255, 0, 0, 5);
            }

            auto drawBone = [&](int joint1, int joint2) {
                drawLine(outBuffer,
                    screenPoints[joint1].x, screenPoints[joint1].y,
                    screenPoints[joint2].x, screenPoints[joint2].y, 3);
            };

            // Torso
            drawBone(NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
            drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
            drawBone(NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);
            // Left Arm
            drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
            drawBone(NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
            drawBone(NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
            drawBone(NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);
            // Right Arm
            drawBone(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
            drawBone(NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
            drawBone(NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
            drawBone(NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);
            // Left Leg
            drawBone(NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT);
            drawBone(NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT);
            drawBone(NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
            drawBone(NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);
            // Right Leg
            drawBone(NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT);
            drawBone(NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT);
            drawBone(NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
            drawBone(NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);

            return true;
        }
    }
    return false;
}

// Draw a filled circle for thick joints
void KinectDevice::drawPixel(unsigned char* buffer, int x, int y, unsigned char r, unsigned char g, unsigned char b, int radius)
{
    for (int dy = -radius; dy <= radius; dy++) 
    {
        for (int dx = -radius; dx <= radius; dx++) 
        {
            if (dx * dx + dy * dy <= radius * radius) // circle shape
            { 
                int px = x + dx;
                int py = y + dy;
                if (px < 0 || px >= 640 || py < 0 || py >= 480)  // if outside of the sceen
                { 
                    continue; 
                }

                int index = (py * 640 + px) * 4;
                buffer[index] = b;
                buffer[index + 1] = g;
                buffer[index + 2] = r;
                buffer[index + 3] = 255;
            }
        }
    }
}

// Thick line — draws multiple parallel lines offset by thickness
void KinectDevice::drawLine(unsigned char* buffer, int x1, int y1, int x2, int y2, int thickness)
{
    for (int t = -thickness / 2; t <= thickness / 2; ++t) 
    {
        // Offset perpendicular to line direction
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);

        // Offset in Y for mostly-horizontal lines, X for mostly-vertical
        int offsetX = (dy > dx) ? t : 0;
        int offsetY = (dy > dx) ? 0 : t;

        // Bresenham on the offset line
        int cx1 = x1 + offsetX, cy1 = y1 + offsetY;
        int cx2 = x2 + offsetX, cy2 = y2 + offsetY;

        int ddx = abs(cx2 - cx1);
        int sx = cx1 < cx2 ? 1 : -1;
        int ddy = -abs(cy2 - cy1);
        int sy = cy1 < cy2 ? 1 : -1;
        int error = ddx + ddy; 
        int error2;

        while (true) 
        {
            drawPixel(buffer, cx1, cy1, 0, 255, 0, 0); // radius=0 for line pixels
            if (cx1 == cx2 && cy1 == cy2) 
            { 
                break; 
            }

            error2 = 2 * error;
            if (error2 >= ddy) 
            { 
                error += ddy;
                cx1 += sx; 
            }

            if (error2 <= ddx)
            { 
                error += ddx;
                cy1 += sy;
            }
        }
    }
}


void KinectDevice::calculateQuaternion()
{

    // Store last stable cross result per call-site (one shared fallback
    // is fine because stableCross is always called sequentially).
    const float MAX_STABLE_DOT = 0.9f;
    MVector lastStable(0.0, 0.0, 1.0);

    auto stableCross = [&](const MVector& a, const MVector& b) -> MVector
        {
            MVector na = a.normal();
            MVector nb = b.normal();
            if (std::fabs(na * nb) > MAX_STABLE_DOT)   // MVector::operator* = dot
            {
                return lastStable;
            }
            lastStable = na ^ nb;                        // MVector::operator^ = cross
            return lastStable;
        };

    // Bone direction vector from two Kinect joint positions
    auto boneVec = [&](NUI_SKELETON_POSITION_INDEX from,
        NUI_SKELETON_POSITION_INDEX to) -> MVector
        {
            const Vector4& p1 = joints[from].positionKinect;
            const Vector4& p2 = joints[to].positionKinect;
            return MVector(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        };

	// Build a rotation matrix from two axes: vx (primary) and vy (secondary)
    auto mat3FromAxis = [&](const MVector& vx, const MVector& vy) -> MMatrix
        {
            MVector Y = vy.normal();
            MVector Z = (vx.normal() ^ Y).normal();   // cross(normalize(vx), Y)
            MVector X = (Y ^ Z).normal();              // cross(Y, Z)  ->  right-hand rebuild

            double m[4][4] = {
                { X.x, X.y, X.z, 0.0 },
                { Y.x, Y.y, Y.z, 0.0 },
                { Z.x, Z.y, Z.z, 0.0 },
                { 0.0, 0.0, 0.0, 1.0 }
            };
            return MMatrix(m);
        };

 
    // Rotation matrices around each axis
    auto rotX = [](double rad) -> MMatrix
        {
            double c = std::cos(rad), s = std::sin(rad);
            double m[4][4] = {
                { 1,  0,  0,  0 },
                { 0,  c,  s,  0 },
                { 0, -s,  c,  0 },
                { 0,  0,  0,  1 }
            };
            return MMatrix(m);
        };

    auto rotZ = [](double rad) -> MMatrix
        {
            double c = std::cos(rad), s = std::sin(rad);
            double m[4][4] = {
                {  c,  s,  0,  0 },
                { -s,  c,  0,  0 },
                {  0,  0,  1,  0 },
                {  0,  0,  0,  1 }
            };
            return MMatrix(m);
        };

    auto storeQuat = [&](NUI_SKELETON_POSITION_INDEX joint, const MQuaternion& q)
        {
            joints[joint].quat.x = static_cast<float>(q.x);
            joints[joint].quat.y = static_cast<float>(q.y);
            joints[joint].quat.z = static_cast<float>(q.z);
            joints[joint].quat.w = static_cast<float>(q.w);
        };

    auto applyJointRot = [&](NUI_SKELETON_POSITION_INDEX joint,
        const MVector& vx, const MVector& vy,
        const MMatrix& bindInverse)
        {
            MMatrix boneMat = mat3FromAxis(vx, vy);
            MMatrix finalMat = bindInverse * boneMat;   // row-major: bindInv first
            storeQuat(joint, matToQuat(finalMat));
        };


    const MMatrix inverseBindPoseNone = MMatrix::identity;
    const MMatrix inverseBindPoseArmL = rotZ(-PI / 2.0);   // inverse of rotZ(+π/2)
    const MMatrix inverseBindPoseArmR = rotZ(PI / 2.0);   // inverse of rotZ(-π/2)
    const MMatrix inverseBindPoseLeg = rotZ(-PI);          // inverse of rotZ(π)  = rotZ(-π)
    const MMatrix inverseBindPoseAnkle = rotX(-PI / 2.0);   // inverse of rotX(+π/2)


    // Knee-bend disambiguation: vx must point backward relative to body
    auto kneeVx = [&](NUI_SKELETON_POSITION_INDEX upper,
        NUI_SKELETON_POSITION_INDEX mid,
        NUI_SKELETON_POSITION_INDEX lower,
        const MVector& bodyX) -> MVector
        {
            MVector v1 = boneVec(upper, mid);
            MVector v2 = boneVec(mid, lower);
            MVector vx = -(stableCross(v1, v2));        // negate: knees bend backward
            if ((bodyX.normal() * vx.normal()) > 0.0)   //still forward
            {
                vx = -vx;
            }
            return vx;
        };

    // TORSO CHAIN
    MVector hipAxis = boneVec(NUI_SKELETON_POSITION_HIP_LEFT,
        NUI_SKELETON_POSITION_HIP_RIGHT);

    applyJointRot(NUI_SKELETON_POSITION_HIP_CENTER,
        hipAxis,
        boneVec(NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_SPINE),
        inverseBindPoseNone);

    MVector v_body_x = hipAxis;   // saved for knee disambiguation

    applyJointRot(NUI_SKELETON_POSITION_SPINE,
        hipAxis,
        boneVec(NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_SHOULDER_CENTER),
        inverseBindPoseNone);

    applyJointRot(NUI_SKELETON_POSITION_SHOULDER_CENTER,
        hipAxis,
        boneVec(NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_HEAD),
        inverseBindPoseNone);

    // HEAD copies shoulder-center
    joints[NUI_SKELETON_POSITION_HEAD].quat =
        joints[NUI_SKELETON_POSITION_SHOULDER_CENTER].quat;


    // LEFT ARM
    {
        MVector v1, v2;

        v1 = boneVec(NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
        v2 = boneVec(NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
        applyJointRot(NUI_SKELETON_POSITION_SHOULDER_LEFT, stableCross(v1, v2), v1, inverseBindPoseArmL);

        v1 = boneVec(NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
        v2 = boneVec(NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);
        applyJointRot(NUI_SKELETON_POSITION_ELBOW_LEFT, stableCross(v1, v2), v1, inverseBindPoseArmL);

        v1 = boneVec(NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
        v2 = boneVec(NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);
        applyJointRot(NUI_SKELETON_POSITION_WRIST_LEFT, stableCross(v1, v2), v2, inverseBindPoseArmL); // vy=v2

        joints[NUI_SKELETON_POSITION_HAND_LEFT].quat =
            joints[NUI_SKELETON_POSITION_WRIST_LEFT].quat;
    }

    // RIGHT ARM  (cross order flipped for mirror: stableCross(v2,v1))
    {
        MVector v1, v2;

        v1 = boneVec(NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
        v2 = boneVec(NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
        applyJointRot(NUI_SKELETON_POSITION_SHOULDER_RIGHT, stableCross(v2, v1), v1, inverseBindPoseArmR);

        v1 = boneVec(NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
        v2 = boneVec(NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);
        applyJointRot(NUI_SKELETON_POSITION_ELBOW_RIGHT, stableCross(v2, v1), v1, inverseBindPoseArmR);

        v1 = boneVec(NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
        v2 = boneVec(NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);
        applyJointRot(NUI_SKELETON_POSITION_WRIST_RIGHT, stableCross(v2, v1), v2, inverseBindPoseArmR); // vy=v2

        joints[NUI_SKELETON_POSITION_HAND_RIGHT].quat =
            joints[NUI_SKELETON_POSITION_WRIST_RIGHT].quat;
    }

    // LEFT LEG
    {
        MVector v1, v2;

        v1 = boneVec(NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT);
        applyJointRot(NUI_SKELETON_POSITION_HIP_LEFT,
            kneeVx(NUI_SKELETON_POSITION_HIP_LEFT,
                NUI_SKELETON_POSITION_KNEE_LEFT,
                NUI_SKELETON_POSITION_ANKLE_LEFT, v_body_x),
            v1, inverseBindPoseLeg);

        v1 = boneVec(NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
        v2 = boneVec(NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);
        applyJointRot(NUI_SKELETON_POSITION_KNEE_LEFT, stableCross(v1, v2), v1, inverseBindPoseLeg);

        v1 = boneVec(NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
        v2 = boneVec(NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);
        applyJointRot(NUI_SKELETON_POSITION_ANKLE_LEFT, stableCross(v1, v2), v2, inverseBindPoseAnkle); // vy=v2

        joints[NUI_SKELETON_POSITION_FOOT_LEFT].quat =
            joints[NUI_SKELETON_POSITION_ANKLE_LEFT].quat;
    }

    // RIGHT LEG
    {
        MVector v1, v2;

        v1 = boneVec(NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT);
        applyJointRot(NUI_SKELETON_POSITION_HIP_RIGHT,
            kneeVx(NUI_SKELETON_POSITION_HIP_RIGHT,
                NUI_SKELETON_POSITION_KNEE_RIGHT,
                NUI_SKELETON_POSITION_ANKLE_RIGHT, v_body_x),
            v1, inverseBindPoseLeg);

        v1 = boneVec(NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
        v2 = boneVec(NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);
        applyJointRot(NUI_SKELETON_POSITION_KNEE_RIGHT, stableCross(v1, v2), v1, inverseBindPoseLeg);

        v1 = boneVec(NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
        v2 = boneVec(NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);
        applyJointRot(NUI_SKELETON_POSITION_ANKLE_RIGHT, stableCross(v1, v2), v2, inverseBindPoseAnkle); // vy=v2

        joints[NUI_SKELETON_POSITION_FOOT_RIGHT].quat =
            joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].quat;
    }
}


KinectDevice::~KinectDevice()
{
    // shutdownKinect();
}

