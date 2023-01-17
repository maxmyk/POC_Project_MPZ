#include <Windows.h>
#include <Kinect.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <tuple>
#include "stdafx.h"
#include "SimpleSerial.h"

#define PI 3.14159265359

char com_port2[] = "\\\\.\\COM6";
DWORD COM_BAUD_RATE = CBR_9600;
SimpleSerial Serial2(com_port2, COM_BAUD_RATE);
bool con_flag2 = false;
std::string my_data2;

char com_port1[] = "\\\\.\\COM4";
SimpleSerial Serial1(com_port1, COM_BAUD_RATE);
bool con_flag1 = false;
std::string my_data1;
    
IBodyFrameSource* pBodyFrameSource = nullptr;

void colorFrame(IKinectSensor* pSensor, std::vector<std::tuple<CameraSpacePoint, bool> >&targets) {
    HRESULT hr = S_OK;
    IColorFrameSource* pColorFrameSource = nullptr;
    hr = pSensor->get_ColorFrameSource(&pColorFrameSource);
    if (SUCCEEDED(hr))
    {
        IColorFrameReader* pColorFrameReader = nullptr;
        hr = pColorFrameSource->OpenReader(&pColorFrameReader);
        if (SUCCEEDED(hr))
        {
            IColorFrame* pColorFrame = nullptr;
            hr = pColorFrameReader->AcquireLatestFrame(&pColorFrame);
            if (SUCCEEDED(hr))
            {
                UINT colorBufferSize = 0;
                BYTE* pColorBuffer = nullptr;
                hr = pColorFrame->AccessRawUnderlyingBuffer(&colorBufferSize, &pColorBuffer);
                int a = 1;

                if (SUCCEEDED(hr))
                {
                    int colorWidth = 1440;
                    int colorHeight = 720;

                    ICoordinateMapper* pCoordinateMapper = nullptr;
                    pSensor->get_CoordinateMapper(&pCoordinateMapper);

                    for (auto & target: targets) {
                        ColorSpacePoint colorSpacePoint;
                        pCoordinateMapper->MapCameraPointToColorSpace(get<0>(target), &colorSpacePoint);

                        int x = (int)(colorSpacePoint.X + 0.5f);
                        int y = (int)(colorSpacePoint.Y + 0.5f);

                        if (x >= 0 && x < colorWidth && y >= 0 && y < colorHeight) {
                            int index = (y * colorWidth + x) * 4;
                            unsigned char red = pColorBuffer[index + 2];
                            unsigned char green = pColorBuffer[index + 1];
                            unsigned char blue = pColorBuffer[index];
                            if (red > 200 && green > 200 &&  blue > 200) {
                                get<1>(target) = true;
                            }
                        }
                    }
                }
                SafeRelease(pColorFrame);
            }
        }
    }
 
}

void toStream(std::vector<std::tuple<CameraSpacePoint, bool> > &targets) {
    //std::sort(targets.begin(), targets.end());
    if (Serial1.connected_) {
        con_flag1 = true;
    }

    if (Serial2.connected_) {
        con_flag2 = true;
    }

    if (targets.size() != 0) {

        // 3 port

        float x_data1 = get<0>(targets[0]).X;
        float y_data1 = get<0>(targets[0]).Y;
        float z_data1 = get<0>(targets[0]).Z;
        bool state1 = get<1>(targets[0]);

        std::string debug_str1;
        float x_axis1 = 0.0;
        float y_axis1 = 0.0;
        float z_axis1 = 0.0;
        if ((x_data1 * x_data1 + y_data1 * y_data1 + z_data1 * z_data1) != 0) {
            x_axis1 = acos(x_data1 / sqrtf(x_data1 * x_data1 + y_data1 * y_data1 + z_data1 * z_data1));
            y_axis1 = acos(y_data1 / sqrtf(x_data1 * x_data1 + y_data1 * y_data1 + z_data1 * z_data1));
        }
        debug_str1 += to_string(120 * x_axis1 / PI);
        debug_str1 += " ";
        debug_str1 += to_string((120 - 120 * y_axis1) / PI);
        debug_str1 += " ";
        debug_str1 += to_string(70 + 180 * z_axis1 / PI);
        debug_str1 += "\n";

        if (state1) { my_data1 = "L1X"; }
        else { my_data1 = "X"; }
        my_data1 += to_string((int)(120 - 120 * x_axis1 / PI - 1));
        my_data1 += "Y";
        my_data1 += to_string((int)(120 * y_axis1 / PI - 6));
        OutputDebugStringA(debug_str1.c_str());
        char* new_data1 = new char[my_data1.size()];
        for (int i = 0; i < my_data1.size(); ++i) {
            new_data1[i] = my_data1[i];
        }

        // 4 port

        float x_data2 = get<0>(targets[targets.size() - 1]).X;
        float y_data2 = get<0>(targets[targets.size() - 1]).Y;
        float z_data2 = get<0>(targets[targets.size() - 1]).Z;
        bool state2 = get<1>(targets[targets.size() - 1]);

        std::string debug_str2;
        float x_axis2 = 0.0;
        float y_axis2 = 0.0;
        float z_axis2 = 0.0;
        if ((x_data2 * x_data2 + y_data2 * y_data2 + z_data2 * z_data2) != 0) {
            x_axis2 = acos(x_data2 / sqrtf(x_data2 * x_data2 + y_data2 * y_data2 + z_data2 * z_data2));
            y_axis2 = acos(y_data2 / sqrtf(x_data2 * x_data2 + y_data2 * y_data2 + z_data2 * z_data2));
        }
        debug_str2 += to_string(120 * x_axis2 / PI);
        debug_str2 += " ";
        debug_str2 += to_string((120 - 120 * y_axis2) / PI);
        debug_str2 += " ";
        debug_str2 += to_string(70 + 180 * z_axis2 / PI);
        debug_str2 += "\n";


        if (state2) { my_data2 = "L1X"; }
        else { my_data2 = "X"; }
        my_data2 += to_string((int)(120 - 120 * x_axis2 / PI - 1));
        my_data2 += "Y";
        my_data2 += to_string((int)(120 * y_axis2 / PI - 6));
        OutputDebugStringA(debug_str2.c_str());
        char* new_data2 = new char[my_data2.size()];
        for (int i = 0; i < my_data2.size(); ++i) {
            new_data2[i] = my_data2[i];
        }

        // data send

        if (con_flag1) {
            OutputDebugStringA("Connected 3 PORT!!!");
            Serial1.WriteSerialPort(new_data1);
        }

        if (con_flag2) {
            OutputDebugStringA("Connected 6 PORT!!!");
            Serial2.WriteSerialPort(new_data2);
            
        }
        Sleep(15);
    }
}



int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    IKinectSensor* pSensor = nullptr;
    HRESULT hr = S_OK;
    hr = GetDefaultKinectSensor(&pSensor);
    if (FAILED(hr))
    {
        OutputDebugStringA("Connection ERROR");
        return -1;
    }

    IBodyFrameReader* pBodyFrameReader = nullptr;
    hr = pSensor->Open();
    if (SUCCEEDED(hr))
    {
        hr = pSensor->get_BodyFrameSource(&pBodyFrameSource);
    }
    if (SUCCEEDED(hr))
    {
        hr = pBodyFrameSource->OpenReader(&pBodyFrameReader);
    }

    IBody* pBody[BODY_COUNT] = { 0 };
    while (true)
    {
        IBodyFrame* pBodyFrame = nullptr;
        hr = pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
            if (SUCCEEDED(hr))
            {
                std::vector<std::tuple<CameraSpacePoint, bool> > targets;
                for (int i = 0; i < BODY_COUNT; ++i)
                {
                    IBody* Body = pBody[i];
                    if (Body)
                    {
                        BOOLEAN bTracked = false;
                        hr = Body->get_IsTracked(&bTracked);

                        if (SUCCEEDED(hr) && bTracked)
                        {
                            Joint joints[JointType_Count];
                            D2D1_POINT_2F jointPoints[JointType_Count];
                            HandState leftHandState = HandState_Unknown;
                            HandState rightHandState = HandState_Unknown;

                            Body->get_HandLeftState(&leftHandState);
                            Body->get_HandRightState(&rightHandState);

                            hr = Body->GetJoints(_countof(joints), joints);
                            if (SUCCEEDED(hr))
                            {
                                targets.push_back(std::make_tuple(joints[JointType_SpineMid].Position, false));
                            }
                        }
                    }
                }
                if (targets.size() != 0) {
                    colorFrame(pSensor, targets);
                    toStream(targets);
                }
            }
        }


        SafeRelease(pBodyFrame);
    }

    for (int i = 0; i < BODY_COUNT; ++i)
    {
        SafeRelease(pBody[i]);
    }
    SafeRelease(pBodyFrameReader);
    SafeRelease(pSensor);

    return 0;
}
