#include <Windows.h>
#include <Kinect.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <tuple>
#include "stdafx.h"
#include "SimpleSerial.h"
//#include <opencv2/opencv.hpp>

#define PI 3.14159265359

char com_port2[] = "\\\\.\\COM6";
DWORD COM_BAUD_RATE = CBR_9600;
SimpleSerial Serial2(com_port2, COM_BAUD_RATE);
std::string my_data2;

char com_port1[] = "\\\\.\\COM7";
SimpleSerial Serial1(com_port1, COM_BAUD_RATE);
std::string my_data1;

char com_port3[] = "\\\\.\\COM3";
SimpleSerial Serial3(com_port3, COM_BAUD_RATE);

IBodyFrameSource* pBodyFrameSource = nullptr;

void colorFrame(IKinectSensor* pSensor, std::vector<std::tuple<float, float, float, bool> >& targets) {
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
                int colorWidth = 1920;
                int colorHeight = 1080;
                UINT colorBufferSize = colorWidth * colorHeight * 4;
                BYTE* pColorBuffer = nullptr;
                hr = pColorFrame->AccessRawUnderlyingBuffer(&colorBufferSize, &pColorBuffer);
                hr = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, pColorBuffer, ColorImageFormat_Rgba);
                int b = int(pColorBuffer[0]);
                int a = 1;
                
                SafeRelease(pColorFrame);
            }
        }
    }
 
}

void toStream(std::vector<std::tuple<float, float, float, bool> > &targets) {
    std::sort(targets.begin(), targets.end());

    if (targets.size() != 0) {

        // 3 port

        float x_data1 = get<0>(targets[0]);
        float y_data1 = get<1>(targets[0]);
        float z_data1 = get<2>(targets[0]);
        bool state1 = get<3>(targets[0]);

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

        float x_data2 = get<0>(targets[targets.size() - 1]);
        float y_data2 = get<1>(targets[targets.size() - 1]);
        float z_data2 = get<2>(targets[targets.size() - 1]);
        bool state2 = get<3>(targets[targets.size() - 1]);

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

        if (Serial1.connected_) {
            OutputDebugStringA("Connected 5 PORT!!!");
            Serial1.WriteSerialPort(new_data1);
        }

        if (Serial2.connected_) {
            OutputDebugStringA("Connected 6 PORT!!!");
            Serial2.WriteSerialPort(new_data2);
        }
        Sleep(15);
    }
}



int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    while (!Serial1.connected_ && !Serial2.connected_ && !Serial3.connected_) {
        ;
    }
    
    while (1) {
        int reply_wait_time = 1;
        string syntax_type = "greater_less_than";

        string incoming = Serial3.ReadSerialPort(reply_wait_time, syntax_type);
        if (incoming == "S")
            break;
    }

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
                std::vector<std::tuple<float, float, float, bool> > targets;
                //std::vector<std::tuple<float, float, float, bool> > targets;
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
                                float x = joints[JointType_SpineShoulder].Position.X;
                                float y = joints[JointType_SpineShoulder].Position.Y;
                                float z = joints[JointType_SpineShoulder].Position.Z;

                                if (leftHandState == 3 || rightHandState == 3) {
                                    targets.push_back(std::make_tuple(x, y, z, true));
                                }
                                else {
                                    targets.push_back(std::make_tuple(x, y, z, false));
                                }
                                
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
