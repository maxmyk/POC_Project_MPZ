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
std::string my_data2;

char com_port1[] = "\\\\.\\COM7";
SimpleSerial Serial1(com_port1, COM_BAUD_RATE);
std::string my_data1;

char com_port3[] = "\\\\.\\COM3";
SimpleSerial Serial3(com_port3, COM_BAUD_RATE);

int location1 = 0, location2 = 0;
int scale1 = 0, scale2 = 0;

double coefficients[25] = {0.003, 0.013, 0.022, 0.013, 0.003, 0.013, 0.06, 0.098, 0.06, 0.013, 0.022, 0.098, 0.162, 0.098, 0.022, 0.013, 0.06, 0.098, 0.06, 0.013, 0.003, 0.013, 0.022, 0.013, 0.003};

IBodyFrameSource* pBodyFrameSource = nullptr;

void colorFrame(IKinectSensor* pSensor, std::vector<std::tuple<CameraSpacePoint, bool> > &targets) {
    HRESULT hr = S_OK;
    IColorFrameSource* pColorFrameSource = nullptr;
    hr = pSensor->get_ColorFrameSource(&pColorFrameSource);
    RGBQUAD* m_pColorRGBX;
    m_pColorRGBX = new RGBQUAD[1920 * 1080];
    if (SUCCEEDED(hr))
    {
        IColorFrameReader* pColorFrameReader = nullptr;
        hr = pColorFrameSource->OpenReader(&pColorFrameReader);
        if (SUCCEEDED(hr))
        {
            IColorFrame* pColorFrame = NULL;
            hr = pColorFrameReader->AcquireLatestFrame(&pColorFrame);
            if (SUCCEEDED(hr))
            {
                INT64 nTime = 0;
                IFrameDescription* pFrameDescription = NULL;
                int nWidth = 0;
                int nHeight = 0;
                ColorImageFormat imageFormat = ColorImageFormat_None;
                UINT nBufferSize = 0;
                RGBQUAD* pBuffer = NULL;

                hr = pColorFrame->get_RelativeTime(&nTime);

                if (SUCCEEDED(hr))
                {
                    hr = pColorFrame->get_FrameDescription(&pFrameDescription);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pFrameDescription->get_Width(&nWidth);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pFrameDescription->get_Height(&nHeight);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
                }
                if (SUCCEEDED(hr))
                {
                    if (imageFormat == ColorImageFormat_Bgra)
                    {
                        hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
                    }
                    else if (m_pColorRGBX)
                    {
                        pBuffer = m_pColorRGBX;
                        nBufferSize = 1920 * 1080 * sizeof(RGBQUAD);
                        hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Rgba);
                    }
                    else
                    {
                        hr = E_FAIL;
                    }
                }
                
                for (auto& target : targets) {
                    ICoordinateMapper* pCoordinateMapper = nullptr;
                    pSensor->get_CoordinateMapper(&pCoordinateMapper);

                    ColorSpacePoint colorSpacePoint;
                    pCoordinateMapper->MapCameraPointToColorSpace(get<0>(target), &colorSpacePoint);

                    int x = (int)(colorSpacePoint.X + 0.5f);
                    int y = (int)(colorSpacePoint.Y + 0.5f);

                    if (x >= 0 && x < nWidth && y >= 0 && y < nHeight) {
                        
                        int red = 0;
                        int green = 0;
                        int blue = 0;
                        size_t counter = 0;
                        for (size_t i = y - 2; i <= y + 2; ++i) {
                            for (size_t j = x - 2; j <= x + 2; ++j) {
                                int index = (y * nWidth + x);
                                red += int(pBuffer[index].rgbBlue) * coefficients[counter];
                                green += int(pBuffer[index].rgbGreen) * coefficients[counter];
                                blue += int(pBuffer[index].rgbRed) * coefficients[counter];
                                counter++;
                            }
                        }
                        
                        string data = "";
                        data += to_string(red) + ' ' + to_string(green) + ' ' + to_string(blue) + '\n';
                        OutputDebugStringA(data.c_str());
                        if (red > 110 && blue < 70 && green < 70) {
                            get<1>(target) = true;
                        }
                    }
                }
                SafeRelease(pColorFrame);
            }
        }
    }
    delete[] m_pColorRGBX;
}

void toStream(std::vector<std::tuple<CameraSpacePoint, bool> > &targets) {

    if (targets.size() != 0) {

        // 1 port

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
        my_data1 += to_string((int)(120 - 120 * x_axis1 * scale1 / PI + location1));
        my_data1 += "Y";
        my_data1 += to_string((int)(120 * y_axis1 * scale1 / PI + location1));
        //OutputDebugStringA(debug_str1.c_str());
        char* new_data1 = new char[my_data1.size()];
        for (int i = 0; i < my_data1.size(); ++i) {
            new_data1[i] = my_data1[i];
        }

        // 2 port

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
        my_data2 += to_string((int)(120 - 120 * x_axis2 * scale2 / PI + location2));
        my_data2 += "Y";
        my_data2 += to_string((int)(120 * y_axis2 * scale2 / PI + location2));
        //OutputDebugStringA(debug_str2.c_str());
        char* new_data2 = new char[my_data2.size()];
        for (int i = 0; i < my_data2.size(); ++i) {
            new_data2[i] = my_data2[i];
        }

        // data send

        if (Serial1.connected_) {
            OutputDebugStringA("Connected 7 PORT!!!");
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
            Serial1.WriteSerialPort("D");
            string parametrs1 = Serial1.ReadSerialPort(reply_wait_time, syntax_type);
            int idx = 0;
            while (parametrs1[idx] != ',') {
                location1 *= 10;
                location1 += parametrs1[idx] - '0';
                idx++;
            }
            idx++;
            while (idx != parametrs1.size()) {
                scale1 *= 10;
                scale1 += parametrs1[idx] - '0';
                idx++;
            }

            Serial2.WriteSerialPort("D");
            string parametrs2 = Serial2.ReadSerialPort(reply_wait_time, syntax_type);
            int idx = 0;
            while (parametrs2[idx] != ',') {
                location2 *= 10;
                location2 += parametrs2[idx] - '0';
                idx++;
            }
            idx++;
            while (idx != parametrs2.size()) {
                scale2 *= 10;
                scale2 += parametrs2[idx] - '0';
                idx++;
            }

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
