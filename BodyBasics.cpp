#include <Windows.h>
#include <Kinect.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <tuple>
#include "stdafx.h"
#include "SimpleSerial.h"

#define PI 3.14159265359

char com_port2[] = "\\\\.\\COM14";
DWORD COM_BAUD_RATE = CBR_9600;
SimpleSerial Serial2(com_port2, COM_BAUD_RATE);
std::string my_data2;

char com_port1[] = "\\\\.\\COM5";
SimpleSerial Serial1(com_port1, COM_BAUD_RATE);
std::string my_data1;

char com_port3[] = "\\\\.\\COM13";
SimpleSerial Serial3(com_port3, COM_BAUD_RATE);

double location1_x = 0, location1_y = 0, location2_x = 0, location2_y = 0, location1_z = 0, location2_z = 0;

double coefficients[25] = { 0.003, 0.013, 0.022, 0.013, 0.003, 0.013, 0.06, 0.098, 0.06, 0.013, 0.022, 0.098, 0.162, 0.098, 0.022, 0.013, 0.06, 0.098, 0.06, 0.013, 0.003, 0.013, 0.022, 0.013, 0.003 };

IBodyFrameSource* pBodyFrameSource = nullptr;

void colorFrame(IKinectSensor* pSensor, std::vector<std::tuple<CameraSpacePoint, bool> >& targets) {
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
                        OutputDebugStringA("Colors");
                        OutputDebugStringA(data.c_str());
                        if (red > 100 && blue < 80 && green < 80) {
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

void toStream(std::vector<std::tuple<CameraSpacePoint, bool> >& targets) {

    if (targets.size() != 0) {

        /*
        First turret
        */

        float x_data1 = get<0>(targets[0]).X;
        float y_data1 = get<0>(targets[0]).Y;
        float z_data1 = get<0>(targets[0]).Z;
        bool state1 = get<1>(targets[0]);

        std::string debug_str1;
        float x_axis1 = 90.0;
        float y_axis1 = 90.0;

        if ((x_data1 * x_data1 + y_data1 * y_data1 + z_data1 * z_data1) != 0) {
            x_axis1 = (int)(180 + 180 * atan(abs(location1_z - z_data1) / (location1_x - x_data1)) / PI) % 180;
            y_axis1 = (int)(90 - 180 * atan((location1_y - y_data1) / sqrt(pow(location1_z - z_data1, 2) + pow(location1_x - x_data1, 2))) / PI)% 180;
        }

        debug_str1 += to_string(z_data1); // y
        debug_str1 += " ";
        debug_str1 += to_string(x_data1); // z
        debug_str1 += " ";
        debug_str1 += to_string(y_data1); // x
        debug_str1 += "\n :^)";
        debug_str1 += to_string(x_axis1);
        debug_str1 += " ";
        debug_str1 += to_string(y_axis1);
        debug_str1 += "\n";
        OutputDebugStringA(debug_str1.c_str());

        if (state1) { my_data1 = "L1X"; }
        else { my_data1 = "X"; }
        my_data1 += to_string((int)x_axis1);
        my_data1 += "Y";
        my_data1 += to_string((int)y_axis1);

        OutputDebugStringA(my_data1.c_str());

        char* new_data1 = new char[my_data1.length() + 1];
        strcpy(new_data1, my_data1.c_str());

        /*
        Second turret
        */

        float x_data2 = get<0>(targets[targets.size() - 1]).X;
        float y_data2 = get<0>(targets[targets.size() - 1]).Y;
        float z_data2 = get<0>(targets[targets.size() - 1]).Z;
        bool state2 = get<1>(targets[targets.size() - 1]);

        std::string debug_str2;
        float x_axis2 = 90.0;
        float y_axis2 = 90.0;

        if ((x_data2 * x_data2 + y_data2 * y_data2 + z_data2 * z_data2) != 0) {
            x_axis2 = (int)(180 + 180 * atan(abs(location2_z - z_data2) / (location2_x - x_data2)) / PI) % 180;
            y_axis2 = (int)(90 + 180 * atan((location2_y - y_data2) / sqrt(pow(location2_z - z_data2, 2) + pow(location2_x - x_data2, 2))) / PI) % 180;
        }

        debug_str2 += to_string(z_data2); // y
        debug_str2 += " ";
        debug_str2 += to_string(x_data2); // z
        debug_str2 += " ";
        debug_str2 += to_string(y_data2); // x
        debug_str2 += "\n :^)";
        debug_str2 += to_string(x_axis2);
        debug_str2 += " ";
        debug_str2 += to_string(y_axis2);
        debug_str2 += "\n";
        OutputDebugStringA(debug_str2.c_str());

        if (state2) { my_data2 = "L1X"; }
        else { my_data2 = "X"; }
        my_data2 += to_string((int)x_axis2);
        my_data2 += "Y";
        my_data2 += to_string((int)y_axis2);

        OutputDebugStringA(my_data2.c_str());

        char* new_data2 = new char[my_data2.length() + 1];
        strcpy(new_data2, my_data2.c_str());

        // data send

        if (Serial1.connected_) {
            OutputDebugStringA("\nTurret 1 Connected\n");
            Serial1.WriteSerialPort(new_data1);
            OutputDebugStringA(new_data1);
        }
        if (Serial2.connected_) {
            OutputDebugStringA("\nTurret 2 Connected\n");
            Serial2.WriteSerialPort(new_data2);
            OutputDebugStringA(new_data2);
            OutputDebugStringA(" \n ");
        }
        Sleep(15);
    }
}

vector<double> readParams(string& parameters) {
//    Reads parameters from string and returns vector of doubles
//    Parameters are given as ints in such format: <25,-35,45,0>
//    Parameters are: x, y, z, free
    vector<double> params;
    string param = "";
    for (int i = 0; i < parameters.length(); i++) {
        if (parameters[i] == ',') {
            params.push_back(stod(param)/100);
            param = "";
        }
        else if (parameters[i] == '>') {
            params.push_back(stod(param)/100);
            param = "";
            break;
        }
        else {
            param += parameters[i];
        }
    }
    return params;
}

int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    while (!Serial1.connected_ && !Serial2.connected_ && !Serial3.connected_) {
        ;
    }

    while (1) {
        int reply_wait_time = 1;
        string syntax_type = "greater_less_than";
        string parametrs1 = "";
        string parametrs2 = "";
        string incoming = "";
        while (1) {
            incoming = Serial3.ReadSerialPort(reply_wait_time, syntax_type);
            if (incoming != "") {
                OutputDebugStringA(incoming.c_str());
                break;
            }
            else {
                OutputDebugStringA(".");
            }
        }
        char* d = new char[1];
        d[0] = 'D';
        if (incoming == "SIREN") {
            while (1) {
                Serial1.WriteSerialPort(d);
                parametrs1 = Serial1.ReadSerialPort(reply_wait_time, syntax_type);
                if (parametrs1 != "") {
                    break;
                }
            }
            OutputDebugStringA("A");
            vector<double> v1 = readParams(parametrs1);
            location1_x = v1[0];
            location1_y = v1[1];
            location1_z = v1[2];
            Sleep(20);
            while (1) {
                Serial2.WriteSerialPort(d);
                parametrs2 = Serial2.ReadSerialPort(reply_wait_time, syntax_type);
                if (parametrs2 != "") {
                    break;
                }
            }
            OutputDebugStringA("B");
            vector<double> v2 = readParams(parametrs2);
            location2_x = v2[0];
            location2_y = v2[1];
            location2_z = v2[2];
            Sleep(20);
            OutputDebugStringA("C");
            break;
        }
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
                                targets.push_back(std::make_tuple(joints[JointType_SpineShoulder].Position, false));
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
