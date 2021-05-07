//Note:DCAM800 has no RGB frame and no use DCAM_305 in Vzense_dcamtype.h

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "Vzense_api2.h"
#include "VzenseMap_Api.h"
#include <thread>

using namespace std;
using namespace cv;
PsDeviceInfo* pDeviceListInfo = NULL;
PsDeviceHandle deviceHandle = 0;
uint32_t sessionIndex = 0;
uint32_t slope = 1450;
 
static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	Point2d pointxy(width / 2, height / 2);
	int val = dispImg.at<ushort>(pointxy);
	char text[20];
#ifdef _WIN32
	sprintf_s(text, "%d", val);
#else
	snprintf(text, sizeof(text), "%d", val);
#endif
	dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
	applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
	int color;
	if (val > 2500)
		color = 0;
	else
		color = 4096;
	circle(dispImg, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
	putText(dispImg, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
}

int main(int argc, char *argv[])
{
	PsReturnStatus status;
	uint32_t deviceIndex = 0;
	uint32_t deviceCount = 0;
	uint32_t slope = 1450;
	uint32_t depthmax = 1500;
	uint32_t depthmin = 170; 
 
	PsDepthRange depthRange = PsNearRange;
	PsDataMode dataMode = PsDepthAndRGB_30;
 
	status = Ps2_Initialize();
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsInitialize failed!" << endl;
		system("pause");
		return -1;
	}

GET:
	status = Ps2_GetDeviceCount(&deviceCount);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsGetDeviceCount failed!" << endl;
		system("pause");
		return -1;
	}
	cout << "Get device count: " << deviceCount << endl;
	if (0 == deviceCount)
	{
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
	}
 
	pDeviceListInfo = new PsDeviceInfo[deviceCount];
	status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount);
	status = Ps2_OpenDevice(pDeviceListInfo->uri, &deviceHandle);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "OpenDevice failed!" << endl;
		system("pause");
		return -1;
	}
	uint32_t sessionIndex = 0;

	status = Ps2_StartStream(deviceHandle, sessionIndex);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "StartStream failed!" << endl;
		system("pause");
		return -1;
	}

	status = Ps2_SetDataMode(deviceHandle, sessionIndex, dataMode);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "Ps2_SetDataMode failed!" << endl;
		system("pause");
		return -1;
	}
	status = Ps2_SetDepthRange(deviceHandle, sessionIndex, depthRange);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "Ps2_SetDataMode failed!" << endl;
		system("pause");
		return -1;
	}
	Vz_MapParams camparams;

	PsCameraParameters cameraParameters;
	status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsDepthSensor, &cameraParameters);

	cout << "Get PsGetCameraParameters status: " << status << endl;
	cout << "Depth Camera Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "Depth Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "K4: " << cameraParameters.k4 << endl;
	cout << "K5: " << cameraParameters.k5 << endl;
	cout << "K6: " << cameraParameters.k6 << endl;

	camparams.tofIntrinsic_fx = cameraParameters.fx;
	camparams.tofIntrinsic_fy = cameraParameters.fy;
	camparams.tofIntrinsic_cx = cameraParameters.cx;
	camparams.tofIntrinsic_cy = cameraParameters.cy;

	PsCameraExtrinsicParameters CameraExtrinsicParameters;
	status = Ps2_GetCameraExtrinsicParameters(deviceHandle, sessionIndex, &CameraExtrinsicParameters);

	cout << "Get PsGetCameraExtrinsicParameters status: " << status << endl;
	cout << "Camera rotation: " << endl;
	cout << CameraExtrinsicParameters.rotation[0] << " "
		<< CameraExtrinsicParameters.rotation[1] << " "
		<< CameraExtrinsicParameters.rotation[2] << " "
		<< CameraExtrinsicParameters.rotation[3] << " "
		<< CameraExtrinsicParameters.rotation[4] << " "
		<< CameraExtrinsicParameters.rotation[5] << " "
		<< CameraExtrinsicParameters.rotation[6] << " "
		<< CameraExtrinsicParameters.rotation[7] << " "
		<< CameraExtrinsicParameters.rotation[8] << " "
		<< endl;

	cout << "Camera transfer: " << endl;
	cout << CameraExtrinsicParameters.translation[0] << " "
		<< CameraExtrinsicParameters.translation[1] << " "
		<< CameraExtrinsicParameters.translation[2] << " " << endl;
 
	//rotation

	camparams.rotation[0] = CameraExtrinsicParameters.rotation[0];
	camparams.rotation[1] = CameraExtrinsicParameters.rotation[1];
	camparams.rotation[2] = CameraExtrinsicParameters.rotation[2];
	camparams.rotation[3] = CameraExtrinsicParameters.rotation[3];
	camparams.rotation[4] = CameraExtrinsicParameters.rotation[4];
	camparams.rotation[5] = CameraExtrinsicParameters.rotation[5];
	camparams.rotation[6] = CameraExtrinsicParameters.rotation[6];
	camparams.rotation[7] = CameraExtrinsicParameters.rotation[7];
	camparams.rotation[8] = CameraExtrinsicParameters.rotation[8];

	//transfer
	camparams.transfer[0] = CameraExtrinsicParameters.translation[0];
	camparams.transfer[1] = CameraExtrinsicParameters.translation[1];
	camparams.transfer[2] = CameraExtrinsicParameters.translation[2];


	status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsRgbSensor, &cameraParameters);

	cout << "Get PsGetCameraParameters status: " << status << endl;
	cout << "RGB Camera Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "RGB Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;

	camparams.rgbIntrinsic_fx = cameraParameters.fx;
	camparams.rgbIntrinsic_fy = cameraParameters.fy;
	camparams.rgbIntrinsic_cx = cameraParameters.cx;
	camparams.rgbIntrinsic_cy = cameraParameters.cy;

	uint16_t rgbResolution = 0;
	Ps2_GetRGBResolution(deviceHandle, sessionIndex, &rgbResolution);

	switch (rgbResolution)
	{
	case 0:
		camparams.rgbFrameWidth = 1920;
		camparams.rgbFrameHeigth = 1080;
		break;
	case 1:
		camparams.rgbFrameWidth = 1280;
		camparams.rgbFrameHeigth = 720;
		break;
	case 2:
		camparams.rgbFrameWidth = 640;
		camparams.rgbFrameHeigth = 480;
		break;
	case 3:
		camparams.rgbFrameWidth = 640;
		camparams.rgbFrameHeigth = 360;
		break;
	}
	camparams.tofFrameWidth = 640;
	camparams.tofFrameHeigth = 480;

	APIMapProcess * ptr = Get_APIMapProcess();
	if (ptr->PreProcInit(camparams))
	{
		cout << "PreProcInit error" << endl;
	}

 	//Enable the Depth and RGB synchronize feature
	Ps2_SetSynchronizeEnabled(deviceHandle, sessionIndex, true);

	//Set PixelFormat as PsPixelFormatBGR888 for opencv display
	Ps2_SetColorPixelFormat(deviceHandle, sessionIndex, PsPixelFormatBGR888);

	cv::Mat imageMat;
	const string depthImageWindow = "Depth Image";
	const string rgbImageWindow = "RGB Image";
	const string mappedDepthImageWindow = "MappedDepth Image";
	const string mappedRgbImageWindow = "MappedRGB Image";
	bool f_bMappedRGB = true;
	bool f_bMappedDepth = true;

	cout << "\n--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "Press following key to set corresponding feature:" << endl;
 	cout << "R/r: Change the RGB resolution: input corresponding index in terminal:" << endl;
	cout << "                             0: 1920*1080" << endl;
	cout << "                             1: 1280*720" << endl;
	cout << "                             2: 640*480" << endl;
	cout << "                             3: 640*360" << endl;
	cout << "Esc: Program quit " << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------\n" << endl;


	for (;;)
	{
	
		PsFrame depthFrame = { 0 };
		PsFrame rgbFrame = { 0 };

		// Read one frame before call PsGetFrame
		PsFrameReady frameReady = { 0 };
		status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);

		if (status != PsRetOK)
		{
			goto KEY;
		}

		//Get depth frame, depth frame only output in following data mode
		if (1 == frameReady.depth)
		{
			status = Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);

			if (depthFrame.pFrameData != NULL)
			{
				Opencv_Depth(slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
				cv::imshow(depthImageWindow, imageMat);
			}
			else
			{
				cout << "Ps2_GetFrame PsDepthFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

		
		//Get RGB frame, RGB frame only output in following data mode
		if (1 == frameReady.rgb)
		{
			status = Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);

			if (rgbFrame.pFrameData != NULL)
			{
				//Display the RGB Image
				imageMat = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
				cv::imshow(rgbImageWindow, imageMat);
			}
			else
			{
				cout << "Ps2_GetFrame PsRGBFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

		if (1 == frameReady.depth && 1 == frameReady.rgb)
		{
			Vz_Frame DepthFrame;
			DepthFrame.type = U16C1;
			DepthFrame.w = depthFrame.width;
			DepthFrame.h = depthFrame.height;
			DepthFrame.pBuf = depthFrame.pFrameData;

	 
			Vz_Frame RGBFrame;
			RGBFrame.type = U8C3;
			RGBFrame.w = rgbFrame.width;
			RGBFrame.h = rgbFrame.height;
			RGBFrame.pBuf = rgbFrame.pFrameData;

			Vz_Frame MappedRGBFrame;
			Vz_Frame MappedDepthFrame;

			ptr->PreProcDepth2RGB(DepthFrame, RGBFrame, &MappedRGBFrame);
			Mat MappedRGB = Mat(MappedRGBFrame.h, MappedRGBFrame.w, CV_8UC3, MappedRGBFrame.pBuf);
			cv::imshow("MappedRGB Image", MappedRGB);
			
			ptr->PreProcRGB2Depth(RGBFrame, DepthFrame, &MappedDepthFrame);
			Mat MappedDepth = Mat(MappedDepthFrame.h, MappedDepthFrame.w, CV_16UC1, MappedDepthFrame.pBuf);
			Opencv_Depth(slope, MappedDepth.rows, MappedDepth.cols, MappedDepthFrame.pBuf, MappedDepth);
			cv::imshow("MappedDepth Image", MappedDepth);

 		}
		 
	KEY:
		unsigned char key = waitKey(1);
		imageMat.release();

		if (key == 'R' || key == 'r')
		{
			cout << "please select RGB resolution to set: 0:1080P; 1:720P; 2:480P; 3:360P" << endl;
			int index = 0;
			cin >> index;
			if (cin.fail())
			{
				std::cout << "Unexpected input\n";
				cin.clear();
				cin.ignore(1024, '\n');
				continue;
			}
			else
			{
				cin.clear();
				cin.ignore(1024, '\n');
			}
			PsResolution resolution = PsRGB_Resolution_1920_1080;
			switch (index)
			{
			case 1:
				resolution = PsRGB_Resolution_1280_720;
				break;
			case 2:
				resolution = PsRGB_Resolution_640_480;
				break;
			case 3:
				resolution = PsRGB_Resolution_640_360;
				break;
			default:
				break;
			}
			Ps2_SetRGBResolution(deviceHandle, sessionIndex, resolution);


			status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsRgbSensor, &cameraParameters);

			cout << "Get PsGetCameraParameters status: " << status << endl;
			cout << "RGB Camera Intinsic: " << endl;
			cout << "Fx: " << cameraParameters.fx << endl;
			cout << "Cx: " << cameraParameters.cx << endl;
			cout << "Fy: " << cameraParameters.fy << endl;
			cout << "Cy: " << cameraParameters.cy << endl;
			cout << "RGB Distortion Coefficient: " << endl;
			cout << "K1: " << cameraParameters.k1 << endl;
			cout << "K2: " << cameraParameters.k2 << endl;
			cout << "K3: " << cameraParameters.k3 << endl;
			cout << "P1: " << cameraParameters.p1 << endl;
			cout << "P2: " << cameraParameters.p2 << endl;

			camparams.rgbIntrinsic_fx = cameraParameters.fx;
			camparams.rgbIntrinsic_fy = cameraParameters.fy;
			camparams.rgbIntrinsic_cx = cameraParameters.cx;
			camparams.rgbIntrinsic_cy = cameraParameters.cy;

			uint16_t rgbResolution = 0;
			Ps2_GetRGBResolution(deviceHandle, sessionIndex, &rgbResolution);

			switch (rgbResolution)
			{
			case 0:
				camparams.rgbFrameWidth = 1920;
				camparams.rgbFrameHeigth = 1080;
				break;
			case 1:
				camparams.rgbFrameWidth = 1280;
				camparams.rgbFrameHeigth = 720;
				break;
			case 2:
				camparams.rgbFrameWidth = 640;
				camparams.rgbFrameHeigth = 480;
				break;
			case 3:
				camparams.rgbFrameWidth = 640;
				camparams.rgbFrameHeigth = 360;
				break;
			}

			if (ptr->PreProcInit(camparams))
			{
				cout << "PreProcInit error" << endl;
			}
		}
		else if (key == 27)	//ESC Pressed
		{
			break;
		}
	}

	status = Ps2_CloseDevice(&deviceHandle);
	cout << "CloseDevice status: " << status << endl;

	status = Ps2_Shutdown();
	cout << "Shutdown status: " << status << endl;
	cv::destroyAllWindows();

	return 0;
}
