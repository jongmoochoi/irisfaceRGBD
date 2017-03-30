///*
//	main.cpp
//	entry point
//
//	author: Jatuporn Toy Leksut
//	version 2.2 04/27/2015
//*/
//
//#include "poseEstimation.h"
//
//
//SampleViewer *g_sampleViewer;
//
//int main(int argc, char **argv) 
//{
//	//dlib
//
//
//	if(argc != 3) {
//		printf("Usage: expr <configfile> <input_dir>\n");
//		exit(-1);
//	}  
//
/////////////// init
//	char *a = "../config/config.json";
//	char  *b = "../data";
//
//	argv[1] = a;
//	argv[2] = b;
//	
//	initExpr(argv[1]);
//
//
/////////////// start
//
//	openni::Status rc = openni::STATUS_OK;
//
//	openni::Device device;
//	openni::VideoStream depth, color;
//	const char* deviceURI = openni::ANY_DEVICE;
//
//	rc = openni::OpenNI::initialize();
//
//	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
//
//	rc = device.open(deviceURI);
//	if (rc != openni::STATUS_OK)
//	{
//		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
//		openni::OpenNI::shutdown();
//		return 1;
//	}
//
//	rc = depth.create(device, openni::SENSOR_DEPTH);
//	if (rc == openni::STATUS_OK)
//	{
//		rc = depth.start();
//		if (rc != openni::STATUS_OK)
//		{
//			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
//			depth.destroy();
//		}
//	}
//	else
//	{
//		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
//	}
//
//	rc = color.create(device, openni::SENSOR_COLOR);
//	if (rc == openni::STATUS_OK)
//	{
//		rc = color.start();
//		if (rc != openni::STATUS_OK)
//		{
//			printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
//			color.destroy();
//		}
//	}
//	else
//	{
//		printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
//	}
//
//	if (!depth.isValid() || !color.isValid())
//	{
//		printf("SimpleViewer: No valid streams. Exiting\n");
//		openni::OpenNI::shutdown();
//		return 2;
//	}
//
//	SampleViewer sampleViewer("Simple Viewer", device, depth, color);
//
//	g_sampleViewer = &sampleViewer;
//
//	rc = sampleViewer.init(argc, argv);
//	if (rc != openni::STATUS_OK)
//	{
//		openni::OpenNI::shutdown();
//		return 3;
//	}
//
//	sampleViewer.run();
//	return 0;
//}
//
//
