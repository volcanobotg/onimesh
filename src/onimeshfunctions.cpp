#include <iostream>
#include <OpenNI.h>
#include "onimeshfunctions.h"

namespace onimesh
{
	/// <summary>
	/// Reads oni input and exports data as excel docs
	/// </summary>
	void outputExcel(int argc, char** argv)
	{
		const char* excelOutputDirectory = argv[1];
		const char* inputFile = argv[3];
		openni::Device device;
		openni::VideoStream ir;
		openni::VideoFrameRef irf;

		// Open the .oni file
		std::cout << "Start excel data output...\n";
		openni::OpenNI::initialize();
		device.open(inputFile);
		ir.create(device, openni::SENSOR_DEPTH);

		// Device Check
		if (!device.isValid())
		{
			std::cerr << "\nError opening oni file.\n";
			exit(2);
		}

		// Verify the device is a file
		if (!device.isFile())
		{
			std::cerr << "\nThe device is not a file.\n";
			exit(3);
		}
		std::cout << "File open success...\n";

		// Set playback controls
		openni::PlaybackControl* pbc = device.getPlaybackControl();
		pbc->setSpeed(-1);
		pbc->setRepeatEnabled(false);

		// Read all frames
		long numberOfFrames = pbc->getNumberOfFrames(ir);
		ir.start();
		std::cout << "Start reading frame data...\n";
		while (true)
		{
			// Read a frame
			ir.readFrame(&irf);

			// Verify frame data is valid
			if (!irf.isValid())
			{
				std::cerr << "Error reading video stream frame\n";
				break;
			}

			// Gather frame data
			OniDepthPixel* pDepth = (OniDepthPixel*)irf.getData();
			int frameHeight = irf.getHeight();
			int frameWidth = irf.getWidth();
			long frameIndex = irf.getFrameIndex();

			std::cout << "Frame: " << frameIndex << "\tHeight: " << frameHeight << "\tWidth: " << frameWidth << "\n";

			if (frameIndex % 100 == 0)
			{
				for (int y = 0; y < frameHeight; ++y)  // All heights
				{
					for (int x = 0; x < frameWidth; ++x, ++pDepth)  // All witdths
					{
						std::cout << *pDepth << " ";
					}
				}
			}

			// Break if reading the last frame
			if (numberOfFrames == frameIndex)
			{
				std::cout << "Last frame has been read...\n";
				break;
			}
		}

		// Cleanup
		ir.stop();
		ir.destroy();
		device.close();
		openni::OpenNI::shutdown();
		std::cout << "Excel data output complete.\n";
	}

	/// <summary>
	/// Reads oni input and exports data as point clouds
	/// </summary>
	void outputPointCloud(int argc, char** argv)
	{

	}
}