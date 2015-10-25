#include <iostream>
#include <fstream>
#include <OpenNI.h>
#include "onimeshfunctions.h"

namespace onimesh
{
	const int FRAME_DATA_MOD = 100;

	/// <summary>
	/// Creates a name for an output file
	/// </summary>
	std::string getOutputFileName(const char* outputDirectory, const char* inputFile, const char* fileExtension)
	{
		// If the path contains '/' characters
		if (std::string(inputFile).find_last_of('/') != std::string::npos)
		{
			// Check if the directory needs a trailing '/'
			if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
				return std::string(outputDirectory + std::string(inputFile).substr(std::string(inputFile).find_last_of('/') + 1) + std::string(fileExtension));

			return std::string(outputDirectory + std::string("/") + std::string(inputFile).substr(std::string(inputFile).find_last_of('/') + 1) + std::string(fileExtension));
		}
		// If the path contains '\' characters
		else if (std::string(inputFile).find_last_of('\\') == std::string::npos)
		{
			// Check if the directory needs a trailing '\'
			if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
				return std::string(outputDirectory + std::string(inputFile).substr(std::string(inputFile).find_last_of('\\') + 1) + std::string(fileExtension));

			return std::string(outputDirectory + std::string("\\") + std::string(inputFile).substr(std::string(inputFile).find_last_of('\\') + 1) + std::string(fileExtension));
		}

		// Otherwise the input file does not contain a path
		// Check if the directory needs a trailing '/'
		if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
			return std::string(std::string(outputDirectory) + std::string(inputFile) + std::string(fileExtension));

		return std::string(std::string(outputDirectory) + std::string("/") + std::string(inputFile) + std::string(fileExtension));
	}

	/// <summary>
	/// Reads oni input and exports data as excel docs
	/// </summary>
	void outputExcel(const int argc, const char** argv)
	{
		std::cout << "Start excel data output...\n";
		
        const char* outputDirectory = argv[1];
        char* inputFile;
		const int numberInputFiles = argc - 2;
		openni::Device device;
		openni::VideoStream ir;
		openni::VideoFrameRef irf;
		OniDepthPixel* pDepth;
		int frameHeight, frameWidth;
		long frameIndex, numberOfFrames;
		std::ofstream out;
        
        // Initialize openni
        openni::Status rc = openni::OpenNI::initialize();
        if (rc != openni::STATUS_OK)
        {
            printf("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
            exit(2);
        }

		// Output excel doc for each input file
		for (int w = 0; w < numberInputFiles; ++w)
		{
			inputFile = (char*)argv[w + 2];
			std::cout << "Working on file " << inputFile << "...\n";
			
			// Open the .oni file
			rc = device.open(inputFile);
            if (rc != openni::STATUS_OK)
            {
                printf("Couldn't open device\n%s\n", openni::OpenNI::getExtendedError());
                exit(3);
            }
            
            // Create the Video Stream
			rc = ir.create(device, openni::SENSOR_DEPTH);
            if (rc != openni::STATUS_OK)
            {
                printf("Couldn't create depth stream\n%s\n", openni::OpenNI::getExtendedError());
                exit(4);
            }
            
			// Device Check
			if (!device.isValid())
			{
				std::cerr << "\nThe device is not valid.\n";
				exit(5);
			}
            
			// Verify the device is a file
			if (!device.isFile())
			{
				std::cerr << "\nThe device is not a file.\n";
				exit(6);
			}
			std::cout << "File open success...\n";

			// Set playback controls
			openni::PlaybackControl* pbc = device.getPlaybackControl();
			pbc->setSpeed(-1);
			pbc->setRepeatEnabled(false);

			// Open output file
			std::string outputFile = getOutputFileName(outputDirectory, inputFile, ".csv");
			out.open(outputFile);

			// Read all frames
			numberOfFrames = pbc->getNumberOfFrames(ir);
			std::cout << "Start reading frame data...\n";
			rc = ir.start();
            if (rc != openni::STATUS_OK)
            {
                printf("Couldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
                exit(7);
            }
            
			while (true)
			{
				// Read a frame
				rc = ir.readFrame(&irf);
                if (rc != openni::STATUS_OK)
                {
                    printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
                    continue;
                }

				// Verify frame data is valid
				if (!irf.isValid())
				{
					std::cerr << "Error reading video stream frame\n";
					break;
				}

				// Gather frame data
				pDepth = (OniDepthPixel*)irf.getData();
				frameIndex = irf.getFrameIndex();

				// Skip unneeded frames
				if (frameIndex % FRAME_DATA_MOD == 0)
				{
					frameHeight = irf.getHeight();
					frameWidth = irf.getWidth();
					std::cout << "Processing " << frameWidth << "x" << frameHeight << " frame number " << frameIndex << "...\n";
					out << "FrameNumber=" << frameIndex << ",FrameWidth=" << frameWidth << ",FrameHeight=" << frameHeight << ",\n";
					for (int y = 0; y < frameHeight; ++y)  // All heights
					{
						for (int x = 0; x < frameWidth; ++x, ++pDepth)  // All witdths
						{
							out << *pDepth << ",";
						}
						out << "\n";
					}
					out << ",\n";
				}
				else
				{
					//std::cout << "Skipping frame " << frameIndex << "\n";
				}

				// Break if reading the last frame
				if (numberOfFrames == frameIndex)
				{
					std::cout << "Last frame has been read...\n";
					break;
				}
			}

			// Cleanup
			out.clear();
			out.close();
			ir.stop();
			ir.destroy();
			device.close();
		}

		// OpenNI cleanup
		openni::OpenNI::shutdown();
		std::cout << "Excel data output complete.\n";
	}

	/// <summary>
	/// Reads oni input and exports data as point clouds
	/// </summary>
	void outputPointCloud(const int argc, const char** argv)
	{

	}
}