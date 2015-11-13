/*
* Copyright (c) 2009-2012, Willow Garage, Inc.
* Copyright (c) 2012-, Open Perception, Inc.
* Copyright (c) 2015-, Christopher Padgett, David Andrews, Luis Henrique Fae Barboza
* 
* You may use, distribute and modify this code under the terms of the BSD license.
*
* You should have received a copy of the full BSD license with this file.
* If not, please visit: https://github.com/volcanobotg/onimesh for full license information.
*/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <OpenNI.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/Grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <boost/filesystem.hpp>
#include <vector>
#include "onimeshfunctions.h"

namespace onimesh
{
        
    boost::mutex mutex_;
    boost::shared_ptr<pcl::PCDGrabber<pcl::PointXYZRGBA> > grabber;
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;
    
    typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
    typedef Cloud::ConstPtr CloudConstPtr;
	
    int frameCounter = 0;
	int fileCounter = 0;
    char buf[4096];
    long totalFrameNumber[200];
	std::string pointCloudOutputPath;
    
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
				return std::string(std::string(outputDirectory) + std::string(inputFile).substr(std::string(inputFile).find_last_of('/') + 1) + std::string(fileExtension));

			return std::string(std::string(outputDirectory) + std::string("/") + std::string(inputFile).substr(std::string(inputFile).find_last_of('/') + 1) + std::string(fileExtension));
		}
		// If the path contains '\' characters
		else if (std::string(inputFile).find_last_of('\\') == std::string::npos)
		{
			// Check if the directory needs a trailing '\'
			if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
				return std::string(std::string(outputDirectory) + std::string(inputFile).substr(std::string(inputFile).find_last_of('\\') + 1) + std::string(fileExtension));

			return std::string(std::string(outputDirectory) + std::string("\\") + std::string(inputFile).substr(std::string(inputFile).find_last_of('\\') + 1) + std::string(fileExtension));
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
            // Fills a global array to use for each files maximum frame number.
            totalFrameNumber[w] = numberOfFrames;
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
	std::string getOutputFilePath(const char* outputDirectory, const char* inputFile)
	{
		// If the path contains '/' characters
		if (std::string(inputFile).find_last_of('/') != std::string::npos)
		{
			// Check if the directory needs a trailing '/'
			if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
				return std::string(std::string(outputDirectory) + std::string(inputFile).substr(std::string(inputFile).find_last_of('/') + 1) );

			return std::string(std::string(outputDirectory) + std::string("/") + std::string(inputFile).substr(std::string(inputFile).find_last_of('/') + 1));
		}
		// If the path contains '\' characters
		else if (std::string(inputFile).find_last_of('\\') == std::string::npos)
		{
			// Check if the directory needs a trailing '\'
			if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
				return std::string(std::string(outputDirectory) + std::string(inputFile).substr(std::string(inputFile).find_last_of('\\') + 1) );

			return std::string(std::string(outputDirectory) + std::string("\\") + std::string(inputFile).substr(std::string(inputFile).find_last_of('\\') + 1) );
		}

		// Otherwise the input file does not contain a path
		// Check if the directory needs a trailing '/'
		if (std::string(outputDirectory).back() == '/' || std::string(outputDirectory).back() == '\\')
			return std::string(std::string(outputDirectory) + std::string(inputFile) );

		return std::string(std::string(outputDirectory) + std::string("/") + std::string(inputFile));
	}
    //////////////////////////////////////////////////////////////////////////////
	//cloud_cb 
	//Purpose: Takes a cloud object and writes it to a PCD file.
	//////////////////////////////////////////////////////////////////////////////
    void cloud_cb (const CloudConstPtr& cloud)
    {
        if (frameCounter <= totalFrameNumber[fileCounter])
        {
            if ( frameCounter % 150 == 0)
            {
                pcl::PCDWriter w;
                sprintf_s (buf, "frame_%06d.pcd", frameCounter);
                w.writeBinaryCompressed (buf, *cloud);
                PCL_INFO ("Wrote a cloud with %lu (%ux%u) points in %s.\n",
                          cloud->size (), cloud->width, cloud->height, buf);
            }
        }
        ++frameCounter;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////
	// outputPointCloud
	// Purpose: Makes a OpenNI2Grabber to read from the ONI file. As the file is read in cloud_cb
	//          is called to write the data to a PCD file. After the file has been read the 
	//			grabber is deleted.
	///////////////////////////////////////////////////////////////////////////////////////////          
	void outputPointCloud(const int argc, const char** argv)
	{
        bool myStopBool = false;
		const char* pointCloudOutputDirectory = argv[1];
		char* pointCloudInputFile;
		std::ofstream out;
		
        for (int j = 2; j < argc; j++)
        {
            myStopBool = false;
			frameCounter = 0;
			fileCounter = j - 2;
			pointCloudInputFile = (char*)argv[j];
			
			pointCloudOutputPath = getOutputFilePath(pointCloudOutputDirectory, pointCloudInputFile);
			out.open(pointCloudOutputPath);
			//Writes a message to the console.
            std::cout<< "Converting " << pointCloudInputFile <<" to PCD format.\n";
        
            //Initializes a grabber for the ONI file.
            pcl::io::OpenNI2Grabber* grabber = new pcl::io::OpenNI2Grabber (argv[j]);
            //Calls boost to set up writing to PCD
            boost::function<void (const CloudConstPtr&) > f = boost::bind (&cloud_cb, _1);
            //Callback to let the program know when the file has been written.
            boost::signals2::connection c = grabber->registerCallback (f);
            
            //Do-while loop to read in all of the frames from the ONI file
            while (myStopBool != true)
            {
                if ( frameCounter <= totalFrameNumber[fileCounter])
                {
                    grabber->start ();
					boost::this_thread::sleep(boost::posix_time::seconds(1));
                    if (frameCounter == totalFrameNumber[fileCounter] || (frameCounter + 50) >= totalFrameNumber[fileCounter])
                    {
                        myStopBool = true;
                    }
                }
				else
				{
					myStopBool = true;
				}
            }
        
            PCL_INFO ("Successfully processed %d frames.\n", totalFrameNumber[fileCounter]);
        
            delete grabber;
        }
    }
    
    
    
    /// <summary>
	/// This is a dummy example to show the use of writing a pcl file
	/// that contains a 5x2 cloud with random values
	/// Code taken from here:
	/// http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd
	/// </summary>
	void createDummyPointCloud()
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;

		// Fill in the cloud data
		cloud.width = 5;
		cloud.height = 2;
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);
        
		for (size_t i = 0; i < cloud.points.size(); ++i)
		{
			cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		}

		pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
		std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

		for (size_t i = 0; i < cloud.points.size(); ++i)
			std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
	}

	
	void staticMesh(std::string pcdFileName) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFileName, *cloud) == -1) //this loads the file
		{
			PCL_ERROR("Couldn't read file \n");
		}

		float dist, distLowest, distLowest2;
		int distIndex, distIndex2;

		for (size_t i = 0; i < cloud->points.size(); i++) {
			for (size_t j = 0; j < cloud->points.size(); j++) {
				//Only calculates if the indexes are different, because otherwise the distance is zero
				if (i != j) {
					//Calculates the distance between I and J in the tridimensional plane
					dist = sqrt(pow((cloud->points[j].x - cloud->points[i].x), 2) +
								pow((cloud->points[j].y - cloud->points[i].y), 2) +
								pow((cloud->points[j].z - cloud->points[i].z), 2));

					if (j == 0) {
						distLowest = dist;
						distIndex = j;
					}
					else if (j == 1) {
						if (i == 0) {
							distLowest = dist;
							distIndex = j;
						}
						else {
							if (dist < distLowest) {
								distLowest2 = distLowest;
								distIndex2 = distIndex;
								distLowest = dist;
								distIndex = j;
							}
							else {
								distLowest2 = dist;
								distIndex2 = j;
							}
						}
						
					}
					else if ((i == 0 || i == 1) && j == 2) {
						if (dist < distLowest) {
							distLowest2 = distLowest;
							distIndex2 = distIndex;
							distLowest = dist;
							distIndex = j;
						}
						else {
							distLowest2 = dist;
							distIndex2 = j;
						}
					}
					else if (dist < distLowest) {
						distLowest2 = distLowest;
						distIndex2 = distIndex;
						distLowest = dist;
						distIndex = j;
					}
					else if (dist < distLowest2) {
						distLowest2 = dist;
						distIndex2 = j;
					}
				}
				
			}
			//you have the two closest points to i now in distLowest and distLowest2, with their indexes distIndex and distIndex2, respectively
			//now we have to draw lines to form a triangle using "i", "distIndex" and "distIndex2"

		}
	}


	
}