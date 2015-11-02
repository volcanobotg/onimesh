#include <iostream>
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
#include <vector>
#include "onimeshfunctions.h"

namespace onimesh
{
        
    boost::mutex mutex_;
    boost::shared_ptr<pcl::PCDGrabber<pcl::PointXYZRGBA> > grabber;
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;
    
    typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
    typedef Cloud::ConstPtr CloudConstPtr;
    
    int i = 0;
    char buf[4096];
    
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
    /*
     * Software License Agreement (BSD License)
     *
     *  Point Cloud Library (PCL) - www.pointclouds.org
     *  Copyright (c) 2011, Willow Garage, Inc.
     *  Copyright (c) 2012-, Open Perception, Inc.
     *
     *  All rights reserved.
     *
     *  Redistribution and use in source and binary forms, with or without
     *  modification, are permitted provided that the following conditions
     *  are met:
     *
     *   * Redistributions of source code must retain the above copyright
     *     notice, this list of conditions and the following disclaimer.
     *   * Redistributions in binary form must reproduce the above
     *     copyright notice, this list of conditions and the following
     *     disclaimer in the documentation and/or other materials provided
     *     with the distribution.
     *   * Neither the name of the copyright holder(s) nor the names of its
     *     contributors may be used to endorse or promote products derived
     *     from this software without specific prior written permission.
     *
     *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
     *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
     *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
     *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
     *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
     *  POSSIBILITY OF SUCH DAMAGE.
     *
     *
     */
       
    //////////////////////////////////////////////////////////////////////////////
	//cloud_cb 
	//Purpose: Takes a cloud object and writes it to a PCD file.
	//////////////////////////////////////////////////////////////////////////////
    void cloud_cb (const CloudConstPtr& cloud)
    {
        pcl::PCDWriter w;
        sprintf_s (buf, "frame_%06d.pcd", i);
        w.writeBinaryCompressed (buf, *cloud);
        PCL_INFO ("Wrote a cloud with %lu (%ux%u) points in %s.\n",
                  cloud->size (), cloud->width, cloud->height, buf);
        ++i;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////
	// outputPointCloud
	// Purpose: Makes a OpenNI2Grabber to read from the ONI file. As the file is read in cloud_cb
	//          is called to write the data to a PCD file. After the file has been read the 
	//			grabber is deleted.
	///////////////////////////////////////////////////////////////////////////////////////////          
	void outputPointCloud(const int argc, const char** argv)
	{
		//Writes a message to the console.
        pcl::console::print_info ("Convert an ONI file to PCD format.\n");
        
        //Initializes a grabber for the ONI file.
        pcl::io::OpenNI2Grabber* grabber = new pcl::io::OpenNI2Grabber (argv[2]);
		//Calls boost to set up writing to PCD
        boost::function<void (const CloudConstPtr&) > f = boost::bind (&cloud_cb, _1);
		//Callback to let the program know when the file has been written.
        boost::signals2::connection c = grabber->registerCallback (f);
        
		//Do-while loop to read in all of the frames from the ONI file
        do
        {
            grabber->start ();
        }while (grabber->isRunning ());
        
        PCL_INFO ("Successfully processed %d frames.\n", i);
        
        delete grabber;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //This is the area for using the pointCloudViewer
    
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
}