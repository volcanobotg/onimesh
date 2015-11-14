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

#include <boost/filesystem.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <OpenNI.h>
#include <OniProperties.h>
#include <pcl/io/pcd_io.h>
#include "filesystemHelper.h"
#include "onimeshfunctions.h"

namespace onimesh
{
	const int FRAME_DATA_MOD = 100;

	/// <summary>
	/// Exports a depth frame to the excel data file
	/// </summary>
	void outputFrameToCsv(std::ofstream& outFileStream, const openni::VideoFrameRef frameReference)
	{
		OniDepthPixel* pDepth = (OniDepthPixel*)frameReference.getData();
		int frameHeight = frameReference.getHeight();
		int frameWidth = frameReference.getWidth();

		// Output the frame header
		std::cout << "Processing " << frameWidth << "x" << frameHeight << " frame number " << frameReference.getFrameIndex() << "...\n";
		outFileStream << "FrameNumber=" << frameReference.getFrameIndex() << ",FrameWidth=" << frameWidth << ",FrameHeight=" << frameHeight << ",\n";

		// All heights of the frame
		for (int y = 0; y < frameHeight; ++y)
		{
			// All widths of the frame
			for (int x = 0; x < frameWidth; ++x, ++pDepth)
			{
				outFileStream << *pDepth << ",";
			}
			outFileStream << "\n";
		}
		outFileStream << ",\n";
	}

	/// <summary>
	/// Exports a depth frame to the excel data file
	/// </summary>
	void outputFrameToPcd(const std::string outputFrameDirectory, const openni::Device* device, const openni::VideoFrameRef frameReference)
	{
		// Set cloud meta data
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
		cloud->header.seq = frameReference.getFrameIndex();
		cloud->header.frame_id = frameReference.getFrameIndex();
		cloud->header.stamp = frameReference.getTimestamp();
		cloud->height = frameReference.getHeight();
		cloud->width = frameReference.getWidth();
		cloud->is_dense = false;
		cloud->points.resize(cloud->height * cloud->width);

		// Get the device field of view
		int xFov = 0, yFov = 0;
		device->getProperty(openni::STREAM_PROPERTY_HORIZONTAL_FOV, &xFov);
		device->getProperty(openni::STREAM_PROPERTY_VERTICAL_FOV, &yFov);

		// Use field of view to calculate focal point which is used to calculate constant height and width
		float constant_x = 1.0f / (((float)cloud->width / 2) / tan(xFov));
		float constant_y = 1.0f / (((float)cloud->height / 2) / tan(yFov));
		
		// Calculate the center of height and width
		float centerX = ((float)cloud->width - 1.f) / 2.f;
		float centerY = ((float)cloud->height - 1.f) / 2.f;

		// Set bad point value
		float bad_point = std::numeric_limits<float>::quiet_NaN();

		// All widths in frame
		const uint16_t* depth_map = (const uint16_t*)frameReference.getData();
		int depth_idx = 0;
		for (int v = 0; v < cloud->width; ++v)
		{
			// All heights in frame
			for (int u = 0; u < cloud->height; ++u, ++depth_idx)
			{
				pcl::PointXYZ& pt = cloud->points[depth_idx];
				// Check for invalid measurements
				if (depth_map[depth_idx] == 0)
				{
					// not valid
					pt.x = pt.y = pt.z = bad_point;
					continue;
				}

				// Set the x,y,z values
				pt.z = depth_map[depth_idx] * 0.001f;
				pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
				pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
			}
		}

		// Set the point cloud orientation
		cloud->sensor_origin_.setZero();
		cloud->sensor_orientation_.w() = 1.0f;
		cloud->sensor_orientation_.x() = 0.0f;
		cloud->sensor_orientation_.y() = 0.0f;
		cloud->sensor_orientation_.z() = 0.0f;

		// Output the point cloud to a pcd frame file
		std::stringstream outputFrameFile;
		outputFrameFile << outputFrameDirectory << "frame_" << std::setw(10) << std::setfill('0') << frameReference.getFrameIndex() << ".pcd";
		pcl::PCDWriter w;
		w.writeBinaryCompressed(outputFrameFile.str(), *cloud);
	}

	/// <summary>
	/// Reads oni input and exports data as excel docs and point clouds
	/// </summary>
	void outputOniData(const int argc, const char** argv)
	{
		std::cout << "\nStart oni data output...\n\n";
		
        const std::string outputDirectory = boost::filesystem::absolute(argv[1]).string();
        char* inputFile;
		const int numberInputFiles = argc - 2;
		openni::Device device;
		openni::VideoStream ir;
		openni::VideoFrameRef irf;
		long frameIndex, numberOfFrames;
		std::ofstream out;
        
		// Create Output directory
		if (!onimesh::createDirectory(outputDirectory))
			exit(2);

        // Initialize openni
        openni::Status rc = openni::OpenNI::initialize();
        if (rc != openni::STATUS_OK)
        {
            printf("\nInitialize failed\n%s\n", openni::OpenNI::getExtendedError());
            exit(3);
        }

		// Output each input file
		for (int w = 0; w < numberInputFiles; ++w)
		{
			inputFile = (char*)argv[w + 2];
			std::cout << "Working on file " << inputFile << "...\n";
			
			// Open the .oni file
			rc = device.open(inputFile);
            if (rc != openni::STATUS_OK)
            {
                printf("\nCouldn't open device\n%s\n", openni::OpenNI::getExtendedError());
                exit(4);
            }
            
            // Create the Video Stream
			rc = ir.create(device, openni::SENSOR_DEPTH);
            if (rc != openni::STATUS_OK)
            {
                printf("\nCouldn't create depth stream\n%s\n", openni::OpenNI::getExtendedError());
                exit(5);
            }
            
			// Device Check
			if (!device.isValid())
			{
				std::cerr << "\nThe device is not valid.\n";
				exit(6);
			}
            
			// Verify the device is a file
			if (!device.isFile())
			{
				std::cerr << "\nThe device is not a file.\n";
				exit(7);
			}
			std::cout << "File open success...\n";

			// Set playback controls
			openni::PlaybackControl* pbc = device.getPlaybackControl();
			pbc->setSpeed(-1);
			pbc->setRepeatEnabled(false);

			// Delete point cloud output directory if it exists, we don't want old frame data
			std::string pointCloudOutputDirectory = onimesh::getOutputFileName(outputDirectory, inputFile, "");
			if (!onimesh::deleteDirectory(pointCloudOutputDirectory))
				exit(8);

			// Create point cloud output directory
			if (!onimesh::createDirectory(pointCloudOutputDirectory))
				exit(9);

			// Open csv output file for writing
			std::string csvOutputFile = onimesh::getOutputFileName(outputDirectory, inputFile, ".csv");
			out.open(csvOutputFile);

			// Start reading the frame data
			numberOfFrames = pbc->getNumberOfFrames(ir);
			std::cout << "Start reading frame data...\n";
			rc = ir.start();
            if (rc != openni::STATUS_OK)
            {
                printf("\nCouldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
                exit(10);
            }
            
			// Read all frames
			while (true)
			{
				// Read a frame
				rc = ir.readFrame(&irf);
                if (rc != openni::STATUS_OK)
                {
                    printf("\nRead failed!\n%s\n", openni::OpenNI::getExtendedError());
                    continue;
                }

				// Verify frame data is valid
				if (!irf.isValid())
				{
					std::cerr << "\nError reading video stream frame\n";
					break;
				}

				// Get the frame index number
				frameIndex = irf.getFrameIndex();

				// Skip unneeded frames
				if (frameIndex % FRAME_DATA_MOD == 0)
				{
					onimesh::outputFrameToCsv(out, irf);
					onimesh::outputFrameToPcd(pointCloudOutputDirectory + '/', &device, irf);
				}
				else
				{
					//std::cout << "Skipping frame " << frameIndex << "\n";
				}

				// Break if reading the last frame
				if (numberOfFrames == frameIndex)
				{
					std::cout << "Last frame has been read...\n\n";
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
		std::cout << "Oni data output complete.\n";
	}

	/// <summary>
	/// Creates a static meshes from point cloud data files
	/// </summary>  
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
				// Only calculates if the indexes are different, because otherwise the distance is zero
				if (i != j) {
					// Calculates the distance between I and J in the tridimensional plane
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
			// you have the two closest points to i now in distLowest and distLowest2, with their indexes distIndex and distIndex2, respectively
			// now we have to draw lines to form a triangle using "i", "distIndex" and "distIndex2"

		}
	}
}