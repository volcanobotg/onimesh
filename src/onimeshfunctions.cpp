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
    void printHelp (int, char **argv)
    {
        pcl::console::print_error ("Syntax is: %s input.oni\n", argv[0]);
    }
    
    //////////////////////////////////////////////////////////////////////////////
    void cloud_cb (const CloudConstPtr& cloud)
    {
        pcl::PCDWriter w;
        sprintf (buf, "frame_%06d.pcd", i);
        w.writeBinaryCompressed (buf, *cloud);
        PCL_INFO ("Wrote a cloud with %lu (%ux%u) points in %s.\n",
                  cloud->size (), cloud->width, cloud->height, buf);
        ++i;
    }
    
    /* ---[ */
	void outputPointCloud(int argc, char** argv)
	{
        pcl::console::print_info ("Convert an ONI file to PCD format. For more information, use: %s -h\n", argv[0]);
        
        
        pcl::io::OpenNI2Grabber* grabber = new pcl::io::OpenNI2Grabber (argv[1]);
        boost::function<void (const CloudConstPtr&) > f = boost::bind (&cloud_cb, _1);
        boost::signals2::connection c = grabber->registerCallback (f);
        
        do
        {
            grabber->start ();
        }while (grabber->isRunning ());
        
        PCL_INFO ("Successfully processed %d frames.\n", i);
        
        delete grabber;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //This is the area for using the pointCloudViewer
    
    void printHelp (int, char **argv)
    {
        //print_error ("Syntax is: %s <file_name 1..N>.pcd <options>\n", argv[0]);
        pcl::console::print_error ("Syntax is: %s <options>\n", argv[0]);
        pcl::console::print_info ("  where options are:\n");
        pcl::console::print_info ("                     -file file_name          = PCD file to be read from\n");
        pcl::console::print_info ("                     -dir directory_path      = directory path to PCD file(s) to be read from\n");
        pcl::console::print_info ("                     -fps frequency           = frames per second\n");
        pcl::console::print_info ("                     -repeat                  = optional parameter that tells whether the PCD file(s) should be \"grabbed\" in a endless loop.\n");
        pcl::console::print_info ("\n");
        pcl::console::print_info ("                     -cam (*)                 = use given camera settings as initial view\n");
        pcl::console::print_info (stderr, " (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Window Size / Window Pos] or use a <filename.cam> that contains the same information.\n");
    }
    
    // Create the PCLVisualizer object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer;
#ifdef DISPLAY_IMAGE
    boost::shared_ptr<pcl::visualization::ImageViewer> img_viewer;
#endif
    
    std::vector<double> fcolor_r, fcolor_b, fcolor_g;
    bool fcolorparam = false;
    
    struct EventHelper
    {
        void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
        {
            if (mutex_.try_lock ())
            {
                cloud_ = cloud;
                mutex_.unlock ();
            }
        }
    };
    
    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
        /// If SPACE is pressed, trigger new cloud callback (only works if framerate is set to 0)
        if (event.getKeyCode() == ' ' && grabber)
            grabber->trigger ();
    }
    
    void mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
    {
        std::string* message = static_cast<std::string*> (cookie);
        if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
        {
            cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
        }
    }
    
    /* ---[ */


    void pointCloudViewer(argc, argv)
    {
        srand (unsigned (time (0)));
        
        if (argc > 1)
        {
            for (int i = 1; i < argc; i++)
            {
                if (std::string (argv[i]) == "-h")
                {
                    printHelp (argc, argv);
                    return (-1);
                }
            }
        }
        
        // Command line parsing
        double bcolor[3] = {0, 0, 0};
        pcl::console::parse_3x_arguments (argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2]);
        
        fcolorparam = pcl::console::parse_multiple_3x_arguments (argc, argv, "-fc", fcolor_r, fcolor_g, fcolor_b);
        
        int psize = 0;
        pcl::console::parse_argument (argc, argv, "-ps", psize);
        
        double opaque;
        pcl::console::parse_argument (argc, argv, "-opaque", opaque);
        
        cloud_viewer.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));
        
#ifdef DISPLAY_IMAGE
        img_viewer.reset (new pcl::visualization::ImageViewer ("OpenNI Viewer"));
#endif
        
        //  // Change the cloud rendered point size
        //  if (psize > 0)
        //    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "OpenNICloud");
        //
        //  // Change the cloud rendered opacity
        //  if (opaque >= 0)
        //    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque, "OpenNICloud");
        
        cloud_viewer->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);
        
        // Read axes settings
        double axes = 0.0;
        pcl::console::parse_argument (argc, argv, "-ax", axes);
        if (axes != 0.0 && cloud_viewer)
        {
            float ax_x = 0.0, ax_y = 0.0, ax_z = 0.0;
            pcl::console::parse_3x_arguments (argc, argv, "-ax_pos", ax_x, ax_y, ax_z, false);
            // Draw XYZ axes if command-line enabled
            cloud_viewer->addCoordinateSystem (axes, ax_x, ax_y, ax_z, "global");
        }
        
        float frames_per_second = 0; // 0 means only if triggered!
        pcl::console::parse (argc, argv, "-fps", frames_per_second);
        if (frames_per_second < 0)
            frames_per_second = 0.0;
        
        
        bool repeat = (pcl::console::find_argument (argc, argv, "-repeat") != -1);
        
        std::cout << "fps: " << frames_per_second << " , repeat: " << repeat << std::endl;
        std::string path = "";
        pcl::console::parse_argument (argc, argv, "-file", path);
        std::cout << "path: " << path << std::endl;
        if (path != "" && boost::filesystem::exists (path))
        {
            grabber.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (path, frames_per_second, repeat));
        }
        else
        {
            std::vector<std::string> pcd_files;
            pcl::console::parse_argument (argc, argv, "-dir", path);
            std::cout << "path: " << path << std::endl;
            if (path != "" && boost::filesystem::exists (path))
            {
                boost::filesystem::directory_iterator end_itr;
                for (boost::filesystem::directory_iterator itr (path); itr != end_itr; ++itr)
                {
#if BOOST_FILESYSTEM_VERSION == 3
                    if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".PCD" )
#else
                        if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->leaf ())) == ".PCD" )
#endif
                        {
#if BOOST_FILESYSTEM_VERSION == 3
                            pcd_files.push_back (itr->path ().string ());
                            std::cout << "added: " << itr->path ().string () << std::endl;
#else
                            pcd_files.push_back (itr->path ().string ());
                            std::cout << "added: " << itr->path () << std::endl;
#endif
                        }
                }
            }
            else
            {
                std::cout << "Neither a pcd file given using the \"-file\" option, nor given a directory containing pcd files using the \"-dir\" option." << std::endl;
            }
            
            // Sort the read files by name
            sort (pcd_files.begin (), pcd_files.end ());
            grabber.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (pcd_files, frames_per_second, repeat));
        }
        
        EventHelper h;
        boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = boost::bind (&EventHelper::cloud_cb, &h, _1);
        boost::signals2::connection c1 = grabber->registerCallback (f);
        
        std::string mouse_msg_3D ("Mouse coordinates in PCL Visualizer");
        std::string key_msg_3D ("Key event for PCL Visualizer");
        
        cloud_viewer->registerMouseCallback (&mouse_callback, static_cast<void*> (&mouse_msg_3D));
        cloud_viewer->registerKeyboardCallback(&keyboard_callback, static_cast<void*> (&key_msg_3D));
        
        std::string mouse_msg_2D ("Mouse coordinates in image viewer");
        std::string key_msg_2D ("Key event for image viewer");
        
#ifdef DISPLAY_IMAGE
        img_viewer->registerMouseCallback (&mouse_callback, static_cast<void*> (&mouse_msg_2D));
        img_viewer->registerKeyboardCallback(&keyboard_callback, static_cast<void*> (&key_msg_2D));
#endif
        
        grabber->start ();
        while (!cloud_viewer->wasStopped ())
        {
            cloud_viewer->spinOnce ();
            
#ifdef DISPLAY_IMAGE
            img_viewer->spinOnce ();
#endif
            
            if (!cloud_)
            {
                boost::this_thread::sleep(boost::posix_time::microseconds(10000));
                continue;
            }
            else if (mutex_.try_lock ())
            {
                pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr temp_cloud;
                temp_cloud.swap (cloud_);
                mutex_.unlock ();
                
#ifdef DISPLAY_IMAGE
                img_viewer->showRGBImage (*temp_cloud);
#endif
                
                if (!cloud_viewer->updatePointCloud (temp_cloud, "PCDCloud"))
                {
                    cloud_viewer->addPointCloud (temp_cloud, "PCDCloud");
                    cloud_viewer->resetCameraViewpoint ("PCDCloud");
                }
            }
        }
        
        grabber->stop ();
    
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
}