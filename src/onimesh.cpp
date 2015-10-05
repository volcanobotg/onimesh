#include <iostream>
#include <OpenNI.h>
#include "onimeshfunctions.h"

int main(int argc, char** argv)
{
	// Argument check
	if (argc < 4)
	{
		std::cerr << "\nUsage: onimesh <excelOutDir> <pointcloudOutDir> <OniInputFiles>\n"
			<< "       excelOutDir - Output directory for excel data files\n"
			<< "  pointCloudOutDir - Output directory for point cloud data files\n"
			<< "     oniInputFiles - One or more .oni input files\n";
		exit(1);
	}

	// Reads oni input and exports data as excel docs
	onimesh::outputExcel(argc, argv);

	std::cout << "Onimesh output generation complete.\n";
	exit(0);
}