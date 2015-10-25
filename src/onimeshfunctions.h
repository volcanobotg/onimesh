// Macroguard
#ifndef _onimeshfunctions
#define _onimeshfunctions

namespace onimesh
{
	/// <summary>
	/// Creates a name for an output file
	/// </summary>
	std::string getOutputFileName(const char* outputDirectory, const char* inputFile, const char* fileExtension);

	/// <summary>
	/// Reads oni input and exports data as excel docs
	/// </summary>
	void outputExcel(const int argc, const char** argv);

	/// <summary>
	/// Reads oni input and exports data as point clouds
	/// </summary>
	void outputPointCloud(const int argc, const char** argv);

	/// <summary>
	/// This is a dummy example to show the use of writing a pcl file
	/// that contains a 5x2 cloud with random values
	/// Code taken from here:
	/// http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd
	/// </summary>
	void createDummyPointCloud();
}

// End Macroguard
#endif
