/*
* Example of using the GOG C++ implementation on the UA-DETRAC dataset
*/

#include <iostream>
#include <fstream>
//#include <cstdlib>
#include <cstdio>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "dirent.h"

#include "GOG.h"
#include "UA-DETRAC.h"
#include "YOLO.h"

/******************************************************************************
* EXTRA FUNCTIONS
******************************************************************************/
int pgm_select(const struct dirent *entry)
{
	return strstr(entry->d_name, ".pgm") != NULL;
}

int ppm_select(const struct dirent *entry)
{
	return strstr(entry->d_name, ".ppm") != NULL;
}

int jpg_select(const struct dirent *entry)
{
	return strstr(entry->d_name, ".jpg") != NULL;
}

/******************************************************************************
* DEFINE SECTION
******************************************************************************/

#define DEFAULT_OUTPUT "result"
#define DEFAULT_DETECTIONS_TYPE "DETRAC"
#define DEFAULT_DETECTIONS_FILE "detections.txt"
#define SHOW_BOXES 1 // Program will output boxes after finish tracking
#define SAVE_BOXES 0 // Save the boxes on image file
// Turn on/off features depending if this will be embedded in UA-DETRAC toolkit or not
#define USE_IN_DETRAC 1

/******************************************************************************
* MAIN
******************************************************************************/

int main(int argc, char *argv[])
{
	std::cout << "GOG C++ implementation" << std::endl;
	// General variables
	std::string input_folder;
	std::string output_folder = DEFAULT_OUTPUT;
	std::string sequence;
	char* detections_type = DEFAULT_DETECTIONS_TYPE;
	char* detections_file = DEFAULT_DETECTIONS_FILE; // Because of sprintf_s needs to be char* .....
	struct dirent **filelist;
	int fcount = -1;
	clock_t begin, end;
	// GOG necessary variables
	Detections dres;
	bool nms = true; // use non-maximum supression

	/// Read arguments
	// TODO: CONTROL OVER THIS OPTIONS, ESPECIALLY DETECTIONS ONE
#if USE_IN_DETRAC
	// executable_name sequence input_folder
	if (argc >= 2)
	{
		sequence = argv[1];
		input_folder = argv[2];
	}
#else 
	// executable_name sequence input_folder det_type det_file output_folder
	if (argc >= 6)
	{
		sequence = argv[1];
		input_folder = argv[2];
		detections_type = argv[3];
		detections_file = argv[4];
		output_folder = argv[5];
	}
#endif

	// End of Added by Lucas
	std::cout << "input folder: " << input_folder << std::endl;

	/// Loading detections part
	// First verify that the detections file exists
	std::ifstream detStream(detections_file);
	if (!detStream || detStream.eof())
	{
		std::cout << "ERROR -> Detection file not found or empty" << std::endl;
		cv::waitKey(0);
		return 0;
	}
	else
	{
		std::cout << "Detection file found. Loading detections..." << std::endl;
		// Load all detections before starting tracking.
#if USE_IN_DETRAC
		UADETRAC::read_detections(detStream, dres);
#else
		if (0 == strcmp(detections_type, "DETRAC")) {
			UADETRAC::read_detections(detStream, dres);
		}
		else if (0 == strcmp(detections_type, "YOLO")) {
			YOLO::read_detections(detStream, dres);
		}
#endif
	}

	//std::cout << "Amount of lines read -> " << dres.x.size() << std::endl;
	//std::cout << "Amount of images -> " << dres.frame.back() << std::endl;
	//return 0;

#if !USE_IN_DETRAC
#if SHOW_BOXES
	/// Loading images part
	fcount = scandir(input_folder.c_str(), &filelist, jpg_select, alphasort);
	if (fcount <= 0)
	{
		std::cout << "ERROR -> Input images directory not found or empty" << std::endl;
		cv::waitKey(0);
		return 0;
	}
	else
		std::cout << "Found " << fcount << " images" << std::endl;
#endif // SHOW_BOXES
#endif // NOT USE_IN_DETRAC
	/// Looking for the output directory part. Create it if does not exit
	//	std::cout << "output folder: " << output_folder << std::endl;
	DIR * dir = opendir(output_folder.c_str());
	if (dir == NULL)
	{
		std::cout << "\tWARNING -> Output folder does not exist -> try to create it" << std::endl;
		if (system(("mkdir " + output_folder).c_str()))
		{
			std::cout << "\tERROR -> Failed to create directory" << std::endl;
			return 0;
		}
	}
	closedir(dir);

	/// Create the graph based on the detections
	begin = clock(); // Start timer
	TrackingGraph graph = TrackingGraph(dres);

	/// Create an instance of GOG and start the tracking
	GOGSettings settings;
	DPTracking gog = DPTracking(nms, graph, settings);
	gog.startTracking();
	end = clock(); // End timer
	// Print time spent tracking
	double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
	std::cout << "Time spent tracking : " << time_spent << "seconds" << std::endl;

	/// Get the detections and their ids after tracking 
	/// Variable that holds the boxes to draw/write info in file
	std::vector< std::vector<BoundingBox> > bboxes = gog.getBBoxes();

#if !USE_IN_DETRAC
#if SHOW_BOXES
	char filename[255];
	cv::Mat image;

	cv::namedWindow("Display Tracking", cv::WINDOW_AUTOSIZE);
	for (int i = 0; i < bboxes.size(); i++)
	{
		// Load the image
		sprintf_s(filename, "%s/%s", input_folder.c_str(), filelist[i]->d_name);
		image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
		// Write all the boxes into the image	
		std::vector<BoundingBox> frameBoxes = bboxes[i];
		for (int j = 0; j < frameBoxes.size(); j++)
		{
			BoundingBox b = frameBoxes[j];
			cv::rectangle(image, cv::Point(b.x, b.y), cv::Point(b.x + b.w, b.y + b.h), cv::Scalar(0, 0, 255), 2);
			cv::putText(image, std::to_string(j), cv::Point(b.x + b.w - b.w / 2, b.y + b.h - 5), 1, 1, cv::Scalar(0, 255, 255), 2);
		}
#if SAVE_BOXES
		// Save the images
		sprintf_s(filename, "%s/%s", output_folder.c_str(), filelist[i]->d_name);
		cv::imwrite(filename, image);
#endif
		imshow("Display Tracking", image);
		cv::waitKey(50);
	}
	std::cout << "Displaying images finished!!" << std::endl;
#endif // SHOW BOXES

#else
	//std::cout << "Before writing results!" << std::endl;
	UADETRAC::write_results(output_folder, sequence, bboxes);
#endif

	cv::waitKey(0);
	return 0;

}

