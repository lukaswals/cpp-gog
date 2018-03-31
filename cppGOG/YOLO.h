/*
* Read detections obtained from YOLO detector
* Author Lucas Wals
*/
#pragma once
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <iomanip>

#include "CSV.h"
#include "GOG.h"

namespace YOLO
{

	/// Read all the provided detections on a Detections struct
	void read_detections(std::ifstream& file, Detections& detections)
	{
		int frame = 0;
		std::string imageName = "";
		// If the column number X doesn't exists, it will do nothing
		for (CSVIterator loop(file); loop != CSVIterator(); ++loop)
		{
			if ((*loop)[0] == "image") continue; // It's the header lines
			if (imageName.compare((*loop)[0]) != 0)
			{
				frame++;
				imageName = (*loop)[0];
			}
			detections.frame.push_back(frame);
			float xmin = std::stof((*loop)[3]);
			float ymin = std::stof((*loop)[4]);
			float xmax = std::stof((*loop)[5]);
			float ymax = std::stof((*loop)[6]);
			detections.x.push_back(xmin);
			detections.y.push_back(ymin);
			detections.w.push_back(xmax - xmin);
			detections.h.push_back(ymax - ymin);
			detections.score.push_back(std::stof((*loop)[2]));
		}
	}

} // end of namespace
