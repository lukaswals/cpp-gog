/*
* File containing all the related functions to use the UA-DETRAC dataset
* More info on <http://detrac-db.rit.albany.edu/>
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

namespace UADETRAC
{
	/// Read all the provided detections on a Detections struct
	void read_detections(std::ifstream& file, Detections& detections)
	{

		// If the column number X doesn't exists, it will do nothing
		for (CSVIterator loop(file); loop != CSVIterator(); ++loop)
		{
			detections.frame.push_back(std::stoi((*loop)[0]));
			detections.x.push_back(std::stof((*loop)[2]));
			detections.y.push_back(std::stof((*loop)[3]));
			detections.w.push_back(std::stof((*loop)[4]));
			detections.h.push_back(std::stof((*loop)[5]));
			// UA-DETRAC made a little change on the score
			detections.score.push_back(std::stof((*loop)[6]) * 3 - 1.5);
		}
	}

	/// To output only up to 2 decimals
	std::string to_string_with_precision(float value)
	{
		int decimals = 0;
		if (value != 0)
			decimals = 2;

		std::ostringstream out;
		out << std::fixed << std::setprecision(decimals) << value;
		return out.str();
	}

	/// Store results in UA-DETRAC format
	void write_results(std::string& output_folder, std::string& sequence,
		std::vector< std::vector<BoundingBox> > track)
	{
		char filename[255];
		sprintf_s(filename, "%s/%s_LX.txt", output_folder.c_str(), sequence);
		std::ofstream lx_file(filename);
		sprintf_s(filename, "%s/%s_LY.txt", output_folder.c_str(), sequence);
		std::ofstream ly_file(filename);
		sprintf_s(filename, "%s/%s_W.txt", output_folder.c_str(), sequence);
		std::ofstream w_file(filename);
		sprintf_s(filename, "%s/%s_H.txt", output_folder.c_str(), sequence);
		std::ofstream h_file(filename);

		std::cout << "Creating result files" << std::endl;

		int columns = track.back().size(), amount = 0;
		std::string lx, ly, w, h, aux;
		std::vector<BoundingBox> frame;

		// Get the highest id
		for (int i = 0; i < track.size(); i++)
		{
			frame = track[i];
			for (int j = 0; j < frame.size(); j++)
				if (amount < frame[j].id) amount = frame[j].id;
		}

		for (int i = 0; i < track.size(); i++)
		{
			frame = track[i];
			int j = 0;

			for (int id = 1; id <= amount; id++)
			{
				if (frame[j].id == id && !frame.empty())
				{
					lx.append(to_string_with_precision(frame[j].x) + ",");
					ly.append(to_string_with_precision(frame[j].y) + ",");
					w.append(to_string_with_precision(frame[j].w) + ",");
					h.append(to_string_with_precision(frame[j].h) + ",");
					frame.erase(frame.begin()); // A nice way to unsure not trash value on frame
				}
				else
				{
					lx.append("0,");
					ly.append("0,");
					w.append("0,");
					h.append("0,");
				}
			} // end for boxes on frame i
			// Remove the trailing comma
			lx.pop_back(); ly.pop_back();
			w.pop_back(); h.pop_back();
			// Write to files
			lx_file << lx << std::endl;
			ly_file << ly << std::endl;
			w_file << w << std::endl;
			h_file << h << std::endl;
			// Empty the strings
			lx.clear(); ly.clear();
			w.clear(); h.clear();

		}
		// Close all the files
		lx_file.close();
		ly_file.close();
		w_file.close();
		h_file.close();

	}

} // end of namespace