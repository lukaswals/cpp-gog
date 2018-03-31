/******************************************************************************
* C++ Implementation of GOG tracking algorithm
* Author Lucas Wals
******************************************************************************/
#pragma once
#include <vector>
#include <algorithm>

/******************************************************************************
* STRUCT DEFINITIONS
******************************************************************************/
/// Datastructure holding the information of the provided detections
struct Detections
{
	// x-component of top left coordinate
	std::vector<float> x;
	// y-component of top left coordinate
	std::vector<float> y;
	// width of the box section
	std::vector<float> w;
	// height of the box section
	std::vector<float> h;
	// Score of the detection
	std::vector<float> score;
	// frame of the detection
	std::vector<int> frame;
};

/// Datastructure only used for showing the results
struct BoundingBox
{
	// x-component of top left coordinate
	float x;
	// y-component of top left coordinate
	float y;
	// width of the box
	float w;
	// height of the box
	float h;
	// identifies object
	int id;
};

/// Settings structure for the tracker
struct GOGSettings
{
	// Cost of a birth of a track. Matlab name: c_en
	int birthCost;
	// Cost of a death of a track. Matlab name: c_ex
	int deathCost;
	// Transition Cost =D. Matlab name: c_ij
	int transitionCost;
	// Betta value, increasing it will produce less tracks for every single detection
	double betta;
	// Maximum number of tracks. Matlab name: max_it
	float maxIterations;
	// Maximum acceptable cost for a track. Matlab name: thr_cost
	int maxTrackCost;

	/// Default constructor for default configuration
	GOGSettings()
	{
		birthCost = 10; // Original value = 10
		deathCost = 10; // Original value = 10
		transitionCost = -2; // Original value = 0
		betta = 0.01; // Original value = 0.2
		maxIterations = INFINITY; // Original value = inf 
		maxTrackCost = 18; // Original value = 18
	}
};

/******************************************************************************
* COMMON FUNCTIONS DEFINITIONS
******************************************************************************/
/// Returns a pair made of (IoU,IoArea)
inline std::pair<float, float> intersectionOverUnion(
	float minx1, float miny1, float maxx1, float maxy1,
	float minx2, float miny2, float maxx2, float maxy2)
{
	std::pair<float, float> result;
	if (minx1 > maxx2 || maxx1 < minx2 || miny1 > maxy2 || maxy1 < miny2)
	{
		result = std::make_pair(0.0f, 0.0f);
		return result;
	}
	else
	{
		float dx = MIN(maxx2, maxx1) - MAX(minx2, minx1);
		float dy = MIN(maxy2, maxy1) - MAX(miny2, miny1);
		float area1 = (maxx1 - minx1)*(maxy1 - miny1);
		float area2 = (maxx2 - minx2)*(maxy2 - miny2);
		float inter = dx*dy; // Intersection
		float uni = area1 + area2 - inter; // Union
		float IoU = inter / uni;
		float IoA1 = inter / area1;
		result = std::make_pair(IoU, IoA1);
		return result;
	}
}

/******************************************************************************
* CLASSES DEFINITION
******************************************************************************/
/// Graph that models the tracking problem
class TrackingGraph
{
public:
	TrackingGraph(Detections detections);

	void buildGraph();
	Detections getDres() { return dres; };
	std::vector< std::vector<int> > getNei() { return neighbors; };

private:
	// Same name as in the matlab implementation. Stores all the detected boxes
	Detections dres;
	// Threshold for overlapping of detected boxes. Original value = 0.5
	double ovThreshold = 0.3;
	// Same name as in the matlab implementation. Stores the graph edges,
	// the transitions between different detections
	std::vector< std::vector<int> > neighbors;

};

/// The main class. DP stands for Dynamic Programming 
class DPTracking
{
public:

	DPTracking(bool nmsOn, TrackingGraph g, const GOGSettings& settings = GOGSettings())
		: nmsInLoop(nmsOn), graph(g),
		birthCost(settings.birthCost), deathCost(settings.deathCost),
		transitionCost(settings.transitionCost), betta(settings.betta),
		maxIterations(settings.maxIterations), maxTrackCost(settings.maxTrackCost) { };

	void startTracking();
	std::vector<int> agressiveNMS(Detections dres, std::vector<int> inds, double nmsThreshold);
	void setResults(Detections dres, std::vector<int> indices);
	std::vector< std::vector<BoundingBox> > getBBoxes();

private:
	int birthCost;
	int deathCost;
	int transitionCost;
	double betta;
	float maxIterations;
	int maxTrackCost;
	bool nmsInLoop;

	TrackingGraph graph;
	Detections result;
	std::vector<int> ids;
};

/******************************************************************************
* GRAPH IMPLEMENTATION
******************************************************************************/
TrackingGraph::TrackingGraph(Detections detections)
{
	dres = detections;
	buildGraph();
}

/// Here we model the tracking as a Graph based on the detections
void TrackingGraph::buildGraph()
{
	int nDet = dres.frame.size(); // Number of detections
	int frame = 2; // Start from the 2nd frame
	std::vector<int> ovBoxes; // Indices of overlapping boxes with a specific box
	int lastFrame = *std::max_element(dres.frame.begin(), dres.frame.end());
	float IoU, ratio, min_ratio;

	// Iterators used to find the detections of frames
	int pfStart, pfEnd, afStart, afEnd;
	std::pair<std::vector<int>::iterator, std::vector<int>::iterator> pfBounds, afBounds;

	for (frame; frame <= lastFrame; frame++)
	{
		// Indices for detections of the previous frame
		pfBounds = std::equal_range(dres.frame.begin(), dres.frame.end(), frame - 1);
		pfStart = pfBounds.first - dres.frame.begin();
		pfEnd = pfBounds.second - dres.frame.begin();
		// Indices for detections of actual frame
		afBounds = std::equal_range(dres.frame.begin(), dres.frame.end(), frame);
		afStart = afBounds.first - dres.frame.begin();
		afEnd = afBounds.second - dres.frame.begin();
		// If we are looking at the first two frames
		if (frame == 2)
		{
			for (pfStart; pfStart < pfEnd; pfStart++)
			{
				neighbors.push_back(ovBoxes);
			}
		}
		// For all the detections in the actual frame
		for (afStart; afStart < afEnd; afStart++)
		{
			pfStart = pfBounds.first - dres.frame.begin();
			// Calculate the overlap with all the detections in previous frame
			for (pfStart; pfStart < pfEnd; pfStart++)
			{
				// Find all the boxes that are > overlapThreshold
				IoU = intersectionOverUnion(
					dres.x[pfStart], dres.y[pfStart], dres.x[pfStart] + dres.w[pfStart], dres.y[pfStart] + dres.h[pfStart],
					dres.x[afStart], dres.y[afStart], dres.x[afStart] + dres.w[afStart], dres.y[afStart] + dres.h[afStart]
				).first; // First always has the IoU
						 // Ignore transitions with large change in bb size
				ratio = dres.h[afStart] / dres.h[pfStart];
				min_ratio = MIN(ratio, 1 / ratio);
				if (IoU > ovThreshold && min_ratio > 0.8)
				{
					ovBoxes.push_back(pfStart); // Make a list of indices of overlapped boxes
				}
			}
			// Populate "neighbors" with a list of neighbors in the previous frame for that detection
			neighbors.push_back(ovBoxes);
			ovBoxes.clear();
		} // end for actual frame detections
	} // end for frame

}

/******************************************************************************
* TRACKING ALGORITHM IMPLEMENTATION
******************************************************************************/
void DPTracking::startTracking()
{
	// Auxiliary variables
	std::vector<float>::iterator iteF; // Auxiliary iterator
	std::vector<int>::iterator iteI, auxIteI; // Auxiliary iterator
	Detections dres = graph.getDres();

	std::vector< std::vector<int> > nei = graph.getNei();
	std::vector<int> auxInds, auxNodes;
	std::vector<float> auxNeiInds;

	// Actual variables used for tracking
	double nmsThreshold = 0.5;
	int dnum = dres.x.size(); // Will always hold the original amount of detections
	std::vector<float> c, dp_c, dp_link, min_cs; // Same names as Matlab implementation
	std::vector<int> orig, redo_nodes, inds, inds_all, id_s, supp_inds; // Same names as Matlab implementation
	std::vector<int> neiInds; // List of indices of each detection (the vector inside "nei vector") 

	for (iteF = dres.score.begin(); iteF <= dres.score.end(); iteF++)
		c.push_back(betta - *iteF); // betta - score

	float min_c = -INFINITY, min_cost = INFINITY;
	int min_link, it = 0, k = 0, k1 = 0;
	// nodesAmount will always hold the size of "redo_nodes"
	int nodesAmount, index;
	int jIndex; // Hold the index for the min cost path

	// redo_nodes has a list of indices.....
	nodesAmount = dres.x.size();
	for (int i = 0; i < nodesAmount; i++)
		redo_nodes.push_back(i);

	// Initialize the vectors with the same size as redo_nodes at the beginning
	dp_c.resize(nodesAmount);
	dp_link.resize(nodesAmount);
	orig.resize(nodesAmount);

	while ((min_c < maxTrackCost) && (it < maxIterations))
	{
		// Arrays in Matlab start at index 1, that's the reason for this i++ at beginning
		it++;

		nodesAmount = redo_nodes.size();
		for (iteI = redo_nodes.begin(); iteI < redo_nodes.end(); iteI++)
		{
			dp_c[*iteI] = c[*iteI] + birthCost; // not the same value as matlab, because of vector C
			dp_link[*iteI] = 0;
			orig[*iteI] = *iteI;
		}

		// Next for same as "for ii=1:length(redo_nodes)"
		for (int i = 0; i < nodesAmount; i++)
		{
			index = redo_nodes[i];
			neiInds = nei[index];
			if (neiInds.empty()) continue;

			// Reset the min_cost for the new index
			min_cost = INFINITY; // MAYBE NOT NECESSARY SINCE WE ARE NOT USING THIS TO COMPARE
			auxNeiInds.clear();
			// Next section same as "[min_cost j] = min(c_ij + dres.c(i) + dres.dp_c(f2));"
			for (int j = 0; j < neiInds.size(); j++)
				auxNeiInds.push_back(transitionCost + c[index] + dp_c[neiInds[j]]);

			iteF = std::min_element(auxNeiInds.begin(), auxNeiInds.end());
			min_cost = *iteF; // Get the value
			min_link = neiInds[iteF - auxNeiInds.begin()]; // Get the index

			if (dp_c[index] > min_cost)
			{
				dp_c[index] = min_cost;
				dp_link[index] = min_link;
				orig[index] = orig[min_link];
			}
		} // end of for redo_nodes size (nodesAmount variable)

		// Next section same as "[min_c ind] = min(dres.dp_c + c_ex);"
		int ind = 0;
		auxNeiInds.clear();
		for (int j = 0; j < dp_c.size(); j++)
			auxNeiInds.push_back(dp_c[j] + deathCost);

		iteF = std::min_element(auxNeiInds.begin(), auxNeiInds.end());
		min_c = *iteF; // Get the value
		ind = iteF - auxNeiInds.begin(); // Get the index

		// Next for same as "inds = zeros(dnum,1);". Reset the "inds" vector to all zeros
		inds.clear();
		for (int i = 0; i < dnum; i++)
			inds.push_back(0);

		k1 = 0;
		while (ind != 0)
		{
			inds[k1] = ind;
			ind = dp_link[ind];
			k1++;
		}
		inds.resize(k1); // Make the vector shorter

		// Add inds values to inds_all
		inds_all.insert(inds_all.end(), inds.begin(), inds.end());
		int limit = k + inds.size();
		for (k; k < limit; k++) id_s.push_back(it);

		// Use Non-Maximum Suppression
		if (nmsInLoop)
		{
			supp_inds = agressiveNMS(dres, inds, nmsThreshold);
			// Next line same as "unique(dres.orig(supp_inds));" because it always have the same number
			int origs = orig[supp_inds[0]]; // The whole vector has the same number
			// Next for same as "find(ismember(dres.orig, origs));"			
			for (iteI = orig.begin(); iteI < orig.end(); iteI++)
				if (*iteI == origs)	auxInds.push_back(iteI - orig.begin());

			redo_nodes = auxInds;
			auxInds.clear();
		}
		else // Not use of NMS
		{
			supp_inds = inds;
			int origs = inds.back();
			// Next for is same as "redo_nodes = find(dres.orig == origs);"
			for (iteI = orig.begin(); iteI < orig.end(); iteI++)
				if (*iteI == origs) auxInds.push_back(iteI - orig.begin());

			redo_nodes = auxInds;
			auxInds.clear();
		} // end if nms is active

		  // Next for same as "redo_nodes = setdiff(redo_nodes, supp_inds);"
		auxNodes.clear(); // Keep the vector clean for this iteration
		// Look for all the elements in redo_nodes that are not in supp_inds
		for (iteI = redo_nodes.begin(); iteI < redo_nodes.end(); iteI++)
			if (find(supp_inds.begin(), supp_inds.end(), *iteI) == supp_inds.end())
				auxNodes.push_back(*iteI);
		redo_nodes = auxNodes;

		for (iteI = supp_inds.begin(); iteI < supp_inds.end(); iteI++)
		{
			dp_c[*iteI] = INFINITY;
			c[*iteI] = INFINITY;
		}

		min_cs.push_back(min_c);
	} // end of while

	// Return a subset of the detections, same as "sub(dres, inds_all);" 
	setResults(dres, inds_all); // REVIEW THIS FUNCTION
	ids = id_s;
}

/// Take only the "most important" detections, and discard the rest
void DPTracking::setResults(Detections dres, std::vector<int> indices)
{
	std::vector<int>::iterator iteI; // Auxiliary iterator

	for (iteI = indices.begin(); iteI < indices.end(); iteI++)
	{
		result.x.push_back(dres.x[*iteI]);
		result.y.push_back(dres.y[*iteI]);
		result.w.push_back(dres.w[*iteI]);
		result.h.push_back(dres.h[*iteI]);
		result.score.push_back(dres.score[*iteI]);
		result.frame.push_back(dres.frame[*iteI]);
	}
	std::cout << "Result size > " << result.x.size() << std::endl;
}

std::vector<int> DPTracking::agressiveNMS(Detections dres, std::vector<int> inds, double nmsThreshold)
{
	/// Auxiliary variables
	std::vector<int>::iterator iteI; // Integer iterator
	std::vector<int> auxInds;
	std::vector<float> overlaps;
	/// Actual useful variables
	std::vector<int> inds_out, frameDetections;
	int zeros = dres.x.size();
	int f1, k = 0, frame;
	std::pair<float, float> ovResult;
	std::vector<int> f2;

	// "inds_out" starts with all zeros
	for (int i = 0; i < zeros; i++)
		inds_out.push_back(0);

	int indsLength = inds.size();
	for (int i = 0; i < indsLength; i++)
	{
		f1 = inds[i];
		frame = dres.frame[f1];
		frameDetections.clear();
		// Next for same as "f2 = find(dres.fr == dres.fr(f1));"
		// Indices of all detections of the same frame
		for (int j = 0; j < dres.frame.size(); j++)
			if (dres.frame[j] == frame) frameDetections.push_back(j);

		for (int l = 0; l < frameDetections.size(); l++)
		{
			ovResult = intersectionOverUnion(
				dres.x[f1], dres.y[f1], dres.x[f1] + dres.w[f1], dres.y[f1] + dres.h[f1],
				dres.x[frameDetections[l]], dres.y[frameDetections[l]],
				dres.x[frameDetections[l]] + dres.w[frameDetections[l]], dres.y[frameDetections[l]] + dres.h[frameDetections[l]]
			);
			if ((ovResult.first > nmsThreshold) && (ovResult.second > 0.9))
			{
				inds_out[k] = frameDetections[l];
				k++;
			}
		} // end of for detections of frame
	}
	// THE SIZE OF THIS inds_out MUST MATCH THE SIZE OF inds
	inds_out.resize(k);
	return inds_out;
}

std::vector< std::vector<BoundingBox> > DPTracking::getBBoxes()
{
	std::vector< std::vector<BoundingBox> > seqBoxes;
	std::vector<BoundingBox> frameBoxes;
	// Amount of frames
	int lastFrame = *std::max_element(result.frame.begin(), result.frame.end());
	// Amount of boxes
	int lastBox = result.x.size();

	for (int i = 0; i < lastFrame; i++)
		seqBoxes.push_back(frameBoxes);

	for (int i = 0; i < lastBox; i++)
	{
		BoundingBox b = { result.x[i], result.y[i], result.w[i], result.h[i], ids[i] };
		seqBoxes[result.frame[i]].push_back(b);
	}

	return seqBoxes;
}
