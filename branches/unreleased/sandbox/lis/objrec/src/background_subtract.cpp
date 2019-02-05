/*
 * background_subtract.cpp
 *
 *  Created on: May 8, 2014
 */
#include "detection.h"

int main(int argc, char** argv) {
	std::ofstream out("/home/sdavies/data/background_subtract/index.html");
	for(int j=1;j<=330;j+=10) {
		std::cout << j << std::endl;
		std::ostringstream image_filename;
		image_filename << "/home/sdavies/data/background_subtract/" << std::setfill('0') << std::setw(4) << j << ".png";
		cv::Mat_<cv::Vec3b> picture = objrec::read_image(image_filename.str());
		cv::Mat_<float> depth = objrec::read_depth(objrec::corresponding_depth_file(image_filename.str()));
		std::ostringstream depth_filename;
		depth_filename << "/home/sdavies/data/background_subtract/" << std::setfill('0') << std::setw(4) << j << "_depth.png";
//		cv::imwrite(depth_filename.str(),objrec::mat_bgr(depth));
		std::ostringstream edges_high_filename;
		edges_high_filename << "/home/sdavies/data/background_subtract/" << std::setfill('0') << std::setw(4) << j << "_edges_high.png";
		objrec::edge_detection_parameters high(8,0,2600,2600);
		cv::imwrite(edges_high_filename.str(), objrec::edges_bgr(picture, depth, high));
		std::ostringstream edges_low_filename;
		edges_low_filename << "/home/sdavies/data/background_subtract/" << std::setfill('0') << std::setw(4) << j << "_edges_low.png";
		objrec::edge_detection_parameters low(8,0,200,200);
		cv::imwrite(edges_low_filename.str(), objrec::edges_bgr(picture, depth, low));

		out << "<h2>" << std::setfill('0') << std::setw(4) << j << "</h2>" << std::endl;
		out << "<img src='" << image_filename.str() << "'/><img src='" << depth_filename.str() << "'/>" <<
				"<img src='" << edges_high_filename.str() << "'/><img src='" << edges_low_filename.str() << "'/><br/>" << std::endl;
	}
}
