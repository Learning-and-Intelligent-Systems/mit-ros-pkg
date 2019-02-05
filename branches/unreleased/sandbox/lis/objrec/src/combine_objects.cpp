/*
 * combine_objects.cpp
 *
 *  Created on: May 24, 2012
 */

#include "representation.h"
#include <iostream>
#include <vector>
#include <string>
#include <set>

namespace objrec {

view combine_views(const view& v1, const view& v2) {
	std::vector<std::vector<visual_part> > visual_parts = v1.get_visual_parts();
	const std::vector<std::vector<visual_part> >& visual_parts2 = v2.get_visual_parts();
	for(int j=0;j<visual_parts2.size();j++) {
		visual_parts[j].insert(visual_parts[j].end(),visual_parts2[j].begin(),visual_parts2[j].end());
	}
	std::vector<depth_part> depth_parts(v1.get_depth_parts());
	const std::vector<depth_part> &depth_parts2 = v2.get_depth_parts();
	depth_parts.insert(depth_parts.end(),depth_parts2.begin(),depth_parts2.end());
//	point p(0.0f,0.0f,1.0f);
//	float x01, x11, y01, y11;
//	float x02, x12, y02, y12;
//	v1.get_bounding_box(p,x01,x11,y01,y11);
//	v2.get_bounding_box(p,x02,x12,y02,y12);
	return view(v2.get_parent_object(), v2.get_viewpoint(), visual_parts, depth_parts);
//			std::min(x01,x02), std::max(x11,x12), std::min(y01,y02), std::max(y11,y12));
}

view relabel_visual_part_indexes(const view& v, const object* parent_object, const std::vector<unsigned int> relabel_indexes) {
	unsigned int max_index = 0;
	for(int j=0;j<relabel_indexes.size();j++) {
		max_index = std::max(max_index,relabel_indexes[j]);
	}
	std::vector<std::vector<visual_part> > relabeled_visual_parts(max_index);
	const std::vector<std::vector<visual_part> >& visual_parts = v.get_visual_parts();
	for(int j=0;j<visual_parts.size();j++) {
		relabeled_visual_parts[relabel_indexes[j]] = visual_parts[j];
	}
	std::vector<depth_part> depth_parts(v.get_depth_parts());
//	float x0, x1, y0, y1;
//	point p(0.0f,0.0f,1.0f);
//	v.get_bounding_box(p,x0,x1,y0,y1);
	return view(parent_object,v.get_viewpoint(),relabeled_visual_parts,v.get_depth_parts());
//	x0,x1,y0,y1);
}

bool combinable_objects(const object& obj1, const object& obj2) {
	return obj1.library.get_edges().compatible_with(obj2.library.get_edges()) &&
			(obj1.library.get_textons()==obj2.library.get_textons() ||
					obj1.library.get_textons().n_clusters()==0 ||
					obj2.library.get_textons().n_clusters()==0);
}


bool operator<(const view& v1, const view& v2) {
	const viewpoint vp1 = v1.get_viewpoint();
	const viewpoint vp2 = v2.get_viewpoint();
	return
			vp1.min_depth < vp2.min_depth || (vp1.min_depth == vp2.min_depth &&
					(vp1.max_depth < vp2.max_depth || (vp1.max_depth == vp2.max_depth &&
							(vp1.min_rx < vp2.min_rx || (vp1.min_rx == vp2.min_rx &&
									(vp1.max_rx < vp2.max_rx || (vp1.max_rx == vp2.max_rx &&
											(vp1.min_ry < vp2.min_ry || (vp1.min_ry == vp2.min_ry &&
													(vp1.max_ry < vp2.max_ry || (vp1.max_ry == vp2.max_ry &&
															(vp1.min_rz < vp2.min_rz || (vp1.min_rz == vp2.min_rz &&
																	(vp1.max_rz < vp2.max_rz))))))))))))));
}

/////combines obj1 into obj2 (modifies obj2)
void combine_objects(const object& obj1, const object& obj2, std::set<view>& views) {
	if(!combinable_objects(obj1,obj2)) {
		throw std::runtime_error("combine_objects: The visual word dictionaries of the objects to combine do not match.");
	}

//	if(obj1.name!=obj2.name) {
//		obj2.name = obj2.name + "_" + obj1.name;
//	}

	const std::vector<edge_direction>& ed1 = obj1.library.get_edges().get_edge_directions();
	const std::vector<edge_direction>& ed2 = obj2.library.get_edges().get_edge_directions();

	bool same_visuals = true;

	if(ed1.size()==ed2.size()) {
		for(int j=0;j<ed1.size();j++) {
			if(!(ed1[j]==ed2[j])) {
				same_visuals = false;
				break;
			}
		}
		if(!(obj1.library.get_textons()==obj2.library.get_textons())) {
			same_visuals = false;
		}
	} else {
		same_visuals = false;
	}

	if(same_visuals) {
		//add all of the views from object 1, combining any with identical viewpoints
		for(std::vector<view>::const_iterator v1=obj1.views.begin();v1!=obj1.views.end();v1++) {
			std::set<view>::iterator v2 = views.find(*v1);
			if(v2==views.end()) { //if it isn't in the set yet, then
				views.insert(*v1); //insert it into the set
			} else {
				view combined = combine_views(*v1,*v2);
				views.erase(v2);
				views.insert(combined);
			}
		}
		return;
	} else {
		throw std::runtime_error("Code for combining views with different visual words not implemented yet.");
	}
//
//	std::set<edge_direction> all_ed_set;
//	for(std::vector<edge_direction>::const_iterator j=ed1.begin();j!=ed1.end();j++) {
//		all_ed_set.insert(*j);
//	}
//	for(std::vector<edge_direction>::const_iterator j=ed2.begin();j!=ed2.end();j++) {
//		all_ed_set.insert(*j);
//	}
//	std::vector<edge_direction> all_ed(all_ed_set.begin(),all_ed_set.end());
//	std::vector<unsigned int> relabel_indexes1(ed1.size());
//	std::vector<unsigned int> relabel_indexes2(ed2.size());
//	for(int j=0;j<all_ed.size();j++) {
//		for(int k=0;k<ed1.size();k++) {
//			if(all_ed[j]==ed1[k]) {
//				relabel_indexes1[k] = j;
//				break;
//			}
//		}
//		for(int k=0;k<ed2.size();k++) {
//			if(all_ed[j]==ed2[k]) {
//				relabel_indexes2[k] = j;
//				break;
//			}
//		}
//	}
//	edge_detection_parameters ce(obj2.visuals.get_edges(),all_ed);
//	textons ct;
//	if(obj2.visuals.get_textons().n_clusters()==0) {
//		ct = obj1.visuals.get_textons();
//	} else {
//		ct = obj2.visuals.get_textons();
//	}
//	obj2.visuals = visual_dictionary(ce,ct);
//
//
//	std::vector<view> views;
//	//first add all of the views from object 1, combining any with identical viewpoints
//	for(std::vector<view>::const_iterator v1=obj1.views.begin();v1!=obj1.views.end();v1++) {
//		bool found_match = false;
//		for(std::vector<view>::const_iterator v2=obj2.views.begin();v2!=obj2.views.end();v2++) {
//			if(v1->get_viewpoint()==v2->get_viewpoint()) {
//				view rv1 = relabel_visual_part_indexes(*v1,&obj2,relabel_indexes1); //parts from obj1 will go to obj2, so obj1 is the parent
//				view rv2 = relabel_visual_part_indexes(*v2,&obj2,relabel_indexes2);
//				views.push_back(combine_views(rv1,rv2));
//				found_match = true;
//				break;
//			}
//		}
//		if(!found_match) {
//			view rv1 = relabel_visual_part_indexes(*v1,&obj2,relabel_indexes1); //parts from obj1 will go to obj2, so obj1 is the parent
//			views.push_back(rv1);
//		}
//	}
//	//then add all of the views from object 2, skipping those that were already combined in the previous loops
//	for(std::vector<view>::const_iterator v2=obj2.views.begin();v2!=obj2.views.end();v2++) {
//		bool found_match = false;
//		for(std::vector<view>::const_iterator v1=obj1.views.begin();v1!=obj1.views.end();v1++) {
//			if(v1->get_viewpoint()==v2->get_viewpoint()) {
//				found_match = true;
//				break;
//			}
//		}
//		if(!found_match) {
//			view rv2 = relabel_visual_part_indexes(*v2,&obj2,relabel_indexes2);
//			views.push_back(rv2);
//		}
//	}
//
//	obj2.views = views;
//
//
}
} //namespace objrec

int main(int argc, char** argv) {
	if(argc>1 && (std::string(argv[1])==std::string("-h") || std::string(argv[1])==std::string("--help"))) {
		std::cerr << "usage:" << std::endl;
		std::cerr << std::endl;
		std::cerr << " " << std::string(argv[0]) << std::endl;
		std::cerr << "  Reads a sequence of objects from stdin and prints the combined object on stdout." << std::endl;
		std::cerr << std::endl;
		std::cerr << " " << std::string(argv[0]) << " outfile" << std::endl;
		std::cerr << "  Reads a sequence of objects from stdin and writes them to outfile." << std::endl;
		std::cerr << std::endl;
		std::cerr << " " << std::string(argv[0]) << " infile1 infile2 infile3 ... outfile" << std::endl;
		std::cerr << "  Reads an object from each infile* and writes a combined object outfile. Overwrites outfile if it already exists." << std::endl;
		return -1;
	}
	objrec::object obj;
	std::set<objrec::view> views;
	if(argc<=2) {
		//first read cin into a string so that we can call tellg/seekg on it
		std::string str((std::istreambuf_iterator<char>(std::cin)),
		                 std::istreambuf_iterator<char>());
		std::istringstream in(str);
		in >> obj;
		std::copy(obj.views.begin(),obj.views.end(),std::inserter(views,views.end()));
		int j=1;
		while(in.good()) {
			objrec::object objj;
			in >> objj;
			objrec::combine_objects(objj,obj,views);
			j++;
		}

		obj.views.resize(views.size());
		std::copy(views.begin(),views.end(),obj.views.begin());

		if(argc==1) {
			std::cout << obj;
			std::cerr << "Combined " << j << " objects." << std::endl;
			return 0;
		}
		//else if(argc==2) {} //output to file below
	} else {
		int argj = 1;
		std::ifstream in(argv[argj]);
		if(!in) throw std::runtime_error(std::string("Could not read file ") + argv[argj]);
		in >> obj;
		std::copy(obj.views.begin(),obj.views.end(),std::inserter(views,views.end()));
		while(in.good()) {
			objrec::object objj;
			in >> objj;
			objrec::combine_objects(objj,obj,views);
		}

		for(argj=2;argj<argc-1;argj++) {
			std::ifstream in(argv[argj]);
			if(!in) throw std::runtime_error(std::string("Could not read file ") + argv[argj]);
			while(in.good()) {
				objrec::object objj;
				in >> objj;
				objrec::combine_objects(objj,obj,views);
			}
		}

		obj.views.resize(views.size());
		std::copy(views.begin(),views.end(),obj.views.begin());

	}
	std::ofstream out(argv[argc-1]);
	if(!out) throw std::runtime_error(std::string("Could not write to file ") + argv[argc-1]);
	out << obj;
	std::cerr << "Wrote object '" << obj.name << "' to file '" << argv[argc-1] << "' with " <<
			obj.views.size() << " views." << std::endl;
}
