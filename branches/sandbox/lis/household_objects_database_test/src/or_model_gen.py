#!/usr/bin/env python
# utility functions for generating openrave kinbody xml files from ros geometry messages

from or_xml_utils import *


def generate_wg_model_xml(model_id, vertices,filename=None):
    if filename == None:
        filename = str(model_id)+'.kinbody.xml'
    output_file = file(filename,'w')
    lines = create_wg_model(model_id, filename, vertices)
    output_file.write(lines)
    output_file.close()

def create_wg_model(model_id, filename, vertices):
    trans = [0,0,0]
    rot = [1,0,0,0,1,0,0,0,1]
    vertices_str = reduce(lambda x,y: str(x)+' '+str(y), reduce(lambda x,y: x+y, vertices))
    return create_trimesh('wg-model-'+str(model_id),trans,rot,vertices_str)
