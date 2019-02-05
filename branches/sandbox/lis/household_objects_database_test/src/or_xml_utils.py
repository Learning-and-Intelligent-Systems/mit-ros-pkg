def xml_tag_start(content):
    return "<"+content+">"

def xml_tag_end(content):
    return "</"+content+">"

def begin_kinbody_with_name(name):
    return xml_tag_start("KinBody name=\""+name+"\"")

def begin_kinbody_with_filename(filename):
    return xml_tag_start("KinBody file=\""+filename+"\"")

def kinbody_with_file(filename):
    return "<KinBody file=\""+filename+"\"/>"

def end_kinbody():
    return xml_tag_end("KinBody")

def begin_body_with_type(type):
    return xml_tag_start("Body type=\""+type+"\"")

def end_body():
    return xml_tag_end("Body")

def body_translation(x,y,z):
    return xml_tag_start("Translation")+" "+str(x)+" "+str(y)+" "+str(z)+" "+xml_tag_end("Translation")

def body_rotation(a,b,c,d,e,f,g,h,i):
    return xml_tag_start("RotationMat")+" "+str(a)+" "+str(b)\
+" "+str(c)+" "+str(d)+" "+str(e)+" "+str(f)+" "+str(g)\
+" "+str(h)+" "+str(i)+" "+xml_tag_end("RotationMat")

def begin_geom_with_type(type):
    return xml_tag_start("Geom type=\""+type+"\"")

def end_geom():
    return xml_tag_end("Geom")

def begin_data():
    return xml_tag_start("Data")

def end_data():
    return xml_tag_end("Data")

def begin_render():
    return xml_tag_start("Render")

def end_render():
    return xml_tag_end("Render")

def begin_vertices():
    return xml_tag_start("Vertices")

def end_vertices():
    return xml_tag_end("Vertices")

def box_extents(x,y,z):
    return xml_tag_start("extents")+" "+str(x)+" "+str(y)+" "+str(z)+" "+xml_tag_end("extents")

def create_box(name,translation,rotation,extents):
    lines=begin_kinbody_with_name(name)+"\n"
    lines+=begin_body_with_type("dynamic")+"\n"
    lines+=body_translation(translation[0],translation[1],translation[2])+"\n"
    lines+=body_rotation(rotation[0],rotation[1],rotation[2],rotation[3],rotation[4],rotation[5],rotation[6],rotation[7],rotation[8])+"\n"
    lines+=begin_geom_with_type("box")+"\n"
    lines+=box_extents(extents[0],extents[1],extents[2])+"\n"
    lines+=end_geom()+"\n"
    lines+=end_body()+"\n"
    lines+=end_kinbody()
    return lines

def create_trimesh(name, translation, rotation, iv_file, scale):
    lines = begin_kinbody_with_name(name) + '\n'
    lines += begin_body_with_type('dynamic') + '\n'
    lines += body_translation(translation[0], translation[1], translation[2])+\
        '\n'
    lines += body_rotation(rotation[0], rotation[1], rotation[2], rotation[3],\
                               rotation[4], rotation[5], rotation[6], \
                               rotation[7], rotation[8]) + '\n'
    lines += begin_geom_with_type('trimesh') + '\n'
    lines += begin_data() + iv_file + end_data() + '\n'
    print scale
    lines += begin_render() + iv_file +' '+scale+' '+ end_render() + '\n'
    lines += end_geom() + '\n'
    lines += end_body() + '\n'
    lines += end_kinbody() 
    return lines

def create_trimesh(name, translation, rotation, vertices):
    lines = begin_kinbody_with_name(name) + '\n'
    lines += begin_body_with_type('dynamic') + '\n'
    lines += body_translation(translation[0], translation[1], translation[2])+\
        '\n'
    lines += body_rotation(rotation[0], rotation[1], rotation[2], rotation[3],\
                               rotation[4], rotation[5], rotation[6], \
                               rotation[7], rotation[8]) + '\n'
    lines += begin_geom_with_type('trimesh') + '\n'
    lines += begin_vertices() + vertices + end_vertices() + '\n'
    lines += end_geom() + '\n'
    lines += end_body() + '\n'
    lines += end_kinbody() 
    return lines

if __name__ =='__main__':
    translation=[.6,0,.03]
    rotation=[1,0,0,0,0,1,0,-1,0]
    extents=[.1,.1,.1]
    filename="testbody.xml"
    outputfile=open(filename,"w")
    lines=create_box ("aoi",translation,rotation,extents)
    outputfile.writelines(lines)
    outputfile.close()
    
    
