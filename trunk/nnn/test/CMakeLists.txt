# ---[ Unit tests


rosbuild_download_test_data(http://projects.csail.mit.edu/pr2/testlogs.tgz 
                      test_data/testlogs.tgz ea3977d311c97f60d8b1ebe26637ddea)
rosbuild_untar_file(test_data/testlogs.tgz test_data/stuff tests)
file(GLOB fileglobs  ${PROJECT_SOURCE_DIR}/test_data/test_logs/*.pcd)

set(firstiter TRUE)
foreach(f ${fileglobs})
	if(firstiter)
		set(filelist \"${f}\")
		set(firstiter FALSE)
	else(firstiter)
		set(filelist ${filelist},\"${f}\")
	endif(firstiter)
endforeach(f ${fileglobs})
list(LENGTH fileglobs numfiles)


#message(STATUS " filelist:   ${filelist}")
configure_file( ${PROJECT_SOURCE_DIR}/test/testpathconfig.h ${PROJECT_SOURCE_DIR}/test/logs_path.h)


#rosbuild_add_executable (test_nnn test_nnn.cpp)
rosbuild_add_gtest (test_nnn test_nnn.cpp)
#target_link_libraries (test_nnn nnn)

