objrec_DIR = ..
BUILD = $(objrec_DIR)/build
BIN = $(objrec_DIR)/bin
SRC = $(objrec_DIR)/src
OPTIONS = -O3 -Xlinker -zmuldefs -g

all: $(BIN)/learn $(BIN)/detect $(BIN)/visualize_object $(BIN)/show_detections $(BIN)/label_pose $(BIN)/verify_table $(BIN)/label_table  $(BIN)/check_tolerances $(BIN)/yml_to_depth $(BIN)/set_table_height $(BIN)/cam_to_scene $(BIN)/pose_to_bbox #$(BIN)/learn_viewpoint $(BIN)/combine_objects $(BIN)/distribute $(BIN)/test_edge_directions $(BIN)/bresenham_rotation $(BIN)/test_constraints $(BIN)/image_detect 

$(BIN)/learn: $(SRC)/learn.cpp $(SRC)/args.h $(BUILD)/edge_detect.o $(BUILD)/textons.o $(BUILD)/simulation.o $(SRC)/learning.h 
	mkdir -p $(BUILD)
	mpicxx $(OPTIONS) $(SRC)/learn.cpp $(BUILD)/edge_detect.o $(BUILD)/simulation.o $(BUILD)/textons.o -lopencv_core -lopencv_highgui -lopencv_imgproc -lGL -lX11 -o $(BIN)/learn

$(BIN)/detect: $(SRC)/detect.cpp $(SRC)/args.h $(SRC)/constraints.h $(BUILD)/edge_detect.o $(BUILD)/textons.o
	mkdir -p $(BUILD)
	mpicxx $(OPTIONS) $(SRC)/detect.cpp $(BUILD)/edge_detect.o $(BUILD)/textons.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/detect

$(BIN)/learn_viewpoint: $(SRC)/learn_viewpoint.cpp $(SRC)/args.h $(BUILD)/edge_detect.o $(BUILD)/textons.o $(BUILD)/simulation.o $(SRC)/learning.h 
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/learn_viewpoint.cpp $(BUILD)/edge_detect.o $(BUILD)/simulation.o $(BUILD)/textons.o -lopencv_core -lopencv_highgui -lopencv_imgproc -lGL -lX11 -o $(BIN)/learn_viewpoint

$(BIN)/combine_objects: $(SRC)/combine_objects.cpp $(SRC)/representation.h $(BUILD)/textons.o 
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/combine_objects.cpp $(BUILD)/textons.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/combine_objects

$(BIN)/image_detect: $(SRC)/image_detect.cpp $(SRC)/args.h $(SRC)/constraints.h $(BUILD)/edge_detect.o $(BUILD)/textons.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/image_detect.cpp $(BUILD)/edge_detect.o $(BUILD)/textons.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/image_detect

$(BIN)/distribute: $(SRC)/distribute.cpp
	mkdir -p $(BUILD)
	mpicxx $(OPTIONS) $(SRC)/distribute.cpp -o $(BIN)/distribute

$(BIN)/visualize_object: $(SRC)/visualize_object.cpp $(SRC)/constraints.h $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/visualize_object.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/visualize_object

$(BIN)/show_detections: $(SRC)/show_detections.cpp $(SRC)/constraints.h $(SRC)/draw.h $(BUILD)/textons.o $(BUILD)/edge_detect.o $(BUILD)/simulation.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/show_detections.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o $(BUILD)/simulation.o -lopencv_core -lopencv_highgui -lopencv_imgproc -lGL -lX11 -o $(BIN)/show_detections

$(BIN)/label_pose: $(SRC)/label_pose.cpp $(SRC)/constraints.h $(SRC)/draw.h $(BUILD)/textons.o $(BUILD)/edge_detect.o $(SRC)/simulation.h $(BUILD)/simulation.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/label_pose.cpp $(BUILD)/textons.o $(BUILD)/simulation.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -lGL -lX11 -o $(BIN)/label_pose

$(BIN)/label_table:  $(SRC)/label_table.cpp $(SRC)/constraints.h  $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/label_table.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/label_table

$(BIN)/bresenham_rotation:  $(SRC)/bresenham_rotation.cpp $(SRC)/detection.h  $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/bresenham_rotation.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/bresenham_rotation

$(BIN)/background_subtract:  $(SRC)/background_subtract.cpp $(SRC)/detection.h  $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/background_subtract.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/background_subtract

$(BIN)/test_constraints: $(SRC)/test_constraints.cpp $(SRC)/constraints.h $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/test_constraints.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/test_constraints

$(BIN)/set_table_height: $(SRC)/set_table_height.cpp $(SRC)/constraints.h $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/set_table_height.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/set_table_height

$(BIN)/test_edge_directions: $(SRC)/test_edge_directions.cpp $(SRC)/constraints.h $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/test_edge_directions.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/test_edge_directions

$(BIN)/yml_to_depth: $(SRC)/yml_to_depth.cpp $(SRC)/io.h
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/yml_to_depth.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/yml_to_depth

$(BIN)/cam_to_scene: $(SRC)/cam_to_scene.cpp $(SRC)/constraints.h $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/cam_to_scene.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/cam_to_scene

$(BIN)/pose_to_bbox: $(SRC)/pose_to_bbox.cpp $(SRC)/constraints.h $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/pose_to_bbox.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/pose_to_bbox

$(BIN)/verify_table: $(SRC)/verify_table.cpp $(SRC)/constraints.h $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/verify_table.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/verify_table

$(BIN)/check_tolerances: $(SRC)/check_tolerances.cpp $(SRC)/constraints.h $(BUILD)/textons.o $(BUILD)/edge_detect.o
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/check_tolerances.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/check_tolerances


$(BIN)/choose_textons: $(SRC)/choose_textons.cpp $(BUILD)/textons.o $(SRC)/textons.h $(BUILD)/textons.o 
	mkdir -p $(BUILD)
	g++ $(OPTIONS) $(SRC)/choose_textons.cpp $(BUILD)/textons.o $(BUILD)/edge_detect.o -lopencv_core -lopencv_highgui -lopencv_imgproc -o $(BIN)/choose_textons

$(BUILD)/edge_detect.o: edge_detect.cpp edge_detect.h
	mkdir -p $(BUILD)
	g++ $(OPTIONS) -c $(SRC)/edge_detect.cpp -o $(BUILD)/edge_detect.o

$(BUILD)/textons.o: textons.cpp textons.h
	mkdir -p $(BUILD)
	g++ $(OPTIONS) -c $(SRC)/textons.cpp -o $(BUILD)/textons.o

$(BUILD)/simulation.o: simulation.cpp simulation.h
	mkdir -p $(BUILD)
	g++ $(OPTIONS) -c $(SRC)/simulation.cpp -o $(BUILD)/simulation.o -lX11

$(SRC)/image.h: $(SRC)/representation.h
	touch $(SRC)/image.h
$(SRC)/features.h: $(SRC)/edge_detect.h $(SRC)/textons.h
	touch $(SRC)/features.h
$(SRC)/edge_detect.h: $(SRC)/io.h
	touch $(SRC)/edge_detect.h
$(SRC)/representation.h: $(SRC)/features.h $(SRC)/io.h
	touch $(SRC)/representation.h
$(SRC)/detection.h: $(SRC)/image.h
	touch $(SRC)/detection.h
$(SRC)/learning.h: $(SRC)/simulation.h $(SRC)/representation.h
	touch $(SRC)/learning.h
$(SRC)/constraints.h: $(SRC)/detection.h
	touch $(SRC)/constraints.h
$(SRC)/simulation.h:
	touch $(SRC)/simulation.h
$(SRC)/textons.h:
	touch $(SRC)/textons.h
$(SRC)/args.h:
	touch $(SRC)/args.h
$(SRC)/draw.h: $(SRC)/representation.h $(SRC)/simulation.h
	touch $(SRC)/draw.h

clean:
	rm $(BUILD)/* $(BIN)/*
