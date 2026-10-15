CXX      = clang++
CXXFLAGS = -std=c++17 -Wall -DGL_SILENCE_DEPRECATION

all: rot_trans_kinematics drone_model physics_UAV

rot_trans_kinematics: rot_trans_kinematics.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

drone_model: drone_model.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< \
		-framework OpenGL -framework GLUT

physics_UAV: physics_UAV.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

run: rot_trans_kinematics
	./rot_trans_kinematics

view: drone_model
	./drone_model

sim: physics_UAV
	./physics_UAV

clean:
	rm -f rot_trans_kinematics drone_model physics_UAV
