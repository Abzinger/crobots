GUROBI=/opt/gurobi/linux64/include/
LINK_GUROBI=-L/opt/gurobi/linux64/lib/ -lgurobi_c++ -lgurobi65

DOTS_CODE=../../../../../../Work/Code/
#LINK_DOTS_CODE=-L../../../../../../Work/Code/DOTs_Code.a

CCC=g++
CCCDEBUGFLAGS= -O
#CCCDEBUGFLAGS= -g -O0 -fno-inline -fno-eliminate-unused-debug-types
CCCFLAGS=-Wall -std=c++11 -I $(DOTS_CODE) -I $(GUROBI) $(CCCDEBUGFLAGS)

SOURCES = CNF.cc grid.cc example_grids.cc grid_stat.cc grid_properties.cc robroute.cc grid_gurobi.cc gurobi_robot_router.cc visualize_robroute.cc dump_robroute.cc robroute2json.cc generate_robroute.cc
OBJECTS = $(SOURCES:.cc=.o)
EXECS   = gurobi_robot_router visualize_robroute dump_robroute robroute2json generate_robroute

all: $(OBJECTS) $(EXECS) # all_transitions

# lib: $(LIBRARY)
#
# obj: $(OBJECTS)
#
# pslistings: $(PSLISTINGS)

clean:
	rm $(OBJECTS) # $(PSLISTINGS) $(LIBRARY)

depend:
	makedepend -o.o -f Makefile.depend -- $(CCCFLAGS) -- $(SOURCES)


all_transitions: all_transitions.cc
	$(CCC) $(CCCFLAGS) -E all_transitions.cc -o all_transitions_out.cc

gurobi_robot_router: $(OBJECTS)
	$(CCC) $(CCCFLAGS) gurobi_robot_router.o grid_gurobi.o robroute.o grid_stat.o grid.o $(LINK_GUROBI) -o gurobi_robot_router

visualize_robroute: $(OBJECTS)
	$(CCC) $(CCCFLAGS) visualize_robroute.o grid_gurobi.o robroute.o grid_stat.o grid.o $(LINK_GUROBI) -o visualize_robroute

dump_robroute: $(OBJECTS)
	$(CCC) $(CCCFLAGS) dump_robroute.o grid_gurobi.o robroute.o grid_stat.o grid.o $(LINK_GUROBI) -o dump_robroute

robroute2json: $(OBJECTS)
	$(CCC) $(CCCFLAGS) robroute2json.o robroute.o grid_stat.o grid.o $(LINK_GUROBI) -o robroute2json

generate_robroute: $(OBJECTS)
	$(CCC) $(CCCFLAGS) generate_robroute.o robroute.o grid_stat.o grid_properties.o grid.o example_grids.o $(LINK_GUROBI) -o generate_robroute



%.o: %.cc
	$(CCC) $(CCCFLAGS) -c $<

# make_gurobi: make_gurobi.o
# 	$(CCC) $(CCCFLAGS) -o make_gurobi make_gurobi.o -L /opt/gurobi605/linux64/lib -l gurobi_c++ -l gurobi60

# %.cc.ps: %.cc
# 	c++2ps $< ../../tmp/tex/ defs.common
#
# %.hh.ps: %.hh
# 	c++2ps $< ../../tmp/tex/ defs.common
#
# %.tcc.ps: %.tcc
# 	c++2ps $< ../../tmp/tex/ defs.common
#
# %.a: $(OBJECTS)
# 	ar -r $@ $(OBJECTS)

include Makefile.depend
# DO NOT DELETE
