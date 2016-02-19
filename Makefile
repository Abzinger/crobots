GUROBI=/opt/gurobi/linux64/include/
LINK_GUROBI=-L/opt/gurobi/linux64/lib/ -lgurobi_c++ -lgurobi65

CCC=g++
# CCCDEBUGFLAGS= -g -O0 -fno-inline -fno-eliminate-unused-debug-types
CCCDEBUGFLAGS= -O
CCCFLAGS=-Wall -std=c++11 -I $(GUROBI) $(CCCDEBUGFLAGS)

HEADERS = grid.hh
SOURCES = grid.cc grid_stat.cc grid_gurobi.cc gurobi_robot_router.cc visualize_solution.cc
OBJECTS = $(SOURCES:.cc=.o)
EXECS   = gurobi_robot_router visualize_solution

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
	$(CCC) $(CCCFLAGS) gurobi_robot_router.o grid_gurobi.o grid_stat.o grid.o $(LINK_GUROBI) -o gurobi_robot_router

visualize_solution: $(OBJECTS)
	$(CCC) $(CCCFLAGS) visualize_solution.o grid_gurobi.o grid_stat.o grid.o $(LINK_GUROBI) -o visualize_solution



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
