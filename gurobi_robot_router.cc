// gurobi_robot_router.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis

#include "grid_gurobi.hh"
#include "example_grids.hh"
#include "robroute.hh"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "gurobi_c++.h"

typedef GridSpace::Grid                      Grid;
typedef GridSpace::Grid_Gurobi               GridGRB;

int main(int argc, const char *argv[]) try
{
    std::srand(std::time(0));

    std::cout<<"Gurobi Robot Router\nPart of the Robot Routing project\n"<<std::endl;
    if (argc!=3) {
        std::cerr<<"USAGE: gurobi_robot_router filename tmax\nwhere filename is the name of a instance file in robroute format.\n";
        return 1;
    }

    const char * const filename    = argv[1];
    const int          t_max       = std::atoi(argv[2]);

    if (t_max < 1) return std::cerr<<"t_max must be at least 1\n", 2;


    std::ifstream file {filename};
    file.exceptions(file.exceptions() | std::ios_base::badbit | std::ios_base::failbit);
    std::string                                comments;
    std::vector< GridSpace::Stat_Vector_t >    init_and_term;
    GridSpace::Grid * p_G = GridSpace::read_robroute(file, &init_and_term, &comments);

    const GridSpace::Grid & G = *p_G;

    const GridSpace::Stat_Vector_t gState_0     = *init_and_term.begin();
    const GridSpace::Stat_Vector_t & gState_t_max = *(init_and_term.end()-1);
    std::cout<<"Initial state:\n"
             <<GridSpace::print(gState_0);
    std::cout<<"Terminal state:\n"
             <<GridSpace::print(gState_t_max);

    std::cout<<"Now creating GuRoBi object w/ t_max="<<t_max<<" ...."<<std::flush;
    GridGRB gGRB {G, (unsigned)t_max};

    std::cout<<
        "done\n"
        "Setting initial grid state...."<<std::flush;
    gGRB.set_initial_state( gState_0 );

    std::cout<<
        "done\n"
        "Setting terminal grid state...."<<std::flush;
    {
        constexpr bool ignore_robots = true;
        constexpr bool ignore_C0     = true;
        constexpr bool hardwire      = false;
        gGRB.set_terminal_state( gState_t_max, ignore_robots, ignore_C0, hardwire);
    }
    std::cout<<"done\n"
             <<"Setting parameters\n";
    //gGRB.set_parameter("SolutionLimit"    ,  2000000000);
    gGRB.set_parameter("Presolve"         ,  2);
    //gGRB.set_parameter("Cuts"             ,  -1);
    gGRB.set_parameter("TimeLimit"        ,  2000.);
    gGRB.set_parameter("Heuristics"       ,  .75);
    gGRB.set_parameter("ImproveStartNodes",  1024);
    gGRB.set_parameter("ImproveStartTime" ,  1000.);
    gGRB.set_parameter("MIPFocus"         ,  1);
    gGRB.set_parameter("Glonck"           ,  93);

    std::cout<<"Starting optimization. Enjoy."<<std::endl;
    gGRB.optimize();

    std::cout<<
        "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
        "Optimization of Grid_Gurobi object has terminated.\n"
        "Let's see if we can get a solution..."<<std::flush;

    std::vector< GridSpace::Stat_Vector_t > fullsol = gGRB.get_solution();
    if ( fullsol.empty() ) {
        std::cout<<"\nLooks like there's no solution, sorry."<<std::endl;
    } else {
        std::cout<<"\nOK, solution has been found."<<std::endl;
        const char filename[] = "solution.robroute";
        std::cout<<"I'll now write everything to the file ``"<<filename<<"'':\n";

        std::cout<<"Opening file..."<<std::flush;
        std::ofstream file {filename};
        file.exceptions(file.exceptions() | std::ios_base::badbit);
        std::cout<<"done.\nWriting to file..."<<std::flush;
        GridSpace::write_robroute(file,fullsol);
        std::cout<<"done.\nClosing..."<<std::flush;
        file.close();
        std::cout<<"done.\nTerminating."<<std::endl;

        std::cout<<
            "\nGurboi Robot Router was brought to you by\n"
            "Algorithms & Theory @ Uni Tartu\n"
            "http://ac.cs.ut.ee\n";
    } // if / else (has solution)
    std::cout<<"Bye-bye."<<std::endl;
} // try/main
catch(const std::exception &stde) {
    std::cout<<std::endl;
    std::cerr<<"The following std::exception was caught by main():\n";
    std::cerr<<stde.what();
    std::cerr<<std::endl;
}
catch(GRBException &grbe) {
    std::cout<<std::endl;
    std::cerr<<"The following GRBException was caught by main():\n";
    std::cerr<<grbe.getMessage();
    std::cerr<<std::endl;
}
catch(const std::string &stre) {
    std::cout<<std::endl;
    std::cerr<<"The following std::string was caught as an exception by main():\n";
    std::cerr<<stre;
    std::cerr<<std::endl;
}
catch(const char *cstr) {
    std::cout<<std::endl;
    std::cerr<<"The following const-char[] exception was caught as an exception by main():\n";
    std::cerr<<cstr;
    std::cerr<<std::endl;
}
catch(...) {
    std::cout<<std::endl;
    std::cerr<<"Unknown exception caught by main()\n";
}

// main()

// EOF gurobi_robot_router.cc
