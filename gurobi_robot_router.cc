// gurobi_robot_router.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis

#include "grid_gurobi.hh"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "gurobi_c++.h"

typedef GridSpace::Full_Rectangle__Grid      FullGrid;
typedef GridSpace::Random__Grid              RndGrid;
typedef GridSpace::Random__Grid_Stat         Rnd_Grid_Stat;
typedef GridSpace::Grid_Gurobi               GridGRB;

//    std::cout<<m3a.print();

int main(int argc, const char *argv[]) try
{
    std::cout<<"Gurobi Robot Router\nPart of the Robot Routing project\n"<<std::endl;
    if (argc!=7) {
        std::cerr<<"USAGE: gurobi_robot_router NS EW r c0 c1 c2\n";
        return 1;
    }
    {// check args
        const int NS        = std::atoi(argv[1]);
        const int EW        = std::atoi(argv[2]);
        const int n_robots  = std::atoi(argv[3]);
        const int n_C0      = std::atoi(argv[4]);
        const int n_C1      = std::atoi(argv[5]);
        const int n_C2      = std::atoi(argv[5]);
        if (NS<1 || NS>255)   return std::cerr<<"NS must be in {1,...,255}\n",2;
        if (EW<1 || EW>255)   return std::cerr<<"EW must be in {1,...,255}\n",2;
        if (n_robots<1 || n_robots>=NS*EW)   return std::cerr<<"r must be in {1,...,NS*EW-1}\n",2;
        if (n_C0<0 || n_C0>=NS*EW) return std::cerr<<"c0 must be in {1,...,NS*EW-1}\n",2;
        if (n_C1<0 || n_C1>=NS*EW) return std::cerr<<"c1 must be in {1,...,NS*EW-1}\n",2;
        if (n_C2<0 || n_C2>=NS*EW) return std::cerr<<"c2 must be in {1,...,NS*EW-1}\n",2;
        if (n_C0+n_C1+n_C2>=NS*EW) return std::cerr<<"c0+c1+c2 must be less than NS*EW-1\n",2;
    }

    std::srand(std::time(0));


    const unsigned short NS  = std::atoi(argv[1]);
    const unsigned short EW  = std::atoi(argv[2]);
    FullGrid G(NS,EW);
    std::printf("Full grid of size %dx%d created\n",G.NS_sz(),G.EW_sz());

    // const double p = .8;
    // RndGrid G(NS,EW,p);
    // std::printf("Random grid of size %dx%d created; number of parking slots is %d\n",G.NS_sz(),G.EW_sz(),G.numo_slots());


    // GridSpace::Marsi3_A G;
    // std::printf("Taking Marsi3_A grid. Size: %dx%d.\n",G.NS_sz(),G.EW_sz());

    const unsigned g_size    = G.numo_slots();
    const unsigned n_robots  = std::atoi(argv[3]);
    const unsigned n_C0      = std::atoi(argv[4]);
    const unsigned n_C1      = std::atoi(argv[5]); // g_size-4; // g_size/4;
    const unsigned n_C2      = std::atoi(argv[5]); // g_size/4;
    const unsigned n_empty   = g_size  -n_C0 -n_C1 -n_C2;

    const unsigned t_max = 200;


    std::printf("Creating random _initial_ grid status with\n"
		"robots:      %d\n"
		"empty slots: %d\n"
		"C0-cars:     %d\n"
		"C1-cars:     %d\n"
		"C2-cars:     %d\n", n_robots, n_empty, n_C0, n_C1, n_C2);
    Rnd_Grid_Stat gState_0      {G, {n_empty,n_C0,n_C1,n_C2}, n_robots };

    std::cout<<"Done. Here it is:\n"
             <<GridSpace::print(gState_0);

    std::printf("Creating random _terminal_ grid status.\n");
    Rnd_Grid_Stat  gState_t_max { G, {n_empty,n_C0,n_C1,n_C2}, n_robots };
    std::cout<<"Done. Here it is:\n"
             <<GridSpace::print(gState_t_max);

    // std::string grid_save  = G.raw_write();
    // std::string stat0_save = GridSpace::raw_write(gState_0);
    // std::string stat1_save = GridSpace::raw_write(gState_t_max);

    std::cout<<"Now creating GuRoBi object w/ t_max="<<t_max<<" ...."<<std::flush;
    GridGRB gGRB {G, t_max};


    std::cout<<
	"done\n"
	"Setting initial grid state...."<<std::flush;
    gGRB.set_initial_state( gState_0 );

    std::cout<<
	"done\n"
	"Setting terminal grid state...."<<std::flush;
    gGRB.set_terminal_state( gState_t_max , GridGRB::Ignore_Robots_In_Terminal_State::No);
//    gGRB.set_terminal_state( gState_0 , GridGRB::Ignore_Robots_In_Terminal_State::Yes);


    constexpr double heuristics = .95;
    constexpr int    n_threads  = 1;
    constexpr double time_limit = 10000.0;
    constexpr int    n_sols     = 1;
    std::cout<<
	"done\n"
	"Starting optimization, with:\n   "<<n_threads<<" threads;\n   time limit "<<time_limit<<"sec;\n   heuristics="<<(int)(100*heuristics)<<"%;\n   terminating as soon as "<<n_sols<<(n_sols >1 ? " solutions are" : " solution is")<<" found.\n"
        "Enjoy."<<std::endl;
    gGRB.optimize(heuristics, n_threads, time_limit, n_sols);

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

        std::cout<<"Creating the raw data strings..."<<std::flush;
        std::string whole_file =
            "# Gurobi Robot Router --- solution file\n"
            "# Syntax:\n"
            "# NS EW \\ngrid(raw)\\ntmax \\ngridstat_0(raw)\\n ... \\ngridstat_tmax(raw)\\n\n"
            "# Comments (lines like this one) are only allowed as a preamble.\n";
        whole_file
            += std::to_string(G.NS) + " " + std::to_string(G.EW)+"\n"
            + G.raw_write() + "\n"
            + std::to_string(t_max) + "\n";
        for (unsigned t=0; t<=t_max; ++t)   whole_file += GridSpace::raw_write( fullsol[t] )  + "\n";
        std::cout<<"done.\nNow opening file..."<<std::flush;
        std::ofstream file {filename};
        file.exceptions(file.exceptions() | std::ios_base::badbit);
        std::cout<<"done.\nWriting to file..."<<std::flush;
        file<<whole_file<<std::flush;
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
