// gurobi_robot_router.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis

#include "example_grids.hh"
#include "grid.hh"
#include "grid_stat.hh"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <ctime>

#include "gurobi_c++.h"

typedef GridSpace::Grid                      Grid;
typedef GridSpace::Random__Grid              RndGrid;
typedef GridSpace::Random__Grid_Stat         Rnd_Grid_Stat;

//    std::cout<<m3a.print();

int main(int argc, const char *argv[]) try
{
    using std::string;

    const char * const usage_msg =
        "USAGE: \n"
        "generate_robroute -rnd filename NS EW p r c0 c1 c2\n"
        "  --random parking lot of given dimensions\n"
        "    with random initial + terminal configurations;\n"
        "generate_robroute -lib parking_lot_name filename r c0 c1 c2\n"
        "  --use library parking lot of that name\n"
        "    with random initial + terminal configurations\n"
        "generate_robroute -lib\n"
        "  --print list of library parking lots to stdout."
        "Make sure that r, c0+c1+c2  <  free slots.\n"
        "Data is writte to 'filename.robroute'.\n";

    std::cout<<"Robroute Generator --- Part of the Robot Routing project\n"<<std::endl;
    if (argc<7 && argc!=2) {
        std::cerr<<usage_msg;
        return 1;
    }

    Grid * p_grid          = nullptr;
    unsigned fillarg_begin = 987654321;

    std::srand(std::time(0));

    if (string(argv[1]) == "-rnd") { // random grid
        if (argc!=10) {
            std::cerr<<usage_msg;
            return 10;
        }
        fillarg_begin = 5;
        {// check args
            const int    NS        = std::atoi(argv[1]);
            const int    EW        = std::atoi(argv[2]);
            const double p         = std::atof(argv[3]);
            if (NS<1 || NS>255)   return std::cerr<<"NS must be in {1,...,255}\n",2;
            if (EW<1 || EW>255)   return std::cerr<<"EW must be in {1,...,255}\n",2;
            if (p<0.  || p>1.)    return std::cerr<<"p must be in [0,1]\n",2;
        }

        const unsigned short NS  = std::atoi(argv[1]);
        const unsigned short EW  = std::atoi(argv[2]);
        const double         p   = std::atof(argv[3]);
        p_grid = new RndGrid(NS,EW,p);
        std::printf("Using random grid: NS=%d; EW=%d; slots=%d.\n",p_grid->NS_sz(),p_grid->EW_sz(),p_grid->numo_slots());
    } else if (string(argv[1]) == "-lib") {  // example grid
        const bool print_only = ( argc==2 );
        if (!print_only && argc!=8) {
            std::cerr<<usage_msg;
            return 20;
        }
        fillarg_begin = 3;

        if (print_only) std::cout<<"List of library parking lots:\n";
        else std::cout<<"Using";

        if (print_only || string(argv[2]) == "Marsi3A") { p_grid = new GridSpace::Marsi3A_ExampleGrid; std::printf(" Marsi3A grid: NS=%d; EW=%d; slots=%d.\n",p_grid->NS_sz(),p_grid->EW_sz(),p_grid->numo_slots()); }
        if (print_only || string(argv[2]) == "Marsi3B") { p_grid = new GridSpace::Marsi3B_ExampleGrid; std::printf(" Marsi3B grid: NS=%d; EW=%d; slots=%d.\n",p_grid->NS_sz(),p_grid->EW_sz(),p_grid->numo_slots()); }
        if (print_only || string(argv[2]) == "Marsi3C") { p_grid = new GridSpace::Marsi3C_ExampleGrid; std::printf(" Marsi3C grid: NS=%d; EW=%d; slots=%d.\n",p_grid->NS_sz(),p_grid->EW_sz(),p_grid->numo_slots()); }
        if (print_only || string(argv[2]) == "Marsi3D") { p_grid = new GridSpace::Marsi3D_ExampleGrid; std::printf(" Marsi3D grid: NS=%d; EW=%d; slots=%d.\n",p_grid->NS_sz(),p_grid->EW_sz(),p_grid->numo_slots()); }
        if (print_only || string(argv[2]) == "Marsi3E") { p_grid = new GridSpace::Marsi3E_ExampleGrid; std::printf(" Marsi3E grid: NS=%d; EW=%d; slots=%d.\n",p_grid->NS_sz(),p_grid->EW_sz(),p_grid->numo_slots()); }

        if (print_only) return 1;
    } else { // error
        std::cerr<<usage_msg;
        return 2;
    } //^ if/else

    const Grid & G = *p_grid;

    { // check args
        const int n_robots = std::atoi(argv[fillarg_begin+1]);
        const int n_C0     = std::atoi(argv[fillarg_begin+2]);
        const int n_C1     = std::atoi(argv[fillarg_begin+3]);
        const int n_C2     = std::atoi(argv[fillarg_begin+4]);
        if (n_robots<1)                          return std::cerr<<"r must be at least 1\n",32;
        if (n_robots>=(int)G.numo_slots())       return std::cerr<<"Too few free slots ("<<G.numo_slots()<<") for this number of robots ("<<n_robots<<")\n",32;
        if (n_C0<0)                              return std::cerr<<"c0 must be nonnegative\n",32;
        if (n_C0>=(int)G.numo_slots())           return std::cerr<<"Too few free slots ("<<G.numo_slots()<<") for this number of type-0 cars ("<<n_C0<<")\n",32;
        if (n_C1<0)                              return std::cerr<<"c1 must be nonnegative\n",32;
        if (n_C1>=(int)G.numo_slots())           return std::cerr<<"Too few free slots ("<<G.numo_slots()<<") for this number of type-2 cars ("<<n_C2<<")\n",32;
        if (n_C2<0)                              return std::cerr<<"c2 must be nonnegative\n",32;
        if (n_C2>=(int)G.numo_slots())           return std::cerr<<"Too few free slots ("<<G.numo_slots()<<") for this number of type-2 cars ("<<n_C2<<")\n",32;
        if (n_C0+n_C1+n_C2>=(int)G.numo_slots()) return std::cerr<<"Too few free slots ("<<G.numo_slots()<<") for this number of cars ("<<n_C0+n_C1+n_C2<<")\n",32;
    }
    const string   filename  = string(argv[fillarg_begin+0])+".robroute";
    const unsigned g_size    = G.numo_slots();
    const unsigned n_robots  = std::atoi(argv[fillarg_begin+1]);
    const unsigned n_C0      = std::atoi(argv[fillarg_begin+2]);
    const unsigned n_C1      = std::atoi(argv[fillarg_begin+3]);
    const unsigned n_C2      = std::atoi(argv[fillarg_begin+4]);
    const unsigned n_empty   = g_size  -n_C0 -n_C1 -n_C2;

    std::printf("Creating random initial & terminal grid statuses with\n"
                "total # slots %d\n"
                "robots:       %d\n"
                "C0-cars:      %d\n"
                "C1-cars:      %d\n"
                "C2-cars:      %d\n"
                "empty slots:  %d\n", g_size, n_robots, n_C0, n_C1, n_C2, n_empty);

    Rnd_Grid_Stat gState_0 { G, {n_empty,n_C0,n_C1,n_C2}, n_robots };
    Rnd_Grid_Stat gState_1 { G, {n_empty,n_C0,n_C1,n_C2}, n_robots };

    string whole_file =
        "# robroute problem file\n"
        "# Syntax:\n"
        "# NS EW \\ngrid(raw)\\ntmax \\ngridstat_0(raw)\\n ... \\ngridstat_tmax(raw)\\n\n"
        "# Comments (lines like this one) are only allowed as a preamble.\n";
    whole_file
        += std::to_string(G.NS) + " " + std::to_string(G.EW)+"\n"
        + G.raw_write() + "\n"
        + std::to_string(1) + "\n";
    whole_file += GridSpace::raw_write( gState_0 ) + '\n';
    whole_file += GridSpace::raw_write( gState_1 ) + '\n';

    std::cout<<"Writing file ``"<<filename<<"'' ...";
    std::ofstream file {filename};
    file.exceptions(file.exceptions() | std::ios_base::badbit);
    file<<whole_file<<std::flush;
    file.close();
    delete p_grid;

    std::cout<<"Success."<<std::endl;
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
catch(const string &stre) {
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
