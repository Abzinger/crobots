// SAT_robot_router.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis

#include "grid_sat.hh"
#include "example_grids.hh"
#include "robroute.hh"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <list>
#include <utility>
#include <unistd.h> // for getopt()

typedef GridSpace::Grid                      Grid;
typedef GridSpace::Grid_Sat                  GridSAT;

static
int parse_options(int argc, char *const*argv,
                  const char *                                  *p_filename,
                  int                                           *p_t_max,
                  bool                                          *p_ignore_robots,
                  bool                                          *p_ignore_C0);


int main(int argc, char *const*argv) try
{
    std::srand(std::time(0));
    std::cout<<"SAT Robot Router --- Part of the Robot Routing project"<<std::endl;

    const char * filename;
    int          t_max;
    bool         ignore_robots;
    bool         ignore_C0;

    const int errval=parse_options(argc,argv,
                                   &filename, &t_max,
                                   &ignore_robots, &ignore_C0);
    if (errval) return errval;

    std::cout<<"File:                     "<<filename                        <<'\n'
             <<"t_max:                    "<<t_max                           <<'\n'
             <<"Ignoring robots:          "<<(ignore_robots ? "yes" : "no")  <<'\n'
             <<"Ignoring type-0 cars:     "<<(ignore_C0 ? "yes" : "no")      <<'\n';


    std::ifstream file {filename};
    if (!file) return std::cerr<<"Cannot open file "<<filename<<'\n', 2;
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


    GridSAT gGRB {G, (unsigned)t_max};
    // My options::
    gGRB.options().set_ignore_robots(ignore_robots);
    gGRB.options().set_ignore_C0(ignore_C0);
    // Initial, terminal states:
    gGRB.set_initial_state( &gState_0 );
    gGRB.set_terminal_state( &gState_t_max);

    std::cout<<"Starting optimization. Enjoy."<<std::endl;
    gGRB.optimize();

    std::cout<<
        "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
        "Optimization of Grid_SAT object has terminated.\n"
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



int parse_options(int argc, char *const*argv,
                  const char *                                  *p_filename,
                  int                                           *p_t_max,
                  bool                                          *p_ignore_robots,
                  bool                                          *p_ignore_C0)
{
    const char usgMsg[] =
        "USAGE: SAT_robot_router [-w] [-h] [-c cpusecs] [-i ign] {-g SATParameter=value}   -f filename  -t t_max\n"
        " where  filename    is the name of a instance file in robroute format;\n"
        "        t_max       is the maximum time.\n"
        "Options:\n"
        " -i ign   In the terminal state, ignore:\n"
        "          nothing (default),             if ign=0;\n"
        "          the positions of the robots,   if ign=R;\n"
        "          the positions of type-0 cars,  if ign=C0.\n"
        "\n"
        " -h       Display this help.\n";

    *p_filename = nullptr;
    *p_t_max    = -2000000;

    // defaults:
    *p_ignore_robots   = false;
    *p_ignore_C0       = false;

    const char * options = "hf:t:i:";
    for (char arg=getopt(argc, argv, options); arg!=-1; arg=getopt(argc, argv, options) ) {
        switch (arg) {
        case 'h': std::cout<<usgMsg;                   return 1;
        case 'f': *p_filename = optarg;                break;
        case 't': *p_t_max    = std::atoi(optarg);     break;
        case 'i': if (std::string(optarg)=="0")  *p_ignore_robots=false, *p_ignore_C0=false;
            else  if (std::string(optarg)=="R")  *p_ignore_robots=true;
            else  if (std::string(optarg)=="C0") *p_ignore_C0=true;
            else return std::cerr<<"Unknown parameter "<<optarg<<" to option -i\n",2;
            break;
        case 'g':
        case -1: break;
        default: return std::cerr<<"Something's wrong with the options!\n",2;
        } //^ case
    } //^ for args

    if (!*p_filename)       return std::cerr<<"Missing filename (-h for help)\n", 1;
    if (*p_t_max < -100000) return std::cerr<<"Missing t_max (-h for help)\n", 1;
    if (*p_t_max < 1)       return std::cerr<<"t_max must be at least 1\n", 2;

    return 0;
} //^ parse_options()



// EOF gurobi_robot_router.cc
