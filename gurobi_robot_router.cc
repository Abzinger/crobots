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
#include <list>
#include <utility>
#include <unistd.h> // for getopt()

#include "gurobi_c++.h"

typedef GridSpace::Grid                      Grid;
typedef GridSpace::Grid_Gurobi               GridGRB;

static
int parse_options(int argc, char *const*argv,
                  const char *                                  *p_filename,
                  int                                           *p_t_max,
                  bool                                          *p_hardwire,
                  bool                                          *p_punish_mismatch,
                  bool                                          *p_early_exit,
                  bool                                          *p_ignore_robots,
                  bool                                          *p_ignore_C0,
                  std::list< std::pair< std::string, int    > > *p_gurobi_int_parameters,
                  std::list< std::pair< std::string, double > > *p_gurobi_double_parameters);


int main(int argc, char *const*argv) try
{
    std::srand(std::time(0));
    std::cout<<"Gurobi Robot Router --- Part of the Robot Routing project"<<std::endl;

    const char * filename;
    int          t_max;
    bool         hardwire        = false;
    bool         punish_mismatch = false;
    bool         early_exit      = false;
    bool         ignore_robots = true;
    bool         ignore_C0     = true;
    std::list< std::pair< std::string, int    > > gurobi_int_parameters;
    std::list< std::pair< std::string, double > > gurobi_double_parameters;

    const int errval=parse_options(argc,argv,
                                   &filename, &t_max,
                                   &hardwire, &punish_mismatch, &early_exit,
                                   &ignore_robots, &ignore_C0,
                                   &gurobi_int_parameters, &gurobi_double_parameters);
    if (errval) return errval;

    std::cout<<"File:                     "<<filename                        <<'\n'
             <<"t_max:                    "<<t_max                           <<'\n'
             <<"Hardwire terminal state:  "<<(hardwire ? "yes" : "no")       <<'\n'
             <<"Punish state mismatch:    "<<(punish_mismatch ? "yes" : "no")<<'\n'
             <<"Early exit:               "<<(early_exit ? "yes" : "no")     <<'\n'
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


    GridGRB gGRB {G, (unsigned)t_max};
    // My options::
    gGRB.options().set_ignore_robots(ignore_robots);
    gGRB.options().set_ignore_C0(ignore_C0);
    gGRB.options().set_hardwire(hardwire);
    gGRB.options().set_punish_mismatch(punish_mismatch);
    gGRB.options().set_early_exit(early_exit);
    // Initial, terminal states:
    gGRB.set_initial_state( &gState_0 );
    gGRB.set_terminal_state( &gState_t_max);
    // Gurobi parameters:
    std::cout<<"Gurobi parameters:\n";
    for (auto it = gurobi_int_parameters.begin();    it != gurobi_int_parameters.end();    ++it)  gGRB.set_GRBparameter(it->first,it->second);
    for (auto it = gurobi_double_parameters.begin(); it != gurobi_double_parameters.end(); ++it)  gGRB.set_GRBparameter(it->first,it->second);

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



int parse_options(int argc, char *const*argv,
                  const char *                                  *p_filename,
                  int                                           *p_t_max,
                  bool                                          *p_hardwire,
                  bool                                          *p_punish_mismatch,
                  bool                                          *p_early_exit,
                  bool                                          *p_ignore_robots,
                  bool                                          *p_ignore_C0,
                  std::list< std::pair< std::string, int    > > *p_gurobi_int_parameters,
                  std::list< std::pair< std::string, double > > *p_gurobi_double_parameters)
{
    const char usgMsg[] =
        "USAGE: gurobi_robot_router [-w] [-h] [-c cpusecs] [-i ign] {-g gurobiParameter=value}   -f filename  -t t_max\n"
        " where  filename    is the name of a instance file in robroute format;\n"
        "        t_max       is the maximum time.\n"
        "Options:\n"
        " -w       The terminal state is hardwired into the model\n"
        "          (otherwise it is in the objecive function).\n"
        " -x       exit as soon as a solution with matching terminal state is found\n"
        " -p       punish mismatch of the terminal state in times before t_max\n"
        " -i ign   In the terminal state, ignore:\n"
        "          nothing,                       if ign=0;\n"
        "          the positions of the robots,   if ign=R;\n"
        "          the positions of type-0 cars,  if ign=C0.\n"
        " -g       Use this to pass parameters directly to Gurobi\n"
        "          (-g ? for a list).\n"
        "\n"
        " -h       Display this help.\n";

    *p_filename = nullptr;
    *p_t_max    = -2000000;

    auto gurobi_parameter_list = GridGRB::list_GRBparameters();
    const char * options = "hwpxf:t:i:g:";
    for (char arg=getopt(argc, argv, options); arg!=-1; arg=getopt(argc, argv, options) ) {
        switch (arg) {
        case 'h': std::cout<<usgMsg;                   return 1;
        case 'f': *p_filename = optarg;                break;
        case 't': *p_t_max    = std::atoi(optarg);     break;
        case 'w': *p_hardwire = true;                  break;
        case 'p': *p_punish_mismatch = true;           break;
        case 'x': *p_early_exit = true;                break;
        case 'i': if (std::string(optarg)=="0")  *p_ignore_robots=false, *p_ignore_C0=false;
            else  if (std::string(optarg)=="R")  *p_ignore_robots=true;
            else  if (std::string(optarg)=="C0") *p_ignore_C0=true;
            else return std::cerr<<"Unknown parameter "<<optarg<<" to option -i\n",2;
            break;
        case 'g':
            if (std::string(optarg)=="?") {
                std::cerr<<"Accepted Gurobi parameters:\n";
                for (auto it=gurobi_parameter_list.begin(); it!=gurobi_parameter_list.end(); ++it)  std::cerr<<"   "<<*it<<'\n';
                return 1;
            } else {
                std::string name  = "";
                std::string value = "";
                char * i;
                for (i=optarg; *i && *i!='='; ++i)  name += *i;
                if (*i!='=') return std::cerr<<"Unable to parse -g option "<<optarg<<'\n',3;
                ++i;
                if (*i=='-') ++i, value+='-';
                if (*i==0) return std::cerr<<"Unable to parse -g option "<<optarg<<'\n',3;
                bool has_point = false;
                for ( ; ('0'<=*i && *i<='9') || *i=='.';  ++i) {
                    value += *i;
                    if (*i=='.') {
                        if (has_point) return std::cerr<<"Unable to parse -g option "<<optarg<<'\n',3;
                        has_point=true;
                    }
                } //^ for value
                if (*i!=0) return std::cerr<<"Unable to parse -g option "<<optarg<<'\n',3;
                if (! gurobi_parameter_list.count(name) ) return std::cerr<<"Unknown parameter name ``"<<optarg<<"'' in -g option (use -g ? for a list)\n",3;
                if (has_point) {
                    std::pair< std::string , double >  the_parameter;
                    the_parameter.first = name;
                    the_parameter.second = std::stod(value);
                    p_gurobi_double_parameters->push_back( the_parameter );
                } else {
                    std::pair< std::string , int >  the_parameter;
                    the_parameter.first = name;
                    the_parameter.second = std::stoi(value);
                    p_gurobi_int_parameters->push_back( the_parameter );
                }
            } //^ if/else
            break;
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
