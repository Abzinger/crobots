// dump_robroute.cc C++11
// Part of the Robot Routing project
// Author: Dirk Oliver Theis

#include "grid.hh"
#include "grid_stat.hh"
#include "robroute.hh"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

int main(int argc, const char *argv[]) try
{
    const char usgMsg[] =
        "Robot Router dump_robroute\nPart of the Robot Routing project\n"
        "USAGE: dump_robroute [-i|-g] fn\n"
        " where    fn is the name of a file of ``robroute'' type.\n"
        "If the   -i option is given, on the meta data is dumped, not the grids;\n"
        "with the -g option, only meta-data and the 1st grid are dumped.\n";

    const char * filename;
    bool n_grids_to_dump = std::numeric_limits<int>::max;
    switch (argc) {
    case 2:
        filename = argv[1];
        break;
    case 3:
        if (argv[1][0]=='-' && argv[1][1]=='i' && argv[1][2]=='\0') {
            n_grids_to_dump = 0;
            filename = argv[2];
            break;
        } else if (argv[1][0]=='-' && argv[1][1]=='g' && argv[1][2]=='\0') {
            n_grids_to_dump = 1;
            filename = argv[2];
            break;
        }
    default:
        std::cerr<<usgMsg;
        return 1;
    }
    //******************************************************************************************************************************************************

    std::ifstream file {filename};
    file.exceptions(file.exceptions() | std::ios_base::badbit | std::ios_base::failbit);
    std::string                                comments;
    std::vector< GridSpace::Stat_Vector_t >    fullsol;
    GridSpace::Grid * p_G = GridSpace::read_robroute(file, &fullsol, &comments);

    const GridSpace::Grid & G     = *p_G;
    const int               t_max = fullsol.size()-1;

    std::cout<<"File: "<<filename<<'\n'
             <<"NS = "<<G.NS<<"\nEW = "<<G.EW<<"\n#slots = "<<G.numo_slots()
             <<"\nFile comments:\n"
             <<comments<<std::endl;

    for (int t=0; t<=t_max && t<n_grids_to_dump; ++t) {
        std::cout<<"===========================================================================\nt="<<t<<":\n";
        std::cout<<GridSpace::print(fullsol[t]);
    } //^ for

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

// EOF dump_robroute.cc

