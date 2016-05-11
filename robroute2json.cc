// robroute2json.cc C++11
// Part of the Robot Routing project
// Author: Dirk Oliver Theis

#include "grid.hh"
#include "grid_stat.hh"
#include "robroute.hh"

#include <iostream>
#include <fstream>

// #define MY__DEBUG

static
std::string write_pos(GridSpace::Stat_Vector_t & );

int main(int argc, const char *argv[]) try
{
    std::cout<<"robroute2json --- Part of the Robot Routing project"<<std::endl;

    if (argc!=3) {
        std::cerr<<
            "robroute2json extracts grid, initial position, and terminal position from a robroute file\n"
            "and jettisons it to a json file.\n"
            "USAGE: robroute2json rrfn jsfn\n"
            "where\n"
            " rrfn is the name of a file of ``robroute'' type;\n"
            " jsnf is name of the json file which will be created (truncated).\n";
        return 1;
    }

    //******************************************************************************************************************************************************
    const char * const filename    = argv[1];
    const char * const outfilename = argv[2];
#   ifdef MY__DEBUG
    std::cout<<"Opening file ``"<<filename<<"'' for reading..."<<std::flush;
#   endif
    std::ifstream file {filename};
    file.exceptions(file.exceptions() | std::ios_base::badbit | std::ios_base::failbit);
#   ifdef MY__DEBUG
    std::cout<<"done\nReading data from file..."<<std::flush;
#   endif

    std::string                                comments;
    std::vector< GridSpace::Stat_Vector_t >    fullsol;
    GridSpace::Grid * p_G = GridSpace::read_robroute(file, &fullsol, &comments);

    const GridSpace::Grid & G     = *p_G;
    const unsigned          t_max = fullsol.size()-1;

#   ifdef MY__DEBUG
    std::cout<<"done. Looks like we're ready to go!"<<std::endl;
#   endif
#   ifdef MY__DEBUG
    std::cout<<"Opening file ``"<<outfilename<<"'' for writing..."<<std::flush;
#   endif
    std::ofstream outfile {outfilename};
    outfile.exceptions(outfile.exceptions() | std::ios_base::badbit | std::ios_base::failbit);

#   ifdef MY__DEBUG
    std::cout<<"done\nWriting json data to file..."<<std::flush;
#   endif

    outfile<<"["<<G.NS<<','<<G.EW<<','
           <<"[";                                          // Begin grid
    { // write grid
        GridSpace::XY xy = GridSpace::nowhere;
        for (xy.y=G.NS-1; xy.y>=0; --xy.y) {
            if (xy.y==G.NS-1) outfile<< "\n [";
            else              outfile<<",\n [";
            for (xy.x=0; xy.x<G.EW; ++xy.x) {
                if (xy.x==0) outfile<<"\"";
                else         outfile<<",\"";
                if ( G.query(xy).east  )   outfile<<'E';
                if ( G.query(xy).north )   outfile<<'N';
                if ( G.query(xy).west  )   outfile<<'W';
                if ( G.query(xy).south )   outfile<<'S';
                outfile<<"\"";
            } //^ for x
            outfile<<"]";
        } //^ for y
    } // done write grid
    outfile<<"\n]"                                         // End grid
           <<",\n[";                                       // Begin initial pos
    outfile << write_pos( fullsol[0] );
    outfile<<"\n]"                                         // End initial pos
           <<",\n[";                                       // Begin terminal pos
    outfile << write_pos( fullsol[t_max] );
    outfile<<"\n]"                                         // End terminal pos
           <<"\n]";                                        // End jsOBJECTn
    outfile<<std::endl;

#   ifdef MY__DEBUG
    std::cout<<"done\nClosing... "<<std::flush;
#   endif
    file.close();

    std::cout<<"Success."<<std::endl;
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


std::string write_pos(GridSpace::Stat_Vector_t & the_pos)
{
    using GridSpace::On_Node;
    using GridSpace::NdStat;

    std::string ret_string = "";
    const GridSpace::Grid & G = the_pos.G;
    GridSpace::XY xy = GridSpace::nowhere;
    for (xy.y=G.NS-1; xy.y>=0; --xy.y) {
        if (xy.y==G.NS-1) ret_string +=  "\n [";
        else              ret_string += ",\n [";
        for (xy.x=0; xy.x<G.EW; ++xy.x) {
            std::string total_ndst = "";

            if (G.exists(xy)) {
                switch (the_pos[xy].on_node) {
                case On_Node::empty:                      break;
                case On_Node::Car0:  total_ndst += '0';   break;
                case On_Node::Car1:  total_ndst += '1';   break;
                case On_Node::Car2:  total_ndst += '2';   break;
                default:
                    throw std::string("Something is wrong with the On_Node at ("+std::to_string(xy.x)+","+std::to_string(xy.y)+"\n");
                }

                switch (the_pos[xy].ndstat) {
                case NdStat::nobodyhome:                      break;
                case NdStat::R_ready:    total_ndst += "r";   break;
                case NdStat::C0R_ready:  total_ndst += "R0";  break;
                case NdStat::C1R_ready:  total_ndst += "R1";  break;
                case NdStat::C2R_ready:  total_ndst += "R2";  break;

                case NdStat::R_moving: case NdStat::R_vertical: case NdStat::C0R_moving: case NdStat::C1R_moving: case NdStat::C2R_moving:
                    std::cerr<<"robroute2json cannot handle moving objects.\n";
                    throw std::string("Sorry!!!");
                default:
                    throw std::string("Something is wrong with the NdStat at ("+std::to_string(xy.x)+","+std::to_string(xy.y)+"\n");
                }
            } //^ if exists xy

            if (xy.x==0) ret_string +="\"" +total_ndst+'"';
            else         ret_string +=",\""+total_ndst+'"';
        } //^ for x
        ret_string +="]";
    } //^ for y
    return ret_string;
} //^ write_pos()
