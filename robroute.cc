// robroute.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis

#include "robroute.hh"

GridSpace::Grid * GridSpace::read_robroute(std::istream                            & in,
                                           std::vector< GridSpace::Stat_Vector_t > * p_vec,
                                           std::string                             * p_comments)
{
    typedef GridSpace::Read_From_Raw_Data__Grid   Grid;

    in>>std::ws; // eat whitespace
    while (in.peek()=='#') {
        std::string comment;
        std::getline(in,comment);
        *p_comments += comment + '\n';
        in>>std::ws; // eat whitespace
    }
    short NS,EW;
    in>>NS>>EW;

    std::string tmpstr;

    std::getline(in>>std::ws,tmpstr);
    Grid *p_G = new Grid(NS,EW,tmpstr);

    unsigned t_max;
    in>>t_max;

    std::vector< GridSpace::Stat_Vector_t > myvec (t_max+1, *p_G);
    for (unsigned t=0; t<=t_max; ++t) {
        std::getline(in>>std::ws,tmpstr);
        GridSpace::raw_read( &( myvec[t] ), tmpstr );
    } //^ for t

    p_vec->swap( myvec );
    return p_G;
} //^ read_robroute()

void GridSpace::write_robroute(std::ostream                                  & out,
                               const std::vector< GridSpace::Stat_Vector_t > & vec,
                               std::string                                     comments)

{
    if (! vec.size()) throw std::string("GridSpace::write_robroute(): vector of gridstats is empty.");

    const GridSpace::Grid & G { vec[0].G };
    const unsigned t_max = vec.size()-1;
    std::string whole_file = "# Robroute file -- part of the Robots project http://ac.cs.ut.ee\n";
    if (comments.size()) whole_file += "# ";
    for (auto it=comments.begin(); it!=comments.end(); ++it) {
        if (*it=='\n') whole_file += "\n# ";
        else           whole_file += *it;
    }
    whole_file += '\n';
    whole_file
        += std::to_string(G.NS) + " " + std::to_string(G.EW)+"\n"
        + G.raw_write() + "\n"
        + std::to_string(t_max) + "\n";
    for (unsigned t=0; t<=t_max; ++t)   whole_file += GridSpace::raw_write( vec[t] )  + "\n";
    out << whole_file<<std::flush;
} //^ write_robroute()

// ^robroute.cc EOF
