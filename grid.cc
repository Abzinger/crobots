#include "grid.hh"

#include <cstdlib>

struct Index {
    Index(short _x_bd, short _y_bd): x_bd{_x_bd}, y_bd{_y_bd} {}
    const short x_bd;
    const short y_bd;
    int operator() (short x, short y) const { return (3*y_bd-y)*(2*x_bd+2)+x; }
};

std::string GridSpace::Grid::print(const char h_sep, const char h_wall,
                                   const char v_sep, const char v_wall,
                                   const char x_sep, const char x_wall,
                                   const char exist, const char notexist)       const
{
    const short x_bd = EW;
    const short y_bd = NS;
    Index i {x_bd, y_bd};

    std::string s  ( ( 3*y_bd+1 )*( 2*x_bd +2 ) , '#' );

    for (short x=0; x<x_bd; ++x) {
        s    [i(2*x  ,3*y_bd  )] = x_wall;   s[i(2*x+1,3*y_bd  )] = v_wall;
        for (short y=0; y<y_bd; ++y) {
            const Node_type nt = query(x,y);
            const char d = (  nt.south             ?   v_sep  :  v_wall   );
            const char l = (  nt.west              ?   h_sep  :  h_wall   );
            const char c = (  nt.west && nt.south  ?   x_sep  :  x_wall   );
            const char f = (  nt.raw ?                 exist  :  notexist );
            s[i(2*x  ,3*y   +2)] = l;     s[i(2*x+1,3*y   +2)] = f;
            s[i(2*x  ,3*y   +1)] = l;     s[i(2*x+1,3*y   +1)] = f;
            s[i(2*x  ,3*y     )] = c;     s[i(2*x+1,3*y     )] =   d;
        } // for y
    } // for x
    s    [i(2*x_bd  ,3*y_bd  )] = x_wall;      s[i(2*x_bd+1,3*y_bd  )] = '\n';
    for (short y=0; y<y_bd; ++y) {
        s[i(2*x_bd  ,3*y   +2)] = h_wall;      s[i(2*x_bd+1,3*y   +2)] = '\n';
        s[i(2*x_bd  ,3*y   +1)] = h_wall;      s[i(2*x_bd+1,3*y   +1)] = '\n';
        s[i(2*x_bd  ,3*y     )] = x_wall;      s[i(2*x_bd+1,3*y     )] = '\n';
    } // for y

    return s;
} // print()


std::string GridSpace::Grid::raw_write() const
{
    std::string s;
    s.reserve(sz());
    for (unsigned i=0; i<sz(); ++i)  s.push_back(  (char)('A'+data[i].raw)  );
    return s;
} // raw_write()


//********************************************************************************************************************************************************************************************************
void GridSpace::check_Grid_consistency(const Grid & G, std::string whos_asking)
{
    const short NS=G.NS;
    const short EW=G.EW;
    for (short y=0; y<NS; ++y) {
        for (short x=0; x<EW; ++x) {
            Node_type d = G.query(XY{x,y});
            bool is = true;
            if (d.north) {
                if (y >= NS-1) is=false;
                if (! G.query( blind_north( XY{x,y} ) ).south ) is=false;
            }
            if (d.south) {
                if (!y) is=false;
                if (! G.query( blind_south( XY{x,y} ) ).north ) is=false;
            }
            if (d.east) {
                if (x >= EW-1) is=false;
                if (! G.query( blind_east( XY{x,y} ) ).west ) is=false;
            }
            if (d.west) {
                if (!x) is=false;
                if (! G.query( blind_west( XY{x,y} ) ).east ) is=false;
            }
            if (!is) {
                throw std::runtime_error(whos_asking+"-->"+std::string("check_Grid_consistency(): Not! (at ") +std::to_string(x)+","+std::to_string(y)+")");
            }
        } // for x
    } // for y
} // check_Grid_consistency()

//********************************************************************************************************************************************************************************************************

GridSpace::Marsi3_A::Marsi3_A(): Grid{4,4}
{
  data[idx(0,0)] = Node_type({                 Direction::east});
  data[idx(1,0)] = Node_type({Direction::north,Direction::east,Direction::west});
  data[idx(2,0)] = Node_type({Direction::north,Direction::east,Direction::west});
  data[idx(3,0)] = Node_type({Direction::north,Direction::west});

  data[idx(0,1)] = Node_type({                                  Direction::east});
  data[idx(1,1)] = Node_type({Direction::north,Direction::south,Direction::east,Direction::west});
  data[idx(2,1)] = Node_type({Direction::north,Direction::south,Direction::east,Direction::west});
  data[idx(3,1)] = Node_type({Direction::north,Direction::south,Direction::west});

  data[idx(0,2)] = Node_type({                                  Direction::east});
  data[idx(1,2)] = Node_type({Direction::north,Direction::south,Direction::east,Direction::west});
  data[idx(2,2)] = Node_type({Direction::north,Direction::south,Direction::east,Direction::west});
  data[idx(3,2)] = Node_type({Direction::north,Direction::south,Direction::west});

  data[idx(0,3)] = Node_type({                 Direction::east});
  data[idx(1,3)] = Node_type({Direction::south,Direction::east,Direction::west});
  data[idx(2,3)] = Node_type({Direction::south,Direction::east,Direction::west});
  data[idx(3,3)] = Node_type({Direction::south,Direction::west});
} // Marsi3_A ---constructor

//********************************************************************************************************************************************************************************************************
GridSpace::Full_Rectangle__Grid::Full_Rectangle__Grid(short _NS, short _EW): Grid(_NS,_EW)
{
    for (int y=0; y<NS; ++y) {
	for (int x=0; x<EW; ++x) {
	    Node_type nd{ {Direction::north,Direction::south,Direction::east,Direction::west} };
	    if (x==0)  nd.west=false;
	    if (x==EW-1) nd.east=false;
	    if (y==0)  nd.south=false;
	    if (y==NS-1) nd.north=false;
	    Grid::data[Grid::idx(x,y)] = nd;
	}
    }
} // Full_Rectangle__Grid()

//********************************************************************************************************************************************************************************************************
GridSpace::Random__Grid::Random__Grid(short _NS, short _EW, double p): Grid(_NS,_EW)
{
    for (int y=0; y<NS; ++y) {
	for (int x=0; x<EW; ++x) {
	    Node_type nd{ {Direction::north,Direction::south,Direction::east,Direction::west} };
	    if (x==0)  nd.west=false;
	    if (x==EW-1) nd.east=false;
	    if (y==0)  nd.south=false;
	    if (y==NS-1) nd.north=false;
	    Grid::data[Grid::idx(x,y)] = nd;
	} // for x
    } // for y
    for (int y=0; y<NS; ++y) {
	for (int x=0; x<EW; ++x) {
            if ( x+1 < EW ) { // east
                const double q = std::rand()/(double)RAND_MAX;
                if (q > p) {
                    Grid::data[Grid::idx(x,y)].east   = false;
                    Grid::data[Grid::idx(x+1,y)].west = false;
                }
            } // if (east exists)
            if ( y+1 < NS ) { // north
                const double q = std::rand()/(double)RAND_MAX;
                if (q > p) {
                    Grid::data[Grid::idx(x,y)].north   = false;
                    Grid::data[Grid::idx(x,y+1)].south = false;
                }
            } // if (north exists)
	} // for x
    } // for y
} // Random__Grid()


//********************************************************************************************************************************************************************************************************
GridSpace::Read_From_Raw_Data__Grid::Read_From_Raw_Data__Grid(short _NS, short _EW, const std::string & raw): Grid(_NS,_EW)
{
    if (raw.size() < sz()) {
        throw std::runtime_error("Read_From_Raw_Data__Grid --(constructor): NS*EW="+std::to_string(NS)+"*"+std::to_string(EW)+"="+std::to_string(sz())+" too large  or  raw string (sz="+std::to_string(raw.size())+") too short!");
    }
    for (int y=0; y<NS; ++y) {
	for (int x=0; x<EW; ++x) {
            const int   i    = idx(x,y);
            const char  r_nd = (char)(raw[i]-'A');
            if (r_nd < 0 || r_nd > 15) {
                throw std::runtime_error("Read_From_Raw_Data__Grid --(constructor): invalid data in string (idx "
                                         +std::to_string(i)+"=("+std::to_string(x)+","+std::to_string(y)+")");
            }
	    data[i] = Node_type {r_nd};
        } // for x
    } // for y
    check_Grid_consistency(*this,"Read_From_Raw_Data__Grid()");
} // Read_From_Raw_Data__Grid()
