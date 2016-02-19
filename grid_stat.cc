// grid_stat.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#include "grid_stat.hh"
#include <stdexcept>

const char* GridSpace::to_string(On_Node what)
{
    switch(what) {
    case On_Node::empty:    return "empty";
    case On_Node::Car0 :    return "Car0 ";
    case On_Node::Car1 :    return "Car1 ";
    case On_Node::Car2 :    return "Car2 ";
    case On_Node::SIZE:     throw std::range_error  ("GridSpace::to_string(On_Node): out of range");
    default:                throw std::runtime_error("GridSpace::to_string(On_Node): BAD BUG");
    }
} // to_string(On_Node)

const char* GridSpace::to_string(NdStat who)
{
    switch(who) {
    case NdStat::nobodyhome:  return "nobodyhome";
    case NdStat::R_ready   :  return "R_ready   ";
    case NdStat::R_moving  :  return "R_moving  ";
    case NdStat::R_vertical:  return "R_vertical";
    case NdStat::C0R_ready :  return "C0R_ready ";
    case NdStat::C0R_moving:  return "C0R_moving";
    case NdStat::C1R_ready :  return "C1R_ready ";
    case NdStat::C1R_moving:  return "C1R_moving";
    case NdStat::C2R_ready :  return "C2R_ready ";
    case NdStat::C2R_moving:  return "C2R_moving";
    case NdStat::SIZE:        throw std::range_error  ("GridSpace::to_string(NdStat): out of range");
    default:                  throw std::runtime_error("GridSpace::to_string(NdStat): BAD BUG");
    }
} // to_string(NdStat)

const char* GridSpace::to_string(R_Vertical vert) 
{
    switch(vert) {
    case R_Vertical::lift:  return "lift";
    case R_Vertical::l1  :  return "l1  ";
    case R_Vertical::l2  :  return "l2  ";
    case R_Vertical::l3  :  return "l3  ";
    case R_Vertical::l4  :  return "l4  ";
    case R_Vertical::drop:  return "drop";
    case R_Vertical::SIZE:    throw std::range_error  ("GridSpace::to_string(R_Vertical): out of range");
    default:                  throw std::runtime_error("GridSpace::to_string(R_Vertical): BAD BUG");
    }
} // to_string(R_Vertical)

const char* GridSpace::to_string(R_Move where)
{
    switch(where) {
    case R_Move::accE   :  return "accE   ";
    case R_Move::mvE0   :  return "mvE0   ";
    case R_Move::accN   :  return "accN   ";
    case R_Move::mvN1   :  return "mvN1   ";
    case R_Move::mvN0   :  return "mvN0   ";
    case R_Move::accW   :  return "accW   ";
    case R_Move::mvW0   :  return "mvW0   ";
    case R_Move::accS   :  return "accS   ";
    case R_Move::mvS1   :  return "mvS1   ";
    case R_Move::mvS0   :  return "mvS0   ";
    case R_Move::w0_accE:  return "w0_accE";
    case R_Move::w0_mvE1:  return "w0_mvE1";
    case R_Move::w0_mvE0:  return "w0_mvE0";
    case R_Move::w0_accN:  return "w0_accN";
    case R_Move::w0_mvN1:  return "w0_mvN1";
    case R_Move::w0_mvN2:  return "w0_mvN2";
    case R_Move::w0_mvN3:  return "w0_mvN3";
    case R_Move::w0_mvN0:  return "w0_mvN0";
    case R_Move::w0_accW:  return "w0_accW";
    case R_Move::w0_mvW1:  return "w0_mvW1";
    case R_Move::w0_mvW0:  return "w0_mvW0";
    case R_Move::w0_accS:  return "w0_accS";
    case R_Move::w0_mvS1:  return "w0_mvS1";
    case R_Move::w0_mvS2:  return "w0_mvS2";
    case R_Move::w0_mvS3:  return "w0_mvS3";
    case R_Move::w0_mvS0:  return "w0_mvS0";
    case R_Move::w1_accE:  return "w1_accE";
    case R_Move::w1_mvE1:  return "w1_mvE1";
    case R_Move::w1_mvE0:  return "w1_mvE0";
    case R_Move::w1_accN:  return "w1_accN";
    case R_Move::w1_mvN1:  return "w1_mvN1";
    case R_Move::w1_mvN2:  return "w1_mvN2";
    case R_Move::w1_mvN3:  return "w1_mvN3";
    case R_Move::w1_mvN0:  return "w1_mvN0";
    case R_Move::w1_accW:  return "w1_accW";
    case R_Move::w1_mvW1:  return "w1_mvW1";
    case R_Move::w1_mvW0:  return "w1_mvW0";
    case R_Move::w1_accS:  return "w1_accS";
    case R_Move::w1_mvS1:  return "w1_mvS1";
    case R_Move::w1_mvS2:  return "w1_mvS2";
    case R_Move::w1_mvS3:  return "w1_mvS3";
    case R_Move::w1_mvS0:  return "w1_mvS0";
    case R_Move::w2_accE:  return "w2_accE";
    case R_Move::w2_mvE1:  return "w2_mvE1";
    case R_Move::w2_mvE0:  return "w2_mvE0";
    case R_Move::w2_accN:  return "w2_accN";
    case R_Move::w2_mvN1:  return "w2_mvN1";
    case R_Move::w2_mvN2:  return "w2_mvN2";
    case R_Move::w2_mvN3:  return "w2_mvN3";
    case R_Move::w2_mvN0:  return "w2_mvN0";
    case R_Move::w2_accW:  return "w2_accW";
    case R_Move::w2_mvW1:  return "w2_mvW1";
    case R_Move::w2_mvW0:  return "w2_mvW0";
    case R_Move::w2_accS:  return "w2_accS";
    case R_Move::w2_mvS1:  return "w2_mvS1";
    case R_Move::w2_mvS2:  return "w2_mvS2";
    case R_Move::w2_mvS3:  return "w2_mvS3";
    case R_Move::w2_mvS0:  return "w2_mvS0";
    case R_Move::SIZE:     throw std::range_error  ("GridSpace::to_string(R_Move): out of range");
    default:               throw std::runtime_error("GridSpace::to_string(R_Move): BAD BUG");
    }
} // to_string(R_Move)

//******************************************************************************************************************************************************
GridSpace::Random__Grid_Stat::Random__Grid_Stat(const Grid & _G, const std::array<unsigned,(unsigned)On_Node::SIZE> car_type_numbers, const unsigned n_robots)
    : Grid_Stat{_G}
{
    // prepare the vector of On_Node values
    std::vector<On_Node> on_slot;
    on_slot.reserve( Grid_Stat::G.numo_slots() );
    for (On_Node type=begin_On_Node(); type!=end_On_Node(); ++type) {
	for (unsigned i=car_type_numbers[(int)type]; i; --i)    on_slot.push_back(type);
    }
    if (on_slot.size()!=Grid_Stat::G.numo_slots()) throw std::range_error("Random_Grid_Stat::constructor: numbers of slots must add up to total number of slots on the Grid.");


    std::vector<NdStat> robot {  Grid_Stat::G.numo_slots(), NdStat::nobodyhome };
    if ( n_robots >= Grid_Stat::G.numo_slots() ) throw std::range_error("Random_Grid_Stat::constructor: you have more robots than slots!.");
    for (unsigned i=0; i<n_robots; ++i) robot[i] = NdStat::R_ready;

    // now comes the randomness
    std::random_shuffle( on_slot.begin(), on_slot.end() );
    std::random_shuffle( robot.begin(), robot.end() );

    // now fill in the values of the the_stat array
    {
	unsigned idx=0;
	for (short y=0; y<Grid_Stat::G.NS_sz(); ++y) {
	    for (short x=0; x<Grid_Stat::G.EW_sz(); ++x) {
		XY xy = {x,y};
		if ( G.exists(xy) )  operator[](xy)=Full_Stat(on_slot[idx],robot[idx]),  idx++;
	    }
	}
	if ( idx != on_slot.size() ) throw std::runtime_error("Random_Grid_Stat::constructor: something's wrong with the number of slots on the grid... (WEIRD BUG)");
    }
} // Random_Grid__Stat --- constructor

//******************************************************************************************************************************************************

static const char dir_arrow[] = {'>','^','<','v'};

struct Index {
    Index(short _x_bd, short _y_bd): x_bd{_x_bd}, y_bd{_y_bd} {}
    const short x_bd;
    const short y_bd;
    int operator() (short x, short y) const { return (3*y_bd-y)*(2*x_bd+2)+x; }
};

std::string
GridSpace::print(const GridSpace::Stat_Vector_t & stat)
{
    const GridSpace::Grid & G=stat.G;
    // get empty grid from G
    std::string s = G.print();

    // fill in data
    for (short x=0; x<G.EW; ++x) {
        for (short y=0; y<G.NS; ++y) {
            XY xy = XY{x,y};
            if ( ! G.exists(xy) )   continue;

            char low,hi;
            Full_Stat st = stat[xy];
            switch (st.on_node) {
            case On_Node::empty:    low=' '; break;
            case On_Node::Car0 :    low='0'; break;
            case On_Node::Car1 :    low='1'; break;
            case On_Node::Car2 :    low='2'; break;
            case On_Node::SIZE:     throw std::range_error  ("Grid_Stat::print(): on_node out of range");
            default:                throw std::runtime_error("Grid_Stat::print(): on_node BAD BUG");
            }
            switch (st.ndstat) {
            case NdStat::nobodyhome:  hi=' ';                                               break;
            case NdStat::R_ready   :  hi='r';                                               break;
            case NdStat::R_moving  :  hi=dir_arrow[(int)get_direction(st.r_mv)];            break;
            case NdStat::R_vertical:  hi=( st.r_vert==R_Vertical::drop ? 'D' : 'L' );       break;
            case NdStat::C0R_ready :  low='R',hi='0';                                       break;
            case NdStat::C0R_moving:  low=dir_arrow[(int)get_direction(st.r_mv)],hi='0';    break;
            case NdStat::C1R_ready :  low='R',hi='1';                                       break;
            case NdStat::C1R_moving:  low=dir_arrow[(int)get_direction(st.r_mv)],hi='1';    break;
            case NdStat::C2R_ready :  low='R',hi='2';                                       break;
            case NdStat::C2R_moving:  low=dir_arrow[(int)get_direction(st.r_mv)],hi='2';    break;
            case NdStat::SIZE:        throw std::range_error  ("Grid_Stat::print() ndstat out of range");
            default:                  throw std::runtime_error("Grid_Stat::print() ndstat BAD BUG");
            }
            s[G.print_idx(xy,1)] = hi;
            s[G.print_idx(xy,0)] = low;
        } // for y
    } // for x

    return s;
} // print()

std::string GridSpace::raw_write(const GridSpace::Stat_Vector_t& stvc)
{
    const GridSpace::Grid & G = stvc.G;

    std::string s;
    s.reserve(4*G.numo_slots());

    for (short y=0; y<G.NS_sz(); ++y) {
        for (short x=0; x<G.EW_sz(); ++x) {
            const XY xy {x,y};
            if (G.exists(xy)) {
                const Full_Stat fs = stvc[xy];
                const char chr[5]  = { (char)('c'+(char)fs.on_node), (char)('A'+(char)fs.ndstat), (char)('v'+(char)fs.r_vert), (char)('!'+(char)fs.r_mv) , 0};
                s.append(chr);
            } // if
        } // for x
    } // for y
    return s;
} // raw_write()


void GridSpace::raw_read(GridSpace::Stat_Vector_t* p_stvc, const std::string & s) {
    Stat_Vector_t         & stvc = *p_stvc;
    GridSpace::Grid const & G    = stvc.G;

    if (s.size() < 4*G.numo_slots()) throw std::runtime_error("GridSpace::raw_read(): string is too short.");

    int idx = 0;
    for (short y=0; y<G.NS_sz(); ++y) {
        for (short x=0; x<G.EW_sz(); ++x) {
            const XY xy {x,y};
            if (G.exists(xy)) {
                On_Node     on_node = (On_Node   )(char)(s[4*idx+0]-'c');
                NdStat      ndstat  = (NdStat    )(char)(s[4*idx+1]-'A');
                R_Vertical  r_vert  = (R_Vertical)(char)(s[4*idx+2]-'v');
                R_Move      r_mv    = (R_Move    )(char)(s[4*idx+3]-'!');

                Full_Stat fs {on_node,ndstat,r_vert,r_mv};
                stvc[xy] = fs;

                ++idx;
            } // if
        } // for x
    } // for y
} // raw_read()




// {
//     const short x_bd = G.EW;
//     const short y_bd = G.NS;
//     Index i {x_bd, y_bd};
//
//     std::string s ( ( 3*y_bd+1 )*( 2*x_bd +2 ) , '#' );
//
//     // draw grid
//     for (short x=0; x<x_bd; ++x) {
//         s    [i(2*x  ,3*y_bd  )] = '+';     s[i(2*x+1,3*y_bd  )] = '-';
//         for (short y=0; y<y_bd; ++y) {
//             s[i(2*x  ,3*y   +2)] = '|';     s[i(2*x+1,3*y   +2)] = ' ';
//             s[i(2*x  ,3*y   +1)] = '|';     s[i(2*x+1,3*y   +1)] = ' ';
//             s[i(2*x  ,3*y     )] = '+';     s[i(2*x+1,3*y     )] = '-';
//         } // for y
//     } // for x
//     s    [i(2*x_bd  ,3*y_bd  )] = '+';      s[i(2*x_bd+1,3*y_bd  )] = '\n';
//     for (short y=0; y<y_bd; ++y) {
//         s[i(2*x_bd  ,3*y   +2)] = '|';      s[i(2*x_bd+1,3*y   +2)] = '\n';
//         s[i(2*x_bd  ,3*y   +1)] = '|';      s[i(2*x_bd+1,3*y   +1)] = '\n';
//         s[i(2*x_bd  ,3*y     )] = '+';      s[i(2*x_bd+1,3*y     )] = '\n';
//     } // for y
//
//     // fill in data
//     for (short x=0; x<x_bd; ++x) {
//         for (short y=0; y<y_bd; ++y) {
//             char low,hi;
//             Full_Stat st = stat[XY{x,y}];
//             switch (st.on_node) {
//             case On_Node::empty:    low=' '; break;
//             case On_Node::Car0 :    low='0'; break;
//             case On_Node::Car1 :    low='1'; break;
//             case On_Node::Car2 :    low='2'; break;
//             case On_Node::SIZE:     throw std::range_error  ("Grid_Stat::print(): on_node out of range");
//             default:                throw std::runtime_error("Grid_Stat::print(): on_node BAD BUG");
//             }
//             switch (st.ndstat) {
//             case NdStat::nobodyhome:  hi=' ';                                               break;
//             case NdStat::R_ready   :  hi='r';                                               break;
//             case NdStat::R_moving  :  hi=dir_arrow[(int)get_direction(st.r_mv)];            break;
//             case NdStat::R_vertical:  hi=( st.r_vert==R_Vertical::drop ? 'L' : 'D' );       break;
//             case NdStat::C0R_ready :  low='R',hi='0';                                       break;
//             case NdStat::C0R_moving:  low=dir_arrow[(int)get_direction(st.r_mv)],hi='0';    break;
//             case NdStat::C1R_ready :  low='R',hi='1';                                       break;
//             case NdStat::C1R_moving:  low=dir_arrow[(int)get_direction(st.r_mv)],hi='1';    break;
//             case NdStat::C2R_ready :  low='R',hi='2';                                       break;
//             case NdStat::C2R_moving:  low=dir_arrow[(int)get_direction(st.r_mv)],hi='2';    break;
//             case NdStat::SIZE:        throw std::range_error  ("Grid_Stat::print() ndstat out of range");
//             default:                  throw std::runtime_error("Grid_Stat::print() ndstat BAD BUG");
//             }
//             s[i(2*x+1,3*y+2)] = hi;
//             s[i(2*x+1,3*y+1)] = low;
//         } // for y
//     } // for x
//
//     return s;
// } print()
//


// EOF grid_stat.cc
