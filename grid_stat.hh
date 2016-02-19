// grid_stat.hh C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#ifndef __ROBOT__GRID_STAT_HH__
#define __ROBOT__GRID_STAT_HH__
#include "grid.hh"
#include <array>
#include <algorithm>
#include <stdexcept>
#include <string>

namespace GridSpace {

    enum class On_Node : char { empty,  Car0,  Car1,  Car2,
	    SIZE};
    enum class NdStat  : char { nobodyhome,
	    R_ready,   R_moving,   R_vertical,
	    C0R_ready, C0R_moving,
	    C1R_ready, C1R_moving,
	    C2R_ready, C2R_moving,
	    SIZE};
    enum class R_Vertical  : char { lift, l1, l2, l3, l4, drop,
	    SIZE};
    enum class R_Move  : char { accE, mvE0, accN, mvN1, mvN0, accW, mvW0, accS, mvS1, mvS0,
	    w0_accE, w0_mvE1, w0_mvE0,   w0_accN, w0_mvN1, w0_mvN2, w0_mvN3, w0_mvN0,   w0_accW, w0_mvW1, w0_mvW0,   w0_accS, w0_mvS1, w0_mvS2, w0_mvS3, w0_mvS0,
	    w1_accE, w1_mvE1, w1_mvE0,   w1_accN, w1_mvN1, w1_mvN2, w1_mvN3, w1_mvN0,   w1_accW, w1_mvW1, w1_mvW0,   w1_accS, w1_mvS1, w1_mvS2, w1_mvS3, w1_mvS0,
	    w2_accE, w2_mvE1, w2_mvE0,   w2_accN, w2_mvN1, w2_mvN2, w2_mvN3, w2_mvN0,   w2_accW, w2_mvW1, w2_mvW0,   w2_accS, w2_mvS1, w2_mvS2, w2_mvS3, w2_mvS0,
	    SIZE};

    inline
    Direction get_direction(R_Move);

    const char* to_string(On_Node);     // don't give "SIZE" to these!
    const char* to_string(NdStat);
    const char* to_string(R_Vertical);
    const char* to_string(R_Move);

    struct Full_Stat {
	On_Node     on_node;
	NdStat      ndstat;
	R_Vertical  r_vert;
        R_Move      r_mv;
	inline Full_Stat(): on_node{On_Node::SIZE}, ndstat{NdStat::SIZE}, r_vert{R_Vertical::SIZE}, r_mv{R_Move::SIZE} {} // B.S. init
	inline Full_Stat(On_Node, NdStat, R_Vertical, R_Move);
    };


    //********************************************************************************************************************************************************************************************************
    typedef      Grid::vector_grid< Full_Stat >       Stat_Vector_t;

    struct Grid_Stat: public Stat_Vector_t
    {
	Grid_Stat(const Grid_Stat &)             = delete;
	Grid_Stat & operator=(const Grid_Stat &) = delete;
    protected:
	Grid_Stat(const Grid & _G): Stat_Vector_t{_G} {}
    }; // struct Grid_Stat


    std::string print     (const Stat_Vector_t&);
    std::string raw_write (const Stat_Vector_t&);
    void        raw_read  (Stat_Vector_t *, const std::string &);


    // Grid_Stat random
    struct Random__Grid_Stat: public Grid_Stat {
	Random__Grid_Stat(const Grid & _G, std::array<unsigned,(unsigned)On_Node::SIZE> car_type_numbers, unsigned n_robots);
    };

    struct Read_From_Raw_Data__Grid_Stat: public Grid_Stat {
        Read_From_Raw_Data__Grid_Stat(const Grid & _G, const std::string & raw): Grid_Stat{_G} { raw_read(this,raw); }
    };



} // namespace GridSpace

//********************************************************************************************************************************************************************************************************
// I M P L E M E N T A T I O N   O F   I N L I N E S

inline
GridSpace::Direction GridSpace::get_direction(GridSpace::R_Move where)
{
    switch(where) {
    case R_Move::accE   :  return Direction::east;
    case R_Move::mvE0   :  return Direction::east;
    case R_Move::accN   :  return Direction::north;
    case R_Move::mvN1   :  return Direction::north;
    case R_Move::mvN0   :  return Direction::north;
    case R_Move::accW   :  return Direction::west;
    case R_Move::mvW0   :  return Direction::west;
    case R_Move::accS   :  return Direction::south;
    case R_Move::mvS1   :  return Direction::south;
    case R_Move::mvS0   :  return Direction::south;
    case R_Move::w0_accE:  return Direction::east;
    case R_Move::w0_mvE1:  return Direction::east;
    case R_Move::w0_mvE0:  return Direction::east;
    case R_Move::w0_accN:  return Direction::north;
    case R_Move::w0_mvN1:  return Direction::north;
    case R_Move::w0_mvN2:  return Direction::north;
    case R_Move::w0_mvN3:  return Direction::north;
    case R_Move::w0_mvN0:  return Direction::north;
    case R_Move::w0_accW:  return Direction::west;
    case R_Move::w0_mvW1:  return Direction::west;
    case R_Move::w0_mvW0:  return Direction::west;
    case R_Move::w0_accS:  return Direction::south;
    case R_Move::w0_mvS1:  return Direction::south;
    case R_Move::w0_mvS2:  return Direction::south;
    case R_Move::w0_mvS3:  return Direction::south;
    case R_Move::w0_mvS0:  return Direction::south;
    case R_Move::w1_accE:  return Direction::east;
    case R_Move::w1_mvE1:  return Direction::east;
    case R_Move::w1_mvE0:  return Direction::east;
    case R_Move::w1_accN:  return Direction::north;
    case R_Move::w1_mvN1:  return Direction::north;
    case R_Move::w1_mvN2:  return Direction::north;
    case R_Move::w1_mvN3:  return Direction::north;
    case R_Move::w1_mvN0:  return Direction::north;
    case R_Move::w1_accW:  return Direction::west;
    case R_Move::w1_mvW1:  return Direction::west;
    case R_Move::w1_mvW0:  return Direction::west;
    case R_Move::w1_accS:  return Direction::south;
    case R_Move::w1_mvS1:  return Direction::south;
    case R_Move::w1_mvS2:  return Direction::south;
    case R_Move::w1_mvS3:  return Direction::south;
    case R_Move::w1_mvS0:  return Direction::south;
    case R_Move::w2_accE:  return Direction::east;
    case R_Move::w2_mvE1:  return Direction::east;
    case R_Move::w2_mvE0:  return Direction::east;
    case R_Move::w2_accN:  return Direction::north;
    case R_Move::w2_mvN1:  return Direction::north;
    case R_Move::w2_mvN2:  return Direction::north;
    case R_Move::w2_mvN3:  return Direction::north;
    case R_Move::w2_mvN0:  return Direction::north;
    case R_Move::w2_accW:  return Direction::west;
    case R_Move::w2_mvW1:  return Direction::west;
    case R_Move::w2_mvW0:  return Direction::west;
    case R_Move::w2_accS:  return Direction::south;
    case R_Move::w2_mvS1:  return Direction::south;
    case R_Move::w2_mvS2:  return Direction::south;
    case R_Move::w2_mvS3:  return Direction::south;
    case R_Move::w2_mvS0:  return Direction::south;
    case R_Move::SIZE:     throw std::range_error  ("GridSpace::to_string(R_Move): out of range");
    default:               throw std::runtime_error("GridSpace::to_string(R_Move): BAD BUG");
    }
} // get_direction()



inline
GridSpace::Full_Stat::Full_Stat(On_Node a, NdStat b, R_Vertical c=R_Vertical::SIZE, R_Move d=R_Move::SIZE):
    on_node{a},
    ndstat{b},
    r_vert{c},
    r_mv{d}
{
    if (!( a!=On_Node::SIZE                                                                                                                  )) throw std::range_error("Grid_Stat::Full_Stat::constructor: bad On_Node [SIZE]");
    if (!( b!=NdStat::SIZE                                                                                                                   )) throw std::range_error("Grid_Stat::Full_Stat::constructor: bad NdStat [SIZE]");
    if (!( (  b==NdStat::R_vertical                                                                           )==(  c!=R_Vertical::SIZE  )   )) throw std::range_error("Grid_Stat::Full_Stat::constructor: R_Vertical mismatch");
    if (!( (  b==NdStat::R_moving || b==NdStat::C0R_moving || b==NdStat::C1R_moving || b==NdStat::C2R_moving  )==(  d!=R_Move::SIZE      )   )) throw std::range_error("Grid_Stat::Full_Stat::constructor: R_Move mismatch");
} // Full_Stat constructor

//********************************************************************************************************************************************************************************************************
// Functions for iteration
inline GridSpace::On_Node    & operator++(GridSpace::On_Node    &x) {return x=(GridSpace::On_Node   )((char)x+1);}
inline GridSpace::NdStat     & operator++(GridSpace::NdStat     &x) {return x=(GridSpace::NdStat    )((char)x+1);}
inline GridSpace::R_Vertical & operator++(GridSpace::R_Vertical &x) {return x=(GridSpace::R_Vertical)((char)x+1);}
inline GridSpace::R_Move     & operator++(GridSpace::R_Move     &x) {return x=(GridSpace::R_Move    )((char)x+1);}

inline GridSpace::On_Node    begin_On_Node   () {return (GridSpace::On_Node   )(0);}
inline GridSpace::NdStat     begin_NdStat    () {return (GridSpace::NdStat    )(0);}
inline GridSpace::R_Vertical begin_R_Vertical() {return (GridSpace::R_Vertical)(0);}
inline GridSpace::R_Move     begin_R_Move    () {return (GridSpace::R_Move    )(0);}

inline GridSpace::On_Node    end_On_Node   () {return GridSpace::On_Node::SIZE;   }
inline GridSpace::NdStat     end_NdStat    () {return GridSpace::NdStat::SIZE;    }
inline GridSpace::R_Vertical end_R_Vertical() {return GridSpace::R_Vertical::SIZE;}
inline GridSpace::R_Move     end_R_Move    () {return GridSpace::R_Move::SIZE;    }


// // use it like this:
// for (On_Node    i=begin_On_Node();    i!=end_On_Node();    ++i) ;
// for (NdStat     i=begin_NdStat();     i!=end_NdStat();     ++i) ;
// for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) ;
// for (R_Move     i=begin_R_Move();     i!=end_R_Move();     ++i) ;

#endif // def __ROBOT__GRID_STATUS_HH__
// EOF grid_stat.hh
