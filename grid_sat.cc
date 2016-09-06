// grid_sat.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#include "grid_sat.hh"
#include "CNF.hh"

#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>


// *****************************************************************************************************************************
// *    Grid_Sat  member functions
// *****************************************************************************************************************************

std::ofstream out("input_crobots");
GridSpace::Grid_Sat::Grid_Sat(const Grid & _G, const unsigned _t_max):
    G                          {_G},
    t_max                      {_t_max},
    p_model                    {new CNF::Model},
    model                      {*p_model},
    p_initial_state            {nullptr},
    p_terminal_state           {nullptr},
    onnode_vars                {_G},
    ndstat_vars                {_G},
    rvertical_vars             {_G},
    rmv_vars                   {_G}
{
    // allocate mem for the arrays containing the variables
    for (short y=0; y<G.NS_sz(); ++y) {
        for (short x=0; x<G.EW_sz(); ++x) {
            XY xy{x,y};
            onnode_vars   [xy].resize(t_max+1);
            ndstat_vars   [xy].resize(t_max+1);
            rvertical_vars[xy].resize(t_max+1);
            rmv_vars      [xy].resize(t_max+1);
        }
    }
    make_model();
} // Grid_Sat---constructor

GridSpace::Grid_Sat::~Grid_Sat()
{
    // delete Var arrays
    for (unsigned t=0; t<=t_max; ++t) {
        for (short y=0; y<G.NS_sz(); ++y) {
            for (short x=0; x<G.EW_sz(); ++x) {
                XY xy {x,y};
                if ( G.exists(xy) )    delete[]  onnode_vars[xy][t];
            } // for x
        } // for y
    } // for t
    // delete model & env
    delete p_model;
} //    ~Grid_Sat

//********************************************************************************************************************************************************************************************************
void GridSpace::Grid_Sat::set_initial_state(const Stat_Vector_t *p_stat0)
{
    if (p_initial_state) throw std::runtime_error("Grid_Sat::set_initial_state(): Attempt to set initial state a 2nd time.");
    p_initial_state = p_stat0;

    for (short y=0; y<G.NS_sz(); ++y) {
        for (short x=0; x<G.EW_sz(); ++x) {
            XY xy {x,y};
		    CNF::Clause c;
            if ( G.exists(xy) )  {
                const Full_Stat s = (*p_stat0)[xy];

                for (On_Node    i=begin_On_Node();    i!=end_On_Node();    ++i) {
		  //const double RHS = (s.on_node==i ? 1 : 0 );
		  //model.addConstr( RHS == var(xy,0,i) );
		  CNF::Var RHS = (s.on_node==i ? CNF::One : not(CNF::One) );
		  CNF::Var lol = var(xy,0,i);
		  c = RHS or not(lol);
		  model.addClause(c);
		  c = not(RHS) or lol;
		  model.addClause(c);
		  model.dump(out);
                }
                for (NdStat     i=begin_NdStat();     i!=end_NdStat();     ++i) {
		  //const double RHS = (s.ndstat==i ? 1 : 0 );
		  //model.addConstr( RHS == var(xy,0,i) );
		  CNF::Var RHS = (s.ndstat==i ? CNF::One : not(CNF::One) );
		  CNF::Var lol = var(xy,0,i);
		  c = RHS or not(lol);
		  model.addClause(c);
		  c = not(RHS) or lol;
		  model.addClause(c);
		  model.dump(out);
                }
                for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
		  //const double RHS = (s.r_vert==i ? 1 : 0 );
		  // model.addConstr( RHS == var(xy,0,i) );
		  CNF::Var RHS = (s.r_vert==i ? CNF::One : not(CNF::One) );
		  CNF::Var lol = var(xy,0,i);
		  c = RHS or not(lol);
		  model.addClause(c);
		  c = not(RHS) or lol;
		  model.addClause(c);
		  model.dump(out);
                }
                for (R_Move       i=begin_R_Move();       i!=end_R_Move();       ++i) {
		  //const double RHS = (s.r_mv==i ? 1 : 0 );
		  //model.addConstr( RHS == var(xy,0,i) );
		  CNF::Var RHS = (s.r_mv==i ? CNF::One : not(CNF::One) );
		  CNF::Var lol = var(xy,0,i);
		  c = RHS or not(lol);
		  model.addClause(c);
		  c = not(RHS) or lol;
		  model.addClause(c);
		  model.dump(out);
                }
            } // if exists
        } // for x
    } // for y
} //^ set_initial_state()

void GridSpace::Grid_Sat::set_terminal_state(const Stat_Vector_t * p_state)
{
    if (p_terminal_state) throw std::runtime_error("Grid_Sat::set_initial_state(): Attempt to set terminal state a 2nd time.");
    p_terminal_state = p_state;

    for (short y=0; y<G.NS; ++y) {
        for (short x=0; x<G.EW; ++x) {
            XY xy {x,y};
		    CNF::Clause c;
            if ( G.exists(xy) )  {
                const Full_Stat s = (*p_state)[xy];

                if ( s.on_node!=On_Node::empty && ( !my_opts.ignore_C0 || s.on_node!=On_Node::Car0 ) ) {
                    for (    On_Node    i=begin_On_Node();    i!=end_On_Node();    ++i) {
		      //const double RHS = (s.on_node==i ? 1 : 0 );
		      //model.addConstr(     RHS == var(xy,t_max,i) );
		      CNF::Var RHS = (s.on_node==i ? CNF::One : not(CNF::One) );
		      CNF::Var lol = var(xy,t_max,i);
		      c = RHS or not(lol);
		      model.addClause(c);
		      c = not(RHS) or lol;
		      model.addClause(c);
		      model.dump(out);
                    } //^ for stat
                } //^ if whether to ignore C0
                if (!my_opts.ignore_robots) {
                    for (NdStat     i=begin_NdStat();     i!=end_NdStat();     ++i) {
		      //const double RHS = (s.ndstat==i ? 1 : 0 );
		      //model.addConstr( RHS == var(xy,t_max,i) );
		      CNF::Var RHS = (s.ndstat==i ? CNF::One : not(CNF::One) );
		      CNF::Var lol = var(xy,t_max,i);
		      c = RHS or not(lol);
		      model.addClause(c);
		      c = not(RHS) or lol;
		      model.addClause(c);
		      model.dump(out);
                    }
                    for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
		      //const double RHS = (s.r_vert==i ? 1 : 0 );
		      //model.addConstr( RHS == var(xy,t_max,i) );
		      CNF::Var RHS = (s.r_vert==i ? CNF::One : not(CNF::One) );
		      CNF::Var lol = var(xy,t_max,i);
		      c = RHS or not(lol);
		      model.addClause(c);
		      c = not(RHS) or lol;
		      model.addClause(c);
		      model.dump(out);
                    }
                    for (R_Move       i=begin_R_Move();       i!=end_R_Move();       ++i) {
                        const Direction d = get_direction(i);
                        if ( G.move(xy,d)!=nowhere ) {
			  //const double RHS = (s.r_mv==i ? 1 : 0 );
			  //model.addConstr( RHS == var(xy,t_max,i) );
			  CNF::Var RHS = (s.r_mv==i ? CNF::One : not(CNF::One) );
			  CNF::Var lol = var(xy,t_max,i);
			  c = RHS or not(lol);
			  model.addClause(c);
			  c = not(RHS) or lol;
			  model.addClause(c);
			  model.dump(out);
                        } //^ if dir exists
                    }
                } // if (do robots)
            } // if exists
        } // for x
    } // for y
} // set_terminal_state()

//********************************************************************************************************************************************************************************************************

char command[512];
char input_CNF[512];
void GridSpace::Grid_Sat::optimize()
{
  std::sprintf(command,"cp input_crobots /home/abdullah/SAT_solver_oriented_coloring/cryptominisat-master/build/;rm input_crobots; cd ~/SAT_solver_oriented_coloring/cryptominisat-master/build; nohup ./cryptominisat5_simple input_crobots&>Output_file &");
  system(command);
} //^ optimize()

//********************************************************************************************************************************************************************************************************

std::vector< GridSpace::Stat_Vector_t > GridSpace::Grid_Sat::get_solution()  const
{
  std::ifstream sat_output("Output_file");
  model.read_DIMACS(sat_output);
  std::vector< Stat_Vector_t > fullsol (t_max+1, G);
  for (unsigned t=0; t<=t_max; ++t) {
    XY v {0,0};
    for (v.y=0; v.y<G.NS_sz(); ++v.y) {
      for (v.x=0; v.x<G.EW_sz(); ++v.x) {
	if ( G.exists(v) ) {
	  On_Node on_node = On_Node::SIZE;
	  for (On_Node    i=begin_On_Node();    i!=end_On_Node();    ++i) {
	    CNF::Var x = var(v,t,i);
	    const double val = model.get_value(x);
	    if (val>.1 && val<.9)    throw std::runtime_error("Grid_Sat::get_solution(): This On_Node variable doesn't appear to be integral.");
	    if (val > .5) {
	      if (on_node!=On_Node::SIZE) throw std::runtime_error("Grid_Sat::get_solution(): There seem to be >1 On_Node variables with value 1.");
	      on_node=i;
	    }
	  } // for  On_Node
	  NdStat ndstat = NdStat::SIZE;
	  for (NdStat     i=begin_NdStat();     i!=end_NdStat();     ++i) {
	    CNF::Var x = var(v,t,i);
	    const double val = model.get_value(x);
	    if (val>.1 && val<.9) throw std::runtime_error("Grid_Sat::get_solution(): This NdStat variable doesn't appear to be integral.");
	    if (val > .5) {
	      if (ndstat!=NdStat::SIZE) throw std::runtime_error("Grid_Sat::get_solution(): There seem to be >1 NdStat variables with value 1.");
	      ndstat=i;
	    }
	  } // for  NdStat

	  R_Vertical r_vert = R_Vertical::SIZE;
	  for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
	    CNF::Var x = var(v,t,i);
	    const double val = model.get_value(x);
	    if (val>.1 && val<.9) throw std::runtime_error("Grid_Sat::get_solution(): This R_Vertical variable doesn't appear to be integral.");
	    if (val > .5) {
	      if (r_vert!=R_Vertical::SIZE) throw std::runtime_error("Grid_Sat::get_solution(): There seem to be >1 R_Vertical variables with value 1.");
	      r_vert=i;
	    }
	  } // for  R_Vertical

	  R_Move r_mv = R_Move::SIZE;
	  for (R_Move     i=begin_R_Move();     i!=end_R_Move();     ++i) {
	    const Direction d = get_direction(i);
	    double val = 0.;
	    if ( G.move(v,d)!=nowhere ) {
	      CNF::Var x = var(v,t,i);
	      val = model.get_value(x);
	    }
	    if (val>.1 && val<.9) throw std::runtime_error("Grid_Sat::get_solution(): This R_Move variable doesn't appear to be integral.");
	    if (val > .5) {
	      if (r_mv!=R_Move::SIZE) throw std::runtime_error("Grid_Sat::get_solution(): There seem to be >1 R_Move variables with value 1.");
	      r_mv=i;
	    }
	  } // for  R_Move

	  fullsol[t][v] = Full_Stat{on_node,ndstat,r_vert,r_mv};		
	} // if exists
      } // for x
    } // for y
  } // for t
  
  return fullsol;
} // get_solution()


//********************************************************************************************************************************************************************************************************
//  A C C E S S    T O   V A R I B L E S
//********************************************************************************************************************************************************************************************************


inline
CNF::Var GridSpace::Grid_Sat::var(const XY v, const unsigned t, const On_Node what) const
{
    if ((unsigned)what == (unsigned)On_Node::SIZE) throw std::range_error  ("Grid_Sat::var(On_Node): On_Node argument is out of range");
    if ((unsigned)what >  (unsigned)On_Node::SIZE) throw std::runtime_error("Grid_Sat::var(On_Node): On_Node argument is broken (BAD BUG)");

    if (v==nowhere) {
      if (what==On_Node::empty) return CNF::One;
      else                      return not(CNF::One);
    } else {
        if (! G.exists(v) ) throw std::range_error  ("Grid_Sat::var(On_Node): node does not exist.");
        return onnode_vars[v][t][(int)what];
    }
} // var()

inline
CNF::Var GridSpace::Grid_Sat::var(const XY v, const unsigned t, const NdStat who) const
{
    if ((unsigned)who == (unsigned)NdStat::SIZE) throw std::range_error  ("Grid_Sat::var(NdStat): NdStat argument is out of range");
    if ((unsigned)who >  (unsigned)NdStat::SIZE) throw std::runtime_error("Grid_Sat::var(NdStat): NdStat argument is broken (BAD BUG)");

    if (v==nowhere) {
      if (who==NdStat::nobodyhome) return CNF::One;
      else                         return not(CNF::One);
    } else {
        if (! G.exists(v) ) throw std::range_error  ("Grid_Sat::var(NdStat): node does not exist.");
        return ndstat_vars[v][t][(int)who];
    }
} // var()

inline
CNF::Var GridSpace::Grid_Sat::var(const XY v, const unsigned t, const R_Vertical vert) const
{
    if ((unsigned)vert == (unsigned)R_Vertical::SIZE) throw std::range_error  ("Grid_Sat::var(R_Vertical): R_Vertical argument is out of range");
    if ((unsigned)vert >  (unsigned)R_Vertical::SIZE) throw std::runtime_error("Grid_Sat::var(R_Vertical): R_Vertical argument is broken (BAD BUG)");

    if (v==nowhere) return not(CNF::One);
    else {
        if (! G.exists(v) ) throw std::range_error  ("Grid_Sat::var(R_Vertical): node does not exist.");
        return rvertical_vars[v][t][(int)vert];
    }
} // var()

inline
CNF::Var GridSpace::Grid_Sat::var(const XY v, const unsigned t, const R_Move where) const
{
    if ((unsigned)where == (unsigned)R_Move::SIZE) throw std::range_error  ("Grid_Sat::var(R_Move): R_Move argument is out of range");
    if ((unsigned)where >  (unsigned)R_Move::SIZE) throw std::runtime_error("Grid_Sat::var(R_Move): R_Move argument is broken (BAD BUG)");

    if (v==nowhere) return not(CNF::One);
    else {
        if (! G.exists(v) ) throw std::range_error  ("Grid_Sat::var(R_Move): node does not exist.");
        const Direction d = get_direction(where);
        return ( G.move(v,d)==nowhere ?    CNF::Zero   :   rmv_vars[v][t][(int)where]  );
    }
} // var()


//********************************************************************************************************************************************************************************************************
//  C R E A T E   T H E   M O D E L
//********************************************************************************************************************************************************************************************************
void GridSpace::Grid_Sat::make_model()
{
    make_vars();
    make_constraints();
} // make_model()


//********************************************************************************************************************************************************************************************************
//  V A R I A B L E S
//********************************************************************************************************************************************************************************************************
void GridSpace::Grid_Sat::make_vars()
{
    for (short y=0; y<G.NS_sz(); ++y) {
        for (short x=0; x<G.EW_sz(); ++x) {
            XY xy {x,y};
            if ( G.exists(xy) ) {
                for (unsigned t=0; t<=t_max; ++t)    atom_vars(xy,t);
            } // if exists
        } // for x
    } // for y
} // make_vars()

void GridSpace::Grid_Sat::atom_vars(const XY v, const unsigned t)
{
    // how many vars per node,time ---in total?
    constexpr int total_size = (int)On_Node::SIZE + (int)NdStat::SIZE + (int)R_Vertical::SIZE + (int)R_Move::SIZE;

    // allocate the mem
    CNF::Var * var_array = new CNF::Var[total_size];

    // store the vars
    // char var_name_buffer[1024];  // for the variable names

    int offset = 0;

    onnode_vars[v][t] = var_array+offset;
    for (On_Node i=begin_On_Node(); i!=end_On_Node(); ++i) {
        // std::sprintf(var_name_buffer, "onnd[%.3d:(%.2d,%.2d):%s]",t,v.x,v.y, to_string(i) );
        var_array[offset++] = model.addVar();
    }

    ndstat_vars[v][t] = var_array+offset;
    for (NdStat i=begin_NdStat(); i!=end_NdStat(); ++i) {
        // std::sprintf(var_name_buffer, "ndst[%.3d:(%.2d,%.2d):%s]",t,v.x,v.y, to_string(i) );
        var_array[offset++] = model.addVar();
    } // for (ndstat)

    rvertical_vars[v][t] = var_array+offset;
    for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
        // std::sprintf(var_name_buffer, "rvrt[%.3d:(%.2d,%.2d):%s]",t,v.x,v.y, to_string(i) );
        var_array[offset++] = model.addVar();
    }


    rmv_vars[v][t] = var_array+offset;
    for (R_Move i=begin_R_Move(); i!=end_R_Move(); ++i) {
        const Direction d = get_direction(i);
        if ( G.move(v,d)!=nowhere ) {
            // std::sprintf(var_name_buffer, "rmv[%.3d:(%.2d,%.2d):%s]",t,v.x,v.y, to_string(i) );
            var_array[offset++] = model.addVar();
        } else {
            ++offset;
        }
    } // for (r_move)

    // dumbness check
    if (offset != total_size) throw std::runtime_error("atom_vars(): Counted variables in two ways, and the two results don't match. :(");
} // atom_vars()


//********************************************************************************************************************************************************************************************************
//  C O N S T R A I N T S
//********************************************************************************************************************************************************************************************************

void GridSpace::Grid_Sat::make_constraints()
{
    for (short y=0; y<G.NS_sz(); ++y) {
        for (short x=0; x<G.EW_sz(); ++x) {
            XY xy {x,y};
            if ( G.exists(xy) )   {
                for (unsigned t=0; t<t_max; ++t) {
                    atom_constraints(xy,t);
                    time_link_constraints(xy,t);
                } // for t
                atom_constraints(xy,t_max);
            } // if exists
        } // for x
    } // for y
} // constraints()


//********************************************************************************************************************************************************************************************************
//       A T O M   C O N S T R A I N T S
//********************************************************************************************************************************************************************************************************


void GridSpace::Grid_Sat::atom_constraints(const XY v, const unsigned t)
{
    // B A S I C
    {
        const CNF::Var Here_now_empty       = var(v,       t,    On_Node::empty);
	const CNF::Var Here_now_car0        = var(v,       t,    On_Node::Car0);
	const CNF::Var Here_now_car1        = var(v,       t,    On_Node::Car1);
	const CNF::Var Here_now_car2        = var(v,       t,    On_Node::Car2);
	
	CNF::Clause c;
	c = Here_now_empty      or Here_now_car0       or Here_now_car1  or Here_now_car2;
	model.addClause(c);
	c = not(Here_now_empty) or not(Here_now_car0);
	model.addClause(c);
	c = not(Here_now_empty) or not(Here_now_car1);
	model.addClause(c);
	c = not(Here_now_empty) or not(Here_now_car2);
	model.addClause(c);
	c = not(Here_now_car0)  or not(Here_now_car1);
	model.addClause(c);
	c = not(Here_now_car0)  or not(Here_now_car2);
	model.addClause(c);
	c = not(Here_now_car1)  or not(Here_now_car2);
	model.addClause(c);
	
	const CNF::Var Here_now_nobodyhome       = var(v,       t,    NdStat::nobodyhome  );
	const CNF::Var Here_now_R_ready          = var(v,       t,    NdStat::R_ready     );
	const CNF::Var Here_now_C0R_ready        = var(v,       t,    NdStat::C0R_ready   );
	const CNF::Var Here_now_C1R_ready        = var(v,       t,    NdStat::C1R_ready   );
	const CNF::Var Here_now_C2R_ready        = var(v,       t,    NdStat::C2R_ready   );
	const CNF::Var Here_now_R_moving         = var(v,       t,    NdStat::R_moving    );
	const CNF::Var Here_now_C0R_moving       = var(v,       t,    NdStat::C0R_moving  );
	const CNF::Var Here_now_C1R_moving       = var(v,       t,    NdStat::C1R_moving  );
	const CNF::Var Here_now_C2R_moving       = var(v,       t,    NdStat::C2R_moving  );
	const CNF::Var Here_now_R_vertical       = var(v,       t,    NdStat::R_vertical  );
	
	c = Here_now_nobodyhome                                                                                                    or Here_now_R_ready        or Here_now_C0R_ready         or Here_now_C1R_ready  or Here_now_C2R_ready                  or Here_now_R_moving       or Here_now_C0R_moving        or Here_now_C1R_moving or Here_now_C2R_moving                 or Here_now_R_vertical;

	model.addClause(c);      
	c = not(Here_now_nobodyhome) or not(Here_now_R_ready);
	model.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C0R_ready);
	model.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C1R_ready);
	model.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C2R_ready);
	model.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_R_moving);
	model.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_R_vertical);
	model.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C0R_ready);
	model.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C1R_ready);
	model.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C2R_ready);
	model.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_R_moving);
	model.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_R_vertical);
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C1R_ready);
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C2R_ready);
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_R_vertical);
	model.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_C2R_ready);
	model.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_R_vertical);
	model.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_R_moving);
	model.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_R_vertical);
	model.addClause(c);
	c = not(Here_now_R_moving)   or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_R_moving)   or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_R_moving)   or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_R_moving)   or not(Here_now_R_vertical);
	model.addClause(c);
	c = not(Here_now_C0R_moving) or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_moving) or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_moving) or not(Here_now_R_vertical);
	model.addClause(c);
	c = not(Here_now_C1R_moving) or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_moving) or not(Here_now_R_vertical);
	model.addClause(c);
	c = not(Here_now_C2R_moving) or not(Here_now_R_vertical);
	model.addClause(c);


	// at most one of crobot or car:
	
	c = not(Here_now_C0R_ready)     or Here_now_C1R_ready       or Here_now_C2R_ready      or Here_now_C0R_moving              or Here_now_C1R_moving      or Here_now_C2R_moving      or Here_now_empty;
	model.addClause(c);
	c = Here_now_C0R_ready          or not(Here_now_C1R_ready)  or Here_now_C2R_ready      or Here_now_C0R_moving              or Here_now_C1R_moving      or Here_now_C2R_moving      or Here_now_empty;
	model.addClause(c);
	c = Here_now_C0R_ready          or Here_now_C1R_ready       or not(Here_now_C2R_ready) or Here_now_C0R_moving              or Here_now_C1R_moving      or Here_now_C2R_moving      or Here_now_empty;
	model.addClause(c);
	c = Here_now_C0R_ready          or Here_now_C1R_ready       or Here_now_C2R_ready      or not(Here_now_C0R_moving)         or Here_now_C1R_moving      or Here_now_C2R_moving      or Here_now_empty;
	model.addClause(c);
	c = Here_now_C0R_ready          or Here_now_C1R_ready       or Here_now_C2R_ready      or Here_now_C0R_moving              or not(Here_now_C1R_moving) or Here_now_C2R_moving      or Here_now_empty;
	model.addClause(c);
	c = Here_now_C0R_ready          or Here_now_C1R_ready       or Here_now_C2R_ready      or Here_now_C0R_moving              or Here_now_C1R_moving      or not(Here_now_C2R_moving) or Here_now_empty;
	model.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C1R_ready);
	model.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C2R_ready);
	model.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_ready)     or not(Here_now_C2R_ready);
	model.addClause(c);
	c = not(Here_now_C1R_ready)     or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_ready)     or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_ready)     or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C2R_ready)     or not(Here_now_C0R_moving);
	model.addClause(c);
	c = not(Here_now_C2R_ready)     or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_C2R_ready)     or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_moving)    or not(Here_now_C1R_moving);
	model.addClause(c);
	c = not(Here_now_C0R_moving)    or not(Here_now_C2R_moving);
	model.addClause(c);
	c = not(Here_now_C1R_moving)    or not(Here_now_C2R_moving);
	model.addClause(c);


	const CNF::Var Here_now_R_lift           = var(v,       t,    R_Vertical::lift);
	const CNF::Var Here_now_R_lifting1       = var(v,       t,    R_Vertical::l1);
	const CNF::Var Here_now_R_lifting2       = var(v,       t,    R_Vertical::l2);
	const CNF::Var Here_now_R_lifting3       = var(v,       t,    R_Vertical::l3);
	const CNF::Var Here_now_R_lifting4       = var(v,       t,    R_Vertical::l4);
	const CNF::Var Here_now_R_drop           = var(v,       t,    R_Vertical::l4);
	// neccessarly 
	c = not(Here_now_R_vertical)    or Here_now_R_lift          or Here_now_R_lifting1     or Here_now_R_lifting2              or Here_now_R_lifting3      or Here_now_R_lifting4      or Here_now_R_drop;
	model.addClause(c);
	// sufficient 
	// 6 choose 1
	c = not(Here_now_R_lift)        or Here_now_R_lifting1      or Here_now_R_lifting2     or Here_now_R_lifting3              or Here_now_R_lifting4      or Here_now_R_drop          or Here_now_R_vertical;
	model.addClause(c);
	c = Here_now_R_lift             or not(Here_now_R_lifting1) or Here_now_R_lifting2     or Here_now_R_lifting3              or Here_now_R_lifting4      or Here_now_R_drop          or Here_now_R_vertical;
	model.addClause(c);
	c = Here_now_R_lift             or Here_now_R_lifting1      or not(Here_now_R_lifting2)or Here_now_R_lifting3              or Here_now_R_lifting4      or Here_now_R_drop          or Here_now_R_vertical;
	model.addClause(c);
	c = Here_now_R_lift             or Here_now_R_lifting1      or Here_now_R_lifting2     or not(Here_now_R_lifting3)         or Here_now_R_lifting4      or Here_now_R_drop          or Here_now_R_vertical;
	model.addClause(c);
	c = Here_now_R_lift             or Here_now_R_lifting1      or Here_now_R_lifting2     or Here_now_R_lifting3              or not(Here_now_R_lifting4) or Here_now_R_drop          or Here_now_R_vertical;
	model.addClause(c);
	c = Here_now_R_lift             or Here_now_R_lifting1      or Here_now_R_lifting2     or Here_now_R_lifting3              or Here_now_R_lifting4      or not(Here_now_R_drop)     or Here_now_R_vertical;
	model.addClause(c);
	// 6 choose 2
	c = not(Here_now_R_lift)        or not(Here_now_R_lifting1);
	model.addClause(c);
	c = not(Here_now_R_lift)        or not(Here_now_R_lifting2);
	model.addClause(c);
	c = not(Here_now_R_lift)        or not(Here_now_R_lifting3);
	model.addClause(c);
	c = not(Here_now_R_lift)        or not(Here_now_R_lifting4);
	model.addClause(c);
	c = not(Here_now_R_lift)        or not(Here_now_R_drop);
	model.addClause(c);
	c = not(Here_now_R_lifting1)    or not(Here_now_R_lifting2);
	model.addClause(c);
	c = not(Here_now_R_lifting1)    or not(Here_now_R_lifting3);
	model.addClause(c);
	c = not(Here_now_R_lifting1)    or not(Here_now_R_lifting4);
	model.addClause(c);
	c = not(Here_now_R_lifting1)    or not(Here_now_R_drop);
	model.addClause(c);
	c = not(Here_now_R_lifting2)    or not(Here_now_R_lifting3);
	model.addClause(c);
	c = not(Here_now_R_lifting2)    or not(Here_now_R_lifting4);
	model.addClause(c);
	c = not(Here_now_R_lifting2)    or not(Here_now_R_drop);
	model.addClause(c);
	c = not(Here_now_R_lifting3)    or not(Here_now_R_lifting4);
	model.addClause(c);
	c = not(Here_now_R_lifting3)    or not(Here_now_R_drop);
	model.addClause(c);
	c = not(Here_now_R_lifting4)    or not(Here_now_R_drop);


	const CNF::Var  Here_now_R_accE = var(v,    t,     R_Move::accE);
	const CNF::Var  Here_now_R_mvE0 = var(v,    t,     R_Move::mvE0);
	const CNF::Var  Here_now_R_accW = var(v,    t,     R_Move::accW);
	const CNF::Var  Here_now_R_mvW0 = var(v,    t,     R_Move::mvW0);
	const CNF::Var  Here_now_R_accN = var(v,    t,     R_Move::accN);
	const CNF::Var  Here_now_R_mvN1 = var(v,    t,     R_Move::mvN1);
	const CNF::Var  Here_now_R_mvN0 = var(v,    t,     R_Move::mvN0);
	const CNF::Var  Here_now_R_accS = var(v,    t,     R_Move::accS);
	const CNF::Var  Here_now_R_mvS1 = var(v,    t,     R_Move::mvS1);
	const CNF::Var  Here_now_R_mvS0 = var(v,    t,     R_Move::mvS0);
	// necessarly
	c = not(Here_now_R_moving)                                                                                                 or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	// sufficient
	// 10 choose 1
	c = Here_now_R_moving                                                                                                      or not(Here_now_R_accE) or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         not(Here_now_R_mvE0)                                                or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or not(Here_now_R_accW) or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         not(Here_now_R_mvW0)                                                or Here_now_R_accN      or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or not(Here_now_R_accN) or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or not(Here_now_R_mvN1) or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or Here_now_R_mvN1      or not(Here_now_R_mvN0)                                                or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or not(Here_now_R_accS) or Here_now_R_mvS1      or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or Here_now_R_accS      or not(Here_now_R_mvS1) or Here_now_R_mvS0;
	model.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or Here_now_R_mvN1      or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or not(Here_now_R_mvS0);
	model.addClause(c);
	//10 choose 2
	c = not(Here_now_R_accE)    or not(Here_now_R_mvE0);
	model.addClause(c);
	c = not(Here_now_R_accE)    or not(Here_now_R_accW);
	model.addClause(c);
	c = not(Here_now_R_accE)    or not(Here_now_R_mvW0);
	model.addClause(c);
	c = not(Here_now_R_accE)    or not(Here_now_R_accN);
	model.addClause(c);
	c = not(Here_now_R_accE)    or not(Here_now_R_mvN1);
	model.addClause(c);
	c = not(Here_now_R_accE)    or not(Here_now_R_mvN0);
	model.addClause(c);
	c = not(Here_now_R_accE)    or not(Here_now_R_accS);
	model.addClause(c);
	c = not(Here_now_R_accE)    or not(Here_now_R_mvS1);
	model.addClause(c);
	c = not(Here_now_R_accE)    or not(Here_now_R_mvS0);
	model.addClause(c);
	c = not(Here_now_R_mvE0)    or not(Here_now_R_accW);
	model.addClause(c);
	c = not(Here_now_R_mvE0)    or not(Here_now_R_mvW0);
	model.addClause(c);
	c = not(Here_now_R_mvE0)    or not(Here_now_R_accN);
	model.addClause(c);
	c = not(Here_now_R_mvE0)    or not(Here_now_R_mvN1);
	model.addClause(c);
	c = not(Here_now_R_mvE0)    or not(Here_now_R_mvN0);
	model.addClause(c);
	c = not(Here_now_R_mvE0)    or not(Here_now_R_accS);
	model.addClause(c);
	c = not(Here_now_R_mvE0)    or not(Here_now_R_mvS1);
	model.addClause(c);
	c = not(Here_now_R_mvE0)    or not(Here_now_R_mvS0);
	model.addClause(c);
	c = not(Here_now_R_accW)    or not(Here_now_R_mvW0);
	model.addClause(c);
	c = not(Here_now_R_accW)    or not(Here_now_R_accN);
	model.addClause(c);
	c = not(Here_now_R_accW)    or not(Here_now_R_mvN1);
	model.addClause(c);
	c = not(Here_now_R_accW)    or not(Here_now_R_mvN0);
	model.addClause(c);
	c = not(Here_now_R_accW)    or not(Here_now_R_accS);
	model.addClause(c);
	c = not(Here_now_R_accW)    or not(Here_now_R_mvS1);
	model.addClause(c);
	c = not(Here_now_R_accW)    or not(Here_now_R_mvS0);
	model.addClause(c);
	c = not(Here_now_R_mvW0)    or not(Here_now_R_accN);
	model.addClause(c);
	c = not(Here_now_R_mvW0)    or not(Here_now_R_mvN1);
	model.addClause(c);
	c = not(Here_now_R_mvW0)    or not(Here_now_R_mvN0);
	model.addClause(c);
	c = not(Here_now_R_mvW0)    or not(Here_now_R_accS);
	model.addClause(c);
	c = not(Here_now_R_mvW0)    or not(Here_now_R_mvS1);
	model.addClause(c);
	c = not(Here_now_R_mvW0)    or not(Here_now_R_mvS0);
	model.addClause(c);
	c = not(Here_now_R_accN)    or not(Here_now_R_mvN1);
	model.addClause(c);
	c = not(Here_now_R_accN)    or not(Here_now_R_mvN0);
	model.addClause(c);
	c = not(Here_now_R_accN)    or not(Here_now_R_accS);
	model.addClause(c);
	c = not(Here_now_R_accN)    or not(Here_now_R_mvS1);
	model.addClause(c);
	c = not(Here_now_R_accN)    or not(Here_now_R_mvS0);
	model.addClause(c);
	c = not(Here_now_R_mvN1)    or not(Here_now_R_mvN0);
	model.addClause(c);
	c = not(Here_now_R_mvN1)    or not(Here_now_R_accS);
	model.addClause(c);
	c = not(Here_now_R_mvN1)    or not(Here_now_R_mvS1);
	model.addClause(c);
	c = not(Here_now_R_mvN1)    or not(Here_now_R_mvS0);
	model.addClause(c);
	c = not(Here_now_R_mvN0)    or not(Here_now_R_accS);
	model.addClause(c);
	c = not(Here_now_R_mvN0)    or not(Here_now_R_mvS1);
	model.addClause(c);
	c = not(Here_now_R_mvN0)    or not(Here_now_R_mvS0);
	model.addClause(c);
	c = not(Here_now_R_accS)    or not(Here_now_R_mvS1);
	model.addClause(c);
	c = not(Here_now_R_accS)    or not(Here_now_R_mvS0);
	model.addClause(c);
	c = not(Here_now_R_mvS1)    or not(Here_now_R_mvS0);
	model.addClause(c);

	// Car 0
	
	const CNF::Var Here_now_C0R_accE = var(v,    t,     R_Move::w0_accE);
	const CNF::Var Here_now_C0R_mvE1 = var(v,    t,     R_Move::w0_mvE1);
	const CNF::Var Here_now_C0R_mvE0 = var(v,    t,     R_Move::w0_mvE0);
	const CNF::Var Here_now_C0R_accW = var(v,    t,     R_Move::w0_accW);
	const CNF::Var Here_now_C0R_mvW1 = var(v,    t,     R_Move::w0_mvW1);
	const CNF::Var Here_now_C0R_mvW0 = var(v,    t,     R_Move::w0_mvW0);
	const CNF::Var Here_now_C0R_accN = var(v,    t,     R_Move::w0_accN);
	const CNF::Var Here_now_C0R_mvN1 = var(v,    t,     R_Move::w0_mvN1);
	const CNF::Var Here_now_C0R_mvN2 = var(v,    t,     R_Move::w0_mvN2);
	const CNF::Var Here_now_C0R_mvN3 = var(v,    t,     R_Move::w0_mvN3);
	const CNF::Var Here_now_C0R_mvN0 = var(v,    t,     R_Move::w0_mvN0);
	const CNF::Var Here_now_C0R_accS = var(v,    t,     R_Move::w0_accS);
	const CNF::Var Here_now_C0R_mvS1 = var(v,    t,     R_Move::w0_mvS1);
	const CNF::Var Here_now_C0R_mvS2 = var(v,    t,     R_Move::w0_mvS2);
	const CNF::Var Here_now_C0R_mvS3 = var(v,    t,     R_Move::w0_mvS3);
	const CNF::Var Here_now_C0R_mvS0 = var(v,    t,     R_Move::w0_mvS0);

	//necceraly
	c = not(Here_now_C0R_moving)               or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	// sufficient
	// 16 choose 1
	c = Here_now_C0R_moving                    or                                                                                           not(Here_now_C0R_accE)    or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or not(Here_now_C0R_mvE1) or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or not(Here_now_C0R_mvE0) or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       not(Here_now_C0R_accW)    or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or not(Here_now_C0R_mvW1) or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or not(Here_now_C0R_mvW0) or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       not(Here_now_C0R_accN)    or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or not(Here_now_C0R_mvN1) or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or not(Here_now_C0R_mvN2) or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or not(Here_now_C0R_mvN3)                or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);

	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or not(Here_now_C0R_mvN0) or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0     or                                        Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0     or                                        Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2     or Here_now_C0R_mvN3                      or Here_now_C0R_mvN0      or                                                                                           not(Here_now_C0R_accS)    or Here_now_C0R_mvS1      or Here_now_C0R_mvS2     or Here_now_C0R_mvS3                      or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0     or                                        Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0     or                                        Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2     or Here_now_C0R_mvN3                      or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or not(Here_now_C0R_mvS1) or Here_now_C0R_mvS2     or Here_now_C0R_mvS3                      or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or not(Here_now_C0R_mvS2) or Here_now_C0R_mvS3                     or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or not(Here_now_C0R_mvS3)                or Here_now_C0R_mvS0;
	model.addClause(c);
	c = Here_now_C0R_moving                    or                                                                                           Here_now_C0R_accE         or Here_now_C0R_mvE1      or Here_now_C0R_mvE0      or                                       Here_now_C0R_accW         or Here_now_C0R_mvW1      or Here_now_C0R_mvW0      or                                       Here_now_C0R_accN         or Here_now_C0R_mvN1      or Here_now_C0R_mvN2      or Here_now_C0R_mvN3                     or Here_now_C0R_mvN0      or                                                                                           Here_now_C0R_accS         or Here_now_C0R_mvS1      or Here_now_C0R_mvS2      or Here_now_C0R_mvS3                     or not(Here_now_C0R_mvS0);
	model.addClause(c);
	// 16 choose 2
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvE1);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvE0);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_accW);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvW1);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvW0);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_accN);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvN1);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvN2);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_accE)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvE0);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_accW);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvW1);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvW0);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_accN);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvN1);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvN2);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_accW);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvW1);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvW0);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_accN);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvN1);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvN2);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvE0)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvW1);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvW0);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_accN);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvN1);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvN2);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_accW)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvW0);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_accN);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvN1);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvN2);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvW1)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_accN);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_mvN1);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_mvN2);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvW0)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_mvN1);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_mvN2);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_accN)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)                 or not(Here_now_C0R_mvN2);
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvN2)                 or not(Here_now_C0R_mvN3);
	model.addClause(c);
	c = not(Here_now_C0R_mvN2)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_mvN2)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_mvN2)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_mvN2)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvN2)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvN2)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvN3)                 or not(Here_now_C0R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_mvN3)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_mvN3)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_mvN3)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvN3)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvN3)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvN0)                 or not(Here_now_C0R_accS);
	model.addClause(c);
	c = not(Here_now_C0R_mvN0)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_mvN0)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvN0)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvN0)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_accS)                 or not(Here_now_C0R_mvS1);
	model.addClause(c);
	c = not(Here_now_C0R_accS)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_accS)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_accS)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvS1)                 or not(Here_now_C0R_mvS2);
	model.addClause(c);
	c = not(Here_now_C0R_mvS1)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvS1)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvS2)                 or not(Here_now_C0R_mvS3);
	model.addClause(c);
	c = not(Here_now_C0R_mvS2)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	c = not(Here_now_C0R_mvS3)                 or not(Here_now_C0R_mvS0);
	model.addClause(c);
	
        // Car 1

	const CNF::Var Here_now_C1R_accE = var(v,    t,     R_Move::w1_accE);
	const CNF::Var Here_now_C1R_mvE1 = var(v,    t,     R_Move::w1_mvE1);
	const CNF::Var Here_now_C1R_mvE0 = var(v,    t,     R_Move::w1_mvE0);
	const CNF::Var Here_now_C1R_accW = var(v,    t,     R_Move::w1_accW);
	const CNF::Var Here_now_C1R_mvW1 = var(v,    t,     R_Move::w1_mvW1);
	const CNF::Var Here_now_C1R_mvW0 = var(v,    t,     R_Move::w1_mvW0);
	const CNF::Var Here_now_C1R_accN = var(v,    t,     R_Move::w1_accN);
	const CNF::Var Here_now_C1R_mvN1 = var(v,    t,     R_Move::w1_mvN1);
	const CNF::Var Here_now_C1R_mvN2 = var(v,    t,     R_Move::w1_mvN2);
	const CNF::Var Here_now_C1R_mvN3 = var(v,    t,     R_Move::w1_mvN3);
	const CNF::Var Here_now_C1R_mvN0 = var(v,    t,     R_Move::w1_mvN0);
	const CNF::Var Here_now_C1R_accS = var(v,    t,     R_Move::w1_accS);
	const CNF::Var Here_now_C1R_mvS1 = var(v,    t,     R_Move::w1_mvS1);
	const CNF::Var Here_now_C1R_mvS2 = var(v,    t,     R_Move::w1_mvS2);
	const CNF::Var Here_now_C1R_mvS3 = var(v,    t,     R_Move::w1_mvS3);
	const CNF::Var Here_now_C1R_mvS0 = var(v,    t,     R_Move::w1_mvS0);

	//necceraly
	c = not(Here_now_C1R_moving)               or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	// sufficient
	// 16 choose 1
	c = Here_now_C1R_moving                    or                                                                                           not(Here_now_C1R_accE)    or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or not(Here_now_C1R_mvE1) or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or not(Here_now_C1R_mvE0)  or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      not(Here_now_C1R_accW)    or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or not(Here_now_C1R_mvW1) or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or not(Here_now_C1R_mvW0)  or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      not(Here_now_C1R_accN)    or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or not(Here_now_C1R_mvN1) or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or not(Here_now_C1R_mvN2)  or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or not(Here_now_C1R_mvN3)               or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or not(Here_now_C1R_mvN0) or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           not(Here_now_C1R_accS)    or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or not(Here_now_C1R_mvS1) or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or not(Here_now_C1R_mvS2)  or Here_now_C1R_mvS3                    or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or not(Here_now_C1R_mvS3)               or Here_now_C1R_mvS0;
	model.addClause(c);
	c = Here_now_C1R_moving                    or                                                                                           Here_now_C1R_accE         or Here_now_C1R_mvE1      or Here_now_C1R_mvE0       or                                      Here_now_C1R_accW         or Here_now_C1R_mvW1      or Here_now_C1R_mvW0       or                                      Here_now_C1R_accN         or Here_now_C1R_mvN1      or Here_now_C1R_mvN2       or Here_now_C1R_mvN3                    or Here_now_C1R_mvN0      or                                                                                           Here_now_C1R_accS         or Here_now_C1R_mvS1      or Here_now_C1R_mvS2       or Here_now_C1R_mvS3                    or not(Here_now_C1R_mvS0);
	model.addClause(c);
	// 16 choose 2
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvE1);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvE0);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_accW);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvW1);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvW0);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_accN);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvN1);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvN2);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_accE)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvE0);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_accW);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvW1);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvW0);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_accN);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvN1);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvN2);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvE1)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_accW);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvW1);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvW0);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_accN);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvN1);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvN2);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvS1);
	model.addClause(c); 
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvE0)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvW1);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvW0);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_accN);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvN1);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvN2);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_accW)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvW0);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_accN);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvN1);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvN2);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvW1)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_accN);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_mvN1);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_mvN2);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvW0)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_mvN1);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_mvN2);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_accN)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)                 or not(Here_now_C1R_mvN2);
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvN2)                 or not(Here_now_C1R_mvN3);
	model.addClause(c);
	c = not(Here_now_C1R_mvN2)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_mvN2)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_mvN2)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_mvN2)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvN2)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvN2)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvN3)                 or not(Here_now_C1R_mvN0);
	model.addClause(c);
	c = not(Here_now_C1R_mvN3)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_mvN3)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_mvN3)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvN3)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvN3)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvN0)                 or not(Here_now_C1R_accS);
	model.addClause(c);
	c = not(Here_now_C1R_mvN0)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_mvN0)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvN0)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvN0)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_accS)                 or not(Here_now_C1R_mvS1);
	model.addClause(c);
	c = not(Here_now_C1R_accS)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_accS)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_accS)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvS1)                 or not(Here_now_C1R_mvS2);
	model.addClause(c);
	c = not(Here_now_C1R_mvS1)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvS1)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvS2)                 or not(Here_now_C1R_mvS3);
	model.addClause(c);
	c = not(Here_now_C1R_mvS2)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
	c = not(Here_now_C1R_mvS3)                 or not(Here_now_C1R_mvS0);
	model.addClause(c);
        // Car 2

	const CNF::Var Here_now_C2R_accE = var(v,    t,     R_Move::w2_accE);
	const CNF::Var Here_now_C2R_mvE1 = var(v,    t,     R_Move::w2_mvE1);
	const CNF::Var Here_now_C2R_mvE0 = var(v,    t,     R_Move::w2_mvE0);
	const CNF::Var Here_now_C2R_accW = var(v,    t,     R_Move::w2_accW);
	const CNF::Var Here_now_C2R_mvW1 = var(v,    t,     R_Move::w2_mvW1);
	const CNF::Var Here_now_C2R_mvW0 = var(v,    t,     R_Move::w2_mvW0);
	const CNF::Var Here_now_C2R_accN = var(v,    t,     R_Move::w2_accN);
	const CNF::Var Here_now_C2R_mvN1 = var(v,    t,     R_Move::w2_mvN1);
	const CNF::Var Here_now_C2R_mvN2 = var(v,    t,     R_Move::w2_mvN2);
	const CNF::Var Here_now_C2R_mvN3 = var(v,    t,     R_Move::w2_mvN3);
	const CNF::Var Here_now_C2R_mvN0 = var(v,    t,     R_Move::w2_mvN0);
	const CNF::Var Here_now_C2R_accS = var(v,    t,     R_Move::w2_accS);
	const CNF::Var Here_now_C2R_mvS1 = var(v,    t,     R_Move::w2_mvS1);
	const CNF::Var Here_now_C2R_mvS2 = var(v,    t,     R_Move::w2_mvS2);
	const CNF::Var Here_now_C2R_mvS3 = var(v,    t,     R_Move::w2_mvS3);
	const CNF::Var Here_now_C2R_mvS0 = var(v,    t,     R_Move::w2_mvS0);

	//necceraly
	c = not(Here_now_C2R_moving)               or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	// sufficient
	// 16 choose 1
	c = Here_now_C2R_moving                    or                                                                                           not(Here_now_C2R_accE)    or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or not(Here_now_C2R_mvE1) or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or not(Here_now_C2R_mvE0) or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       not(Here_now_C0R_accW)    or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or not(Here_now_C2R_mvW1) or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or not(Here_now_C2R_mvW0) or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       not(Here_now_C2R_accN)    or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or not(Here_now_C2R_mvN1) or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or not(Here_now_C2R_mvN2) or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or not(Here_now_C2R_mvN3)                or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or not(Here_now_C2R_mvN0) or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           not(Here_now_C2R_accS)    or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or not(Here_now_C2R_mvS1) or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or not(Here_now_C2R_mvS2) or Here_now_C2R_mvS3                     or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving or                                                                                                              Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or not(Here_now_C2R_mvS3)                or Here_now_C2R_mvS0;
	model.addClause(c);
	c = Here_now_C2R_moving                    or                                                                                           Here_now_C2R_accE         or Here_now_C2R_mvE1      or Here_now_C2R_mvE0      or                                       Here_now_C2R_accW         or Here_now_C2R_mvW1      or Here_now_C2R_mvW0      or                                       Here_now_C2R_accN         or Here_now_C2R_mvN1      or Here_now_C2R_mvN2      or Here_now_C2R_mvN3                     or Here_now_C2R_mvN0      or                                                                                           Here_now_C2R_accS         or Here_now_C2R_mvS1      or Here_now_C2R_mvS2      or Here_now_C2R_mvS3                     or not(Here_now_C2R_mvS0);
	model.addClause(c);
	// 16 choose 2
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvE1);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvE0);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_accW);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvW1);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvW0);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_accN);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvN1);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvN2);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_accE)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvE0);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_accW);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvW1);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvW0);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_accN);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvN1);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvN2);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C0R_mvE1)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvE1)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_accW);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvW1);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvW0);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_accN);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvN1);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvN2);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvE0)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvW1);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvW0);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_accN);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvN1);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvN2);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_accW)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvW0);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_accN);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvN1);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvN2);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvW1)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_accN);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_mvN1);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_mvN2);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvW0)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_mvN1);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_mvN2);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_accN)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)                 or not(Here_now_C2R_mvN2);
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvN2)                 or not(Here_now_C2R_mvN3);
	model.addClause(c);
	c = not(Here_now_C2R_mvN2)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_mvN2)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_mvN2)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_mvN2)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvN2)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvN2)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvN3)                 or not(Here_now_C2R_mvN0);
	model.addClause(c);
	c = not(Here_now_C2R_mvN3)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_mvN3)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_mvN3)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvN3)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvN3)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvN0)                 or not(Here_now_C2R_accS);
	model.addClause(c);
	c = not(Here_now_C2R_mvN0)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_mvN0)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvN0)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvN0)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_accS)                 or not(Here_now_C2R_mvS1);
	model.addClause(c);
	c = not(Here_now_C2R_accS)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_accS)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_accS)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvS1)                 or not(Here_now_C2R_mvS2);
	model.addClause(c);
	c = not(Here_now_C2R_mvS1)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvS1)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvS2)                 or not(Here_now_C2R_mvS3);
	model.addClause(c);
	c = not(Here_now_C2R_mvS2)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);
	c = not(Here_now_C2R_mvS3)                 or not(Here_now_C2R_mvS0);
	model.addClause(c);

	model.dump(out);
    } // endof B A S I C

    // M O V E M E N T
    {
    	CNF::Clause c;
        //CNF::Var Here_now_nobodyhome= var(v,    t,     NdStat::nobodyhome);
	//CNF::Var Here_now_R_ready   = var(v,    t,     NdStat::R_ready);
	const CNF::Var Here_now_R_accE    = var(v,    t,     R_Move::accE);
	const CNF::Var Here_now_R_mvE0    = var(v,    t,     R_Move::mvE0);
	const CNF::Var Here_now_R_accW    = var(v,    t,     R_Move::accW);
	const CNF::Var Here_now_R_mvW0    = var(v,    t,     R_Move::mvW0);
	//CNF::Var Here_now_R_accN    = var(v,    t,     R_Move::accN);
	const CNF::Var Here_now_R_mvN1    = var(v,    t,     R_Move::mvN1);
	const CNF::Var Here_now_R_mvN0    = var(v,    t,     R_Move::mvN0);
	const CNF::Var Here_now_R_accS    = var(v,    t,     R_Move::accS);
	const CNF::Var Here_now_R_mvS1    = var(v,    t,     R_Move::mvS1);
	const CNF::Var Here_now_R_mvS0    = var(v,    t,     R_Move::mvS0);
	//CNF::Var Here_now_R_lift    = var(v,    t,     R_Vertical::lift);
	//CNF::Var Here_now_R_lifting1= var(v,    t,     R_Vertical::l1);
	//CNF::Var Here_now_R_lifting2= var(v,    t,     R_Vertical::l2);
	//CNF::Var Here_now_R_lifting3= var(v,    t,     R_Vertical::l3);
	//CNF::Var Here_now_R_lifting4= var(v,    t,     R_Vertical::l4);
	//CNF::Var Here_now_R_drop    = var(v,    t,     R_Vertical::drop);
	//CNF::Var Here_now_C0R_ready = var(v,    t,     NdStat::C0R_ready);
	const CNF::Var Here_now_C0R_accE  = var(v,    t,     R_Move::w0_accE);
	const CNF::Var Here_now_C0R_mvE1  = var(v,    t,     R_Move::w0_mvE1);
	const CNF::Var Here_now_C0R_mvE0  = var(v,    t,     R_Move::w0_mvE0);
	const CNF::Var Here_now_C0R_accW  = var(v,    t,     R_Move::w0_accW);
	const CNF::Var Here_now_C0R_mvW1  = var(v,    t,     R_Move::w0_mvW1);
	const CNF::Var Here_now_C0R_mvW0  = var(v,    t,     R_Move::w0_mvW0);
	const CNF::Var Here_now_C0R_accN  = var(v,    t,     R_Move::w0_accN);
	const CNF::Var Here_now_C0R_mvN1  = var(v,    t,     R_Move::w0_mvN1);
	const CNF::Var Here_now_C0R_mvN2  = var(v,    t,     R_Move::w0_mvN2);
	const CNF::Var Here_now_C0R_mvN3  = var(v,    t,     R_Move::w0_mvN3);
	const CNF::Var Here_now_C0R_mvN0  = var(v,    t,     R_Move::w0_mvN0);
	const CNF::Var Here_now_C0R_accS  = var(v,    t,     R_Move::w0_accS);
	const CNF::Var Here_now_C0R_mvS1  = var(v,    t,     R_Move::w0_mvS1);
	const CNF::Var Here_now_C0R_mvS2  = var(v,    t,     R_Move::w0_mvS2);
	const CNF::Var Here_now_C0R_mvS3  = var(v,    t,     R_Move::w0_mvS3);
	const CNF::Var Here_now_C0R_mvS0  = var(v,    t,     R_Move::w0_mvS0);
	//CNF::Var Here_now_C1R_ready = var(v,    t,     NdStat::C1R_ready);
	const CNF::Var Here_now_C1R_accE  = var(v,    t,     R_Move::w1_accE);
	const CNF::Var Here_now_C1R_mvE1  = var(v,    t,     R_Move::w1_mvE1);
	const CNF::Var Here_now_C1R_mvE0  = var(v,    t,     R_Move::w1_mvE0);
	const CNF::Var Here_now_C1R_accW  = var(v,    t,     R_Move::w1_accW);
	const CNF::Var Here_now_C1R_mvW1  = var(v,    t,     R_Move::w1_mvW1);
	const CNF::Var Here_now_C1R_mvW0  = var(v,    t,     R_Move::w1_mvW0);
	const CNF::Var Here_now_C1R_accN  = var(v,    t,     R_Move::w1_accN);
	const CNF::Var Here_now_C1R_mvN1  = var(v,    t,     R_Move::w1_mvN1);
	const CNF::Var Here_now_C1R_mvN2  = var(v,    t,     R_Move::w1_mvN2);
	const CNF::Var Here_now_C1R_mvN3  = var(v,    t,     R_Move::w1_mvN3);
	const CNF::Var Here_now_C1R_mvN0  = var(v,    t,     R_Move::w1_mvN0);
	const CNF::Var Here_now_C1R_accS  = var(v,    t,     R_Move::w1_accS);
	const CNF::Var Here_now_C1R_mvS1  = var(v,    t,     R_Move::w1_mvS1);
	const CNF::Var Here_now_C1R_mvS2  = var(v,    t,     R_Move::w1_mvS2);
	const CNF::Var Here_now_C1R_mvS3  = var(v,    t,     R_Move::w1_mvS3);
	const CNF::Var Here_now_C1R_mvS0  = var(v,    t,     R_Move::w1_mvS0);
	//const CNF::Var Here_now_C2R_ready = var(v,    t,     NdStat::C2R_ready);
	const CNF::Var Here_now_C2R_accE  = var(v,    t,     R_Move::w2_accE);
	const CNF::Var Here_now_C2R_mvE1  = var(v,    t,     R_Move::w2_mvE1);
	const CNF::Var Here_now_C2R_mvE0  = var(v,    t,     R_Move::w2_mvE0);
	const CNF::Var Here_now_C2R_accW  = var(v,    t,     R_Move::w2_accW);
	const CNF::Var Here_now_C2R_mvW1  = var(v,    t,     R_Move::w2_mvW1);
	const CNF::Var Here_now_C2R_mvW0  = var(v,    t,     R_Move::w2_mvW0);
	const CNF::Var Here_now_C2R_accN  = var(v,    t,     R_Move::w2_accN);
	const CNF::Var Here_now_C2R_mvN1  = var(v,    t,     R_Move::w2_mvN1);
	const CNF::Var Here_now_C2R_mvN2  = var(v,    t,     R_Move::w2_mvN2);
	const CNF::Var Here_now_C2R_mvN3  = var(v,    t,     R_Move::w2_mvN3);
	const CNF::Var Here_now_C2R_mvN0  = var(v,    t,     R_Move::w2_mvN0);
	const CNF::Var Here_now_C2R_accS  = var(v,    t,     R_Move::w2_accS);
	const CNF::Var Here_now_C2R_mvS1  = var(v,    t,     R_Move::w2_mvS1);
	const CNF::Var Here_now_C2R_mvS2  = var(v,    t,     R_Move::w2_mvS2);
	const CNF::Var Here_now_C2R_mvS3  = var(v,    t,     R_Move::w2_mvS3);
	const CNF::Var Here_now_C2R_mvS0  = var(v,    t,     R_Move::w2_mvS0);

       

	const CNF::Var E_now_nobodyhome= var( G.east(v),    t,    NdStat::nobodyhome);
	//CNF::Var E_now_R_ready   = var( G.east(v),    t,    NdStat::R_ready);
	const CNF::Var E_now_R_accE    = var( G.east(v),    t,    R_Move::accE);
	const CNF::Var E_now_R_mvE0    = var( G.east(v),    t,    R_Move::mvE0);
	const CNF::Var E_now_R_accW    = var( G.east(v),    t,    R_Move::accW);
	const CNF::Var E_now_R_mvW0    = var( G.east(v),    t,    R_Move::mvW0);
	//const CNF::Var E_now_C0R_ready = var( G.east(v),    t,    NdStat::C0R_ready);
	const CNF::Var E_now_C0R_accE  = var( G.east(v),    t,    R_Move::w0_accE);
	const CNF::Var E_now_C0R_mvE1  = var( G.east(v),    t,    R_Move::w0_mvE1);
	const CNF::Var E_now_C0R_mvE0  = var( G.east(v),    t,    R_Move::w0_mvE0);
	const CNF::Var E_now_C0R_accW  = var( G.east(v),    t,    R_Move::w0_accW);
	const CNF::Var E_now_C0R_mvW1  = var( G.east(v),    t,    R_Move::w0_mvW1);
	const CNF::Var E_now_C0R_mvW0  = var( G.east(v),    t,    R_Move::w0_mvW0);
	//const CNF::Var E_now_C1R_ready = var( G.east(v),    t,    NdStat::C1R_ready);
	const CNF::Var E_now_C1R_accE  = var( G.east(v),    t,    R_Move::w1_accE);
	const CNF::Var E_now_C1R_mvE1  = var( G.east(v),    t,    R_Move::w1_mvE1);
	const CNF::Var E_now_C1R_mvE0  = var( G.east(v),    t,    R_Move::w1_mvE0);
	const CNF::Var E_now_C1R_accW  = var( G.east(v),    t,    R_Move::w1_accW);
	const CNF::Var E_now_C1R_mvW1  = var( G.east(v),    t,    R_Move::w1_mvW1);
	const CNF::Var E_now_C1R_mvW0  = var( G.east(v),    t,    R_Move::w1_mvW0);
	//const CNF::Var E_now_C2R_ready = var( G.east(v),    t,    NdStat::C2R_ready);
	const CNF::Var E_now_C2R_accE  = var( G.east(v),    t,    R_Move::w2_accE);
	const CNF::Var E_now_C2R_mvE1  = var( G.east(v),    t,    R_Move::w2_mvE1);
	const CNF::Var E_now_C2R_mvE0  = var( G.east(v),    t,    R_Move::w2_mvE0);
	const CNF::Var E_now_C2R_accW  = var( G.east(v),    t,    R_Move::w2_accW);
	const CNF::Var E_now_C2R_mvW1  = var( G.east(v),    t,    R_Move::w2_mvW1);
	const CNF::Var E_now_C2R_mvW0  = var( G.east(v),    t,    R_Move::w2_mvW0);

       

	const CNF::Var N_now_nobodyhome= var( G.north(v),    t,    NdStat::nobodyhome);
	//const CNF::Var N_now_R_ready   = var( G.north(v),    t,    NdStat::R_ready);
	const CNF::Var N_now_R_accN    = var( G.north(v),    t,    R_Move::accN);
	const CNF::Var N_now_R_mvN1    = var( G.north(v),    t,    R_Move::mvN1);
	const CNF::Var N_now_R_mvN0    = var( G.north(v),    t,    R_Move::mvN0);
	const CNF::Var N_now_R_accS    = var( G.north(v),    t,    R_Move::accS);
	const CNF::Var N_now_R_mvS1    = var( G.north(v),    t,    R_Move::mvS1);
	const CNF::Var N_now_R_mvS0    = var( G.north(v),    t,    R_Move::mvS0);
	//const CNF::Var N_now_C0R_ready = var( G.north(v),    t,    NdStat::C0R_ready);
	const CNF::Var N_now_C0R_accN  = var( G.north(v),    t,    R_Move::w0_accN);
	const CNF::Var N_now_C0R_mvN1  = var( G.north(v),    t,    R_Move::w0_mvN1);
	const CNF::Var N_now_C0R_mvN2  = var( G.north(v),    t,    R_Move::w0_mvN2);
	const CNF::Var N_now_C0R_mvN3  = var( G.north(v),    t,    R_Move::w0_mvN3);
	const CNF::Var N_now_C0R_mvN0  = var( G.north(v),    t,    R_Move::w0_mvN0);
	const CNF::Var N_now_C0R_accS  = var( G.north(v),    t,    R_Move::w0_accS);
	const CNF::Var N_now_C0R_mvS1  = var( G.north(v),    t,    R_Move::w0_mvS1);
	const CNF::Var N_now_C0R_mvS2  = var( G.north(v),    t,    R_Move::w0_mvS2);
	const CNF::Var N_now_C0R_mvS3  = var( G.north(v),    t,    R_Move::w0_mvS3);
	const CNF::Var N_now_C0R_mvS0  = var( G.north(v),    t,    R_Move::w0_mvS0);
	//const CNF::Var N_now_C1R_ready = var( G.north(v),    t,    NdStat::C1R_ready);
	const CNF::Var N_now_C1R_accN  = var( G.north(v),    t,    R_Move::w1_accN);
	const CNF::Var N_now_C1R_mvN1  = var( G.north(v),    t,    R_Move::w1_mvN1);
	const CNF::Var N_now_C1R_mvN2  = var( G.north(v),    t,    R_Move::w1_mvN2);
	const CNF::Var N_now_C1R_mvN3  = var( G.north(v),    t,    R_Move::w1_mvN3);
	const CNF::Var N_now_C1R_mvN0  = var( G.north(v),    t,    R_Move::w1_mvN0);
	const CNF::Var N_now_C1R_accS  = var( G.north(v),    t,    R_Move::w1_accS);
	const CNF::Var N_now_C1R_mvS1  = var( G.north(v),    t,    R_Move::w1_mvS1);
	const CNF::Var N_now_C1R_mvS2  = var( G.north(v),    t,    R_Move::w1_mvS2);
	const CNF::Var N_now_C1R_mvS3  = var( G.north(v),    t,    R_Move::w1_mvS3);
	const CNF::Var N_now_C1R_mvS0  = var( G.north(v),    t,    R_Move::w1_mvS0);
	//const CNF::Var N_now_C2R_ready = var( G.north(v),    t,    NdStat::C2R_ready);
	const CNF::Var N_now_C2R_accN  = var( G.north(v),    t,    R_Move::w2_accN);
	const CNF::Var N_now_C2R_mvN1  = var( G.north(v),    t,    R_Move::w2_mvN1);
	const CNF::Var N_now_C2R_mvN2  = var( G.north(v),    t,    R_Move::w2_mvN2);
	const CNF::Var N_now_C2R_mvN3  = var( G.north(v),    t,    R_Move::w2_mvN3);
	const CNF::Var N_now_C2R_mvN0  = var( G.north(v),    t,    R_Move::w2_mvN0);
	const CNF::Var N_now_C2R_accS  = var( G.north(v),    t,    R_Move::w2_accS);
	const CNF::Var N_now_C2R_mvS1  = var( G.north(v),    t,    R_Move::w2_mvS1);
	const CNF::Var N_now_C2R_mvS2  = var( G.north(v),    t,    R_Move::w2_mvS2);
	const CNF::Var N_now_C2R_mvS3  = var( G.north(v),    t,    R_Move::w2_mvS3);
	const CNF::Var N_now_C2R_mvS0  = var( G.north(v),    t,    R_Move::w2_mvS0);

       

	const CNF::Var W_now_nobodyhome= var( G.west(v),    t,    NdStat::nobodyhome);
	//const CNF::Var W_now_R_ready   = var( G.west(v),    t,    NdStat::R_ready);
	const CNF::Var W_now_R_accE    = var( G.west(v),    t,    R_Move::accE);
	const CNF::Var W_now_R_mvE0    = var( G.west(v),    t,    R_Move::mvE0);
	const CNF::Var W_now_R_accW    = var( G.west(v),    t,    R_Move::accW);
	const CNF::Var W_now_R_mvW0    = var( G.west(v),    t,    R_Move::mvW0);
	//const CNF::Var W_now_C0R_ready = var( G.west(v),    t,    NdStat::C0R_ready);
	const CNF::Var W_now_C0R_accE  = var( G.west(v),    t,    R_Move::w0_accE);
	const CNF::Var W_now_C0R_mvE1  = var( G.west(v),    t,    R_Move::w0_mvE1);
	const CNF::Var W_now_C0R_mvE0  = var( G.west(v),    t,    R_Move::w0_mvE0);
	const CNF::Var W_now_C0R_accW  = var( G.west(v),    t,    R_Move::w0_accW);
	const CNF::Var W_now_C0R_mvW1  = var( G.west(v),    t,    R_Move::w0_mvW1);
	const CNF::Var W_now_C0R_mvW0  = var( G.west(v),    t,    R_Move::w0_mvW0);
	//const CNF::Var W_now_C1R_ready = var( G.west(v),    t,    NdStat::C1R_ready);
	const CNF::Var W_now_C1R_accE  = var( G.west(v),    t,    R_Move::w1_accE);
	const CNF::Var W_now_C1R_mvE1  = var( G.west(v),    t,    R_Move::w1_mvE1);
	const CNF::Var W_now_C1R_mvE0  = var( G.west(v),    t,    R_Move::w1_mvE0);
	const CNF::Var W_now_C1R_accW  = var( G.west(v),    t,    R_Move::w1_accW);
	const CNF::Var W_now_C1R_mvW1  = var( G.west(v),    t,    R_Move::w1_mvW1);
	const CNF::Var W_now_C1R_mvW0  = var( G.west(v),    t,    R_Move::w1_mvW0);
	//const CNF::Var W_now_C2R_ready = var( G.west(v),    t,    NdStat::C2R_ready);
	const CNF::Var W_now_C2R_accE  = var( G.west(v),    t,    R_Move::w2_accE);
	const CNF::Var W_now_C2R_mvE1  = var( G.west(v),    t,    R_Move::w2_mvE1);
	const CNF::Var W_now_C2R_mvE0  = var( G.west(v),    t,    R_Move::w2_mvE0);
	const CNF::Var W_now_C2R_accW  = var( G.west(v),    t,    R_Move::w2_accW);
	const CNF::Var W_now_C2R_mvW1  = var( G.west(v),    t,    R_Move::w2_mvW1);
	const CNF::Var W_now_C2R_mvW0  = var( G.west(v),    t,    R_Move::w2_mvW0);


	const CNF::Var S_now_nobodyhome= var( G.south(v),    t,    NdStat::nobodyhome);
	//const CNF::Var S_now_R_ready   = var( G.south(v),    t,    NdStat::R_ready);
	const CNF::Var S_now_R_accN    = var( G.south(v),    t,    R_Move::accN);
	const CNF::Var S_now_R_mvN1    = var( G.south(v),    t,    R_Move::mvN1);
	const CNF::Var S_now_R_mvN0    = var( G.south(v),    t,    R_Move::mvN0);
	const CNF::Var S_now_R_accS    = var( G.south(v),    t,    R_Move::accS);
	const CNF::Var S_now_R_mvS1    = var( G.south(v),    t,    R_Move::mvS1);
	const CNF::Var S_now_R_mvS0    = var( G.south(v),    t,    R_Move::mvS0);
	//const CNF::Var S_now_C0R_ready = var( G.south(v),    t,    NdStat::C0R_ready);
	const CNF::Var S_now_C0R_accN  = var( G.south(v),    t,    R_Move::w0_accN);
	const CNF::Var S_now_C0R_mvN1  = var( G.south(v),    t,    R_Move::w0_mvN1);
	const CNF::Var S_now_C0R_mvN2  = var( G.south(v),    t,    R_Move::w0_mvN2);
	const CNF::Var S_now_C0R_mvN3  = var( G.south(v),    t,    R_Move::w0_mvN3);
	const CNF::Var S_now_C0R_mvN0  = var( G.south(v),    t,    R_Move::w0_mvN0);
	const CNF::Var S_now_C0R_accS  = var( G.south(v),    t,    R_Move::w0_accS);
	const CNF::Var S_now_C0R_mvS1  = var( G.south(v),    t,    R_Move::w0_mvS1);
	const CNF::Var S_now_C0R_mvS2  = var( G.south(v),    t,    R_Move::w0_mvS2);
	const CNF::Var S_now_C0R_mvS3  = var( G.south(v),    t,    R_Move::w0_mvS3);
	const CNF::Var S_now_C0R_mvS0  = var( G.south(v),    t,    R_Move::w0_mvS0);
	//const CNF::Var S_now_C1R_ready = var( G.south(v),    t,    NdStat::C1R_ready);
	const CNF::Var S_now_C1R_accN  = var( G.south(v),    t,    R_Move::w1_accN);
	const CNF::Var S_now_C1R_mvN1  = var( G.south(v),    t,    R_Move::w1_mvN1);
	const CNF::Var S_now_C1R_mvN2  = var( G.south(v),    t,    R_Move::w1_mvN2);
	const CNF::Var S_now_C1R_mvN3  = var( G.south(v),    t,    R_Move::w1_mvN3);
	const CNF::Var S_now_C1R_mvN0  = var( G.south(v),    t,    R_Move::w1_mvN0);
	const CNF::Var S_now_C1R_accS  = var( G.south(v),    t,    R_Move::w1_accS);
	const CNF::Var S_now_C1R_mvS1  = var( G.south(v),    t,    R_Move::w1_mvS1);
	const CNF::Var S_now_C1R_mvS2  = var( G.south(v),    t,    R_Move::w1_mvS2);
	const CNF::Var S_now_C1R_mvS3  = var( G.south(v),    t,    R_Move::w1_mvS3);
	const CNF::Var S_now_C1R_mvS0  = var( G.south(v),    t,    R_Move::w1_mvS0);
	//const CNF::Var S_now_C2R_ready = var( G.south(v),    t,    NdStat::C2R_ready);
	const CNF::Var S_now_C2R_accN  = var( G.south(v),    t,    R_Move::w2_accN);
	const CNF::Var S_now_C2R_mvN1  = var( G.south(v),    t,    R_Move::w2_mvN1);
	const CNF::Var S_now_C2R_mvN2  = var( G.south(v),    t,    R_Move::w2_mvN2);
	const CNF::Var S_now_C2R_mvN3  = var( G.south(v),    t,    R_Move::w2_mvN3);
	const CNF::Var S_now_C2R_mvN0  = var( G.south(v),    t,    R_Move::w2_mvN0);
	const CNF::Var S_now_C2R_accS  = var( G.south(v),    t,    R_Move::w2_accS);
	const CNF::Var S_now_C2R_mvS1  = var( G.south(v),    t,    R_Move::w2_mvS1);
	const CNF::Var S_now_C2R_mvS2  = var( G.south(v),    t,    R_Move::w2_mvS2);
	const CNF::Var S_now_C2R_mvS3  = var( G.south(v),    t,    R_Move::w2_mvS3);
	const CNF::Var S_now_C2R_mvS0  = var( G.south(v),    t,    R_Move::w2_mvS0);



        // At most one robot moving towards this node:
	c =  E_now_R_accW   or E_now_R_mvW0                                                                                      or W_now_R_accE   or W_now_R_mvE0                                                                                      or N_now_R_accS   or N_now_R_mvS0   or N_now_R_mvS1                                                                    or S_now_R_accN   or S_now_R_mvN0   or S_now_R_mvN1                                                                    or E_now_C0R_accW or E_now_C1R_accW or E_now_C2R_accW                                                                  or E_now_C0R_mvW0 or E_now_C1R_mvW0 or E_now_C2R_mvW0                                                                  or E_now_C0R_mvW1 or E_now_C1R_mvW1 or E_now_C2R_mvW1                                                                  or W_now_C0R_accE or W_now_C1R_accE or W_now_C2R_accE                                                                  or W_now_C0R_mvE0 or W_now_C1R_mvE0 or W_now_C2R_mvE0                                                                  or W_now_C0R_mvE1 or W_now_C1R_mvE1 or W_now_C2R_mvE1                                                                  or N_now_C0R_accS or N_now_C1R_accS or N_now_C2R_accS                                                                  or N_now_C0R_mvS0 or N_now_C1R_mvS0 or N_now_C2R_mvS0                                                                  or N_now_C0R_mvS1 or N_now_C1R_mvS1 or N_now_C2R_mvS1                                                                  or N_now_C0R_mvS2 or N_now_C1R_mvS2 or N_now_C2R_mvS2                                                                  or N_now_C0R_mvS3 or N_now_C1R_mvS3 or N_now_C2R_mvS3                                                                  or S_now_C0R_accN or S_now_C1R_accN or S_now_C2R_accN                                                                  or S_now_C0R_mvN0 or S_now_C1R_mvN0 or S_now_C2R_mvN0                                                                  or S_now_C0R_mvN1 or S_now_C1R_mvN1 or S_now_C2R_mvN1                                                                  or S_now_C0R_mvN2 or S_now_C1R_mvN2 or S_now_C2R_mvN2                                                                  or S_now_C0R_mvN3 or S_now_C1R_mvN3 or S_now_C2R_mvN3;
	model.addClause(c);
	// 58 choose 2
	c = not(E_now_R_accW) or not(E_now_R_mvW0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(W_now_R_accE);
	model.addClause(c);
	c = not(E_now_R_accW) or not(W_now_R_mvE0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_R_accS);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_R_mvS0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_R_mvS1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_R_accN);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_R_mvN0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(E_now_R_accW) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(E_now_R_accW) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(E_now_R_accW) or not(E_now_C2R_accW);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_R_accW) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_R_accW) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_R_accW) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_R_accW) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_R_accW) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(E_now_R_mvW0) or not(W_now_R_accE);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_R_mvE0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_R_accS);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_R_mvS0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_R_mvS1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_R_accN);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_R_mvN0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(E_now_R_mvW0) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_C2R_accW);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_C2R_mvS3);
	model.addClause(c);

	c = not(E_now_R_mvW0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_R_mvW0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(W_now_R_accE) or not(W_now_R_mvE0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_R_accS);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_R_mvS0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_R_mvS1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_R_accN);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_R_mvN0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(W_now_R_accE) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(W_now_R_accE) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(W_now_R_accE) or not(E_now_C2R_accW);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(W_now_R_accE) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(W_now_R_accE) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_R_accE) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_R_accE) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_R_accE) or not(S_now_C2R_mvN3);
	model.addClause(c);

	
	c = not(W_now_R_mvE0) or not(N_now_R_accS);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_R_mvS0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_R_mvS1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_R_accN);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_R_mvN0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(W_now_R_mvE0) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_C2R_accW);
	model.addClause(c);
       
	c = not(W_now_R_mvE0) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_R_mvE0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(N_now_R_accS) or not(N_now_R_mvS0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_R_mvS1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_R_accN);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_R_mvN0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(N_now_R_accS) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(N_now_R_accS) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(N_now_R_accS) or not(E_now_C2R_accW);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(N_now_R_accS) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(N_now_R_accS) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_R_accS) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_R_accS) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_R_accS) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(N_now_R_mvS0) or not(N_now_R_mvS1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_R_accN);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_R_mvN0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(N_now_R_mvS0) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_C2R_accW);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_R_mvS0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(N_now_R_mvS1) or not(S_now_R_accN);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_R_mvN0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(N_now_R_mvS1) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_C2R_accW);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_R_mvS1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(S_now_R_accN) or not(S_now_R_mvN0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(S_now_R_accN) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(S_now_R_accN) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(S_now_R_accN) or not(E_now_C2R_accW);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(S_now_R_accN) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(S_now_R_accN) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(S_now_R_accN) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(S_now_R_accN) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(S_now_R_accN) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(S_now_R_accN) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(S_now_R_accN) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_R_accN) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_R_accN) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(S_now_R_mvN0) or not(S_now_R_mvN1);
	model.addClause(c);

	c = not(S_now_R_mvN0) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_C2R_accW);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_R_mvN0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(S_now_R_mvN1) or not(E_now_C0R_accW);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_C2R_accW);
	model.addClause(c);	
	
	c = not(S_now_R_mvN1) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_R_mvN1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	
	
	c = not(E_now_C0R_accW) or not(E_now_C1R_accW);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C2R_accW);
	model.addClause(c);

	c = not(E_now_C0R_accW) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C0R_accW) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(E_now_C1R_accW) or not(E_now_C2R_accW);
	model.addClause(c);

	c = not(E_now_C1R_accW) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C1R_accW) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C1R_accW) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(E_now_C2R_accW) or not(E_now_C0R_mvW0);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(E_now_C0R_accW) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C2R_accW) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C2R_accW) or not(S_now_C2R_mvN3);
	model.addClause(c);	
	
	
	
	c = not(E_now_C0R_mvW0) or not(E_now_C1R_mvW0);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C0R_mvW0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(E_now_C1R_mvW0) or not(E_now_C2R_mvW0);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C1R_mvW0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(E_now_C2R_mvW0) or not(E_now_C0R_mvW1);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C2R_mvW0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	
	c = not(E_now_C0R_mvW1) or not(E_now_C1R_mvW1);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C0R_mvW1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C0R_mvW1) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(E_now_C1R_mvW1) or not(E_now_C2R_mvW1);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C1R_mvW1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C1R_mvW1) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(E_now_C2R_mvW1) or not(W_now_C0R_accE);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(E_now_C2R_mvW1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(E_now_C2R_mvW1) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	
	
	c = not(W_now_C0R_accE) or not(W_now_C1R_accE);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C2R_mvN1);
	model.addClause(c);	

	c = not(W_now_C0R_accE) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C0R_accE) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C0R_accE) or not(S_now_C2R_mvN3);
	model.addClause(c);	
	
	
	c = not(W_now_C1R_accE) or not(W_now_C2R_accE);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C1R_accE) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C1R_accE) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(W_now_C2R_accE) or not(W_now_C0R_mvE0);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C2R_accE) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C2R_accE) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	
	c = not(W_now_C0R_mvE0) or not(W_now_C1R_mvE0);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C0R_mvE0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(W_now_C1R_mvE0) or not(W_now_C2R_mvE0);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C1R_mvE0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(W_now_C2R_mvE0) or not(W_now_C0R_mvE1);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C2R_mvE0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	
	c = not(W_now_C0R_mvE1) or not(W_now_C1R_mvE1);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C2R_mvN1);
	model.addClause(c);	

	c = not(W_now_C0R_mvE1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C0R_mvE1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C0R_mvE1) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(W_now_C1R_mvE1) or not(W_now_C2R_mvE1);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C1R_mvE1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C1R_mvE1) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(W_now_C2R_mvE1) or not(N_now_C0R_accS);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(W_now_C2R_mvE1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(W_now_C2R_mvE1) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	
	
	c = not(N_now_C0R_accS) or not(N_now_C1R_accS);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C0R_accS) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_accS) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(N_now_C1R_accS) or not(N_now_C2R_accS);
	model.addClause(c);
	
	c = not(N_now_C1R_accS) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(N_now_C1R_accS) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_C1R_accS) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C1R_accS) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C1R_accS) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C1R_accS) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C1R_accS) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C2R_mvN1);
	model.addClause(c);       

	c = not(N_now_C1R_accS) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C1R_accS) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_accS) or not(S_now_C2R_mvN3);
	model.addClause(c);

	
	c = not(N_now_C2R_accS) or not(N_now_C0R_mvS0);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(N_now_C2R_accS) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_C2R_accS) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C2R_accS) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C2R_accS) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C2R_accS) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C2R_accS) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C2R_accS) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C2R_accS) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_accS) or not(S_now_C2R_mvN3);
	model.addClause(c);

	

	c = not(N_now_C0R_mvS0) or not(N_now_C1R_mvS0);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C2R_mvN1);
	model.addClause(c);	

	c = not(N_now_C0R_mvS0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_mvS0) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(N_now_C1R_mvS0) or not(N_now_C2R_mvS0);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C2R_mvN1);
	model.addClause(c);	

	c = not(N_now_C1R_mvS0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_mvS0) or not(S_now_C2R_mvN3);
	model.addClause(c);
	
	
	c = not(N_now_C2R_mvS0) or not(N_now_C0R_mvS1);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS0) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS0) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS0) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS0) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_mvS0) or not(S_now_C2R_mvN3);
	model.addClause(c);


	
	c = not(N_now_C0R_mvS1) or not(N_now_C1R_mvS1);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_mvS1) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(N_now_C1R_mvS1) or not(N_now_C2R_mvS1);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_mvS1) or not(S_now_C2R_mvN3);
	model.addClause(c);

			
	c = not(N_now_C2R_mvS1) or not(N_now_C0R_mvS2);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS1) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS1) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS1) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS1) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_mvS1) or not(S_now_C2R_mvN3);
	model.addClause(c);


	
	c = not(N_now_C0R_mvS2) or not(N_now_C1R_mvS2);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS2) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS2) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS2) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS2) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS2) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS2) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_mvS2) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(N_now_C1R_mvS2) or not(N_now_C2R_mvS2);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS2) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS2) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS2) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS2) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C2R_mvN1);
	model.addClause(c);	

	c = not(N_now_C1R_mvS2) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS2) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_mvS2) or not(S_now_C2R_mvN3);
	model.addClause(c);

	
	c = not(N_now_C2R_mvS2) or not(N_now_C0R_mvS3);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS2) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS2) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS2) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS2) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS2) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_mvS2) or not(S_now_C2R_mvN3);
	model.addClause(c);


	
	c = not(N_now_C0R_mvS3) or not(N_now_C1R_mvS3);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS3) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS3) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS3) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS3) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C0R_mvS3) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C0R_mvS3) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(N_now_C1R_mvS3) or not(N_now_C2R_mvS3);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS3) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS3) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS3) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS3) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C1R_mvS3) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C1R_mvS3) or not(S_now_C2R_mvN3);
	model.addClause(c);

	
	c = not(N_now_C2R_mvS3) or not(S_now_C0R_accN);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS3) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS3) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C2R_mvN1);
	model.addClause(c);

	c = not(N_now_C2R_mvS3) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(N_now_C2R_mvS3) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(N_now_C2R_mvS3) or not(S_now_C2R_mvN3);
	model.addClause(c);





	c = not(S_now_C0R_accN) or not(S_now_C1R_accN);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(S_now_C0R_accN) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(S_now_C0R_accN) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C2R_mvN1);
	model.addClause(c);

	c = not(S_now_C0R_accN) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C0R_accN) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(S_now_C1R_accN) or not(S_now_C2R_accN);
	model.addClause(c);
	
	c = not(S_now_C1R_accN) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(S_now_C1R_accN) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(S_now_C1R_accN) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C1R_accN) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C2R_mvN3);
	model.addClause(c);

	
	c = not(S_now_C2R_accN) or not(S_now_C0R_mvN0);
	model.addClause(c);
	c = not(S_now_C2R_accN) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(S_now_C2R_accN) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(S_now_C2R_accN) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_C2R_accN) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_C2R_accN) or not(S_now_C2R_mvN1);
	model.addClause(c);

	c = not(S_now_C2R_accN) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C2R_accN) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C2R_accN) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C2R_accN) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C2R_accN) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C2R_accN) or not(S_now_C2R_mvN3);
	model.addClause(c);

	

	c = not(S_now_C0R_mvN0) or not(S_now_C1R_mvN0);
	model.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(S_now_C0R_mvN0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C2R_mvN1);
	model.addClause(c);	

	c = not(S_now_C0R_mvN0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C0R_mvN0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(S_now_C1R_mvN0) or not(S_now_C2R_mvN0);
	model.addClause(c);
	
	c = not(S_now_C1R_mvN0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_C1R_mvN0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_C1R_mvN0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(S_now_C1R_mvN0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C1R_mvN0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C1R_mvN0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C1R_mvN0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C1R_mvN0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C1R_mvN0) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(S_now_C2R_mvN0) or not(S_now_C0R_mvN1);
	model.addClause(c);
	c = not(S_now_C2R_mvN0) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_C2R_mvN0) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(S_now_C2R_mvN0) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C2R_mvN0) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C2R_mvN0) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C2R_mvN0) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C2R_mvN0) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C2R_mvN0) or not(S_now_C2R_mvN3);
	model.addClause(c);

	

	c = not(S_now_C0R_mvN1) or not(S_now_C1R_mvN1);
	model.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(S_now_C0R_mvN1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C0R_mvN1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(S_now_C1R_mvN1) or not(S_now_C2R_mvN1);
	model.addClause(c);
	
	c = not(S_now_C1R_mvN1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C1R_mvN1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C1R_mvN1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C1R_mvN1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C1R_mvN1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C1R_mvN1) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(S_now_C2R_mvN1) or not(S_now_C0R_mvN2);
	model.addClause(c);
	c = not(S_now_C2R_mvN1) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C2R_mvN1) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C2R_mvN1) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C2R_mvN1) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C2R_mvN1) or not(S_now_C2R_mvN3);
	model.addClause(c);



	c = not(S_now_C0R_mvN2) or not(S_now_C1R_mvN2);
	model.addClause(c);
	c = not(S_now_C0R_mvN2) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C0R_mvN2) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_mvN2) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_mvN2) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(S_now_C1R_mvN2) or not(S_now_C2R_mvN2);
	model.addClause(c);
	
	c = not(S_now_C1R_mvN2) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C1R_mvN2) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C1R_mvN2) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(S_now_C2R_mvN2) or not(S_now_C0R_mvN3);
	model.addClause(c);
	c = not(S_now_C2R_mvN2) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C2R_mvN2) or not(S_now_C2R_mvN3);
	model.addClause(c);


	
	c = not(S_now_C0R_mvN3) or not(S_now_C1R_mvN3);
	model.addClause(c);
	c = not(S_now_C0R_mvN3) or not(S_now_C2R_mvN3);
	model.addClause(c);


	c = not(S_now_C1R_mvN3) or not(S_now_C2R_mvN3);
	model.addClause(c);



	

	c = not(Here_now_R_mvN0)  or N_now_nobodyhome or N_now_R_mvN0   or N_now_R_accN   or N_now_R_mvN1                 	                         or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                                                or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(N_now_R_accN)     or N_now_nobodyhome                   or N_now_R_accN   or N_now_R_mvN1                   	                         or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                                                or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(Here_now_R_mvN1)  or N_now_nobodyhome                                     or N_now_R_mvN1                 	                         or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(Here_now_R_mvN0)  or S_now_nobodyhome or S_now_R_mvN0                                   ;
	model.addClause(c);
	c = not(Here_now_R_mvS0)  or S_now_nobodyhome or S_now_R_mvS0   or S_now_R_accS   or S_now_R_mvS1                      	                         or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                                                or S_now_C0R_mvS3   or S_now_C1R_mvN3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_R_accS)  or S_now_nobodyhome                   or S_now_R_accS   or S_now_R_mvS1                     	                         or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                                                or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_R_mvS1)  or S_now_nobodyhome                                     or S_now_R_mvS1                   	                         or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_R_mvS0)  or S_now_nobodyhome or N_now_R_mvS0                                   ;
	model.addClause(c);
	c = not(Here_now_R_mvE0)  or E_now_nobodyhome or E_now_R_mvE0   or E_now_R_accE                                    	                         or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1;
	model.addClause(c);
	c = not(Here_now_R_accE)  or E_now_nobodyhome                   or E_now_R_accE                                    	                         or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1;
	model.addClause(c);
	c = not(Here_now_R_mvE0)  or W_now_nobodyhome or W_now_R_mvE0                                   ;
	model.addClause(c); 
	c = not(Here_now_R_mvW0)  or W_now_nobodyhome or W_now_R_mvW0   or W_now_R_accW                                    	                         or W_now_C0R_mvW1   or W_now_C1R_mvW1 or W_now_C2R_mvW1;
	model.addClause(c);
        c = not(Here_now_R_accW)  or W_now_nobodyhome                   or W_now_R_accW                                    	                         or W_now_C0R_mvW1   or W_now_C1R_mvW1 or W_now_C2R_mvW1;
	model.addClause(c);
	c = not(Here_now_R_mvW0)  or W_now_nobodyhome or E_now_R_mvW0                                   ; 
	model.addClause(c);


	c = not(Here_now_C0R_mvN0) or N_now_nobodyhome or N_now_C0R_mvN0 or N_now_C1R_mvN0 or N_now_C2R_mvN0                                              or N_now_C0R_accN   or N_now_C1R_accN or N_now_C2R_accN                               	                         or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                        or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                               or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_accN                                                or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C0R_accN) or N_now_nobodyhome                                                                                                    or N_now_C0R_accN   or N_now_C1R_accN or N_now_C2R_accN                               	                         or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                        or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                               or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_accN                                                or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C0R_mvN1) or N_now_nobodyhome                                                                  	                          or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                         or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                        or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C0R_mvN2) or N_now_nobodyhome                                                                   	                          or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                         or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(Here_now_C0R_mvN3) or N_now_nobodyhome                                                                   	                          or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(Here_now_C0R_mvN0) or S_now_nobodyhome or N_now_C0R_mvN0 or N_now_C1R_mvN0 or N_now_C2R_mvN0;
	model.addClause(c);

	c = not(Here_now_C0R_mvS0) or S_now_nobodyhome or S_now_C0R_mvS0 or S_now_C1R_mvS0 or S_now_C2R_mvS0                                              or S_now_C0R_accS   or S_now_C1R_accS or S_now_C2R_accS                               	                         or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                        or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                               or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_accS                                                or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C0R_accS) or S_now_nobodyhome                                                                                                    or S_now_C0R_accS   or S_now_C1R_accS or S_now_C2R_accS                               	                         or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                        or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                               or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_accS                                                or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C0R_mvS1) or S_now_nobodyhome                                                                  	                          or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                         or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                        or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C0R_mvS2) or S_now_nobodyhome                                                                   	                          or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                         or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_C0R_mvS3) or S_now_nobodyhome                                                                   	                          or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_C0R_mvS0) or S_now_nobodyhome or S_now_C0R_mvS0 or S_now_C1R_mvS0 or S_now_C2R_mvS0;
	model.addClause(c);    

	c = not(Here_now_C0R_mvE0) or E_now_nobodyhome or E_now_C0R_mvE0 or E_now_C1R_mvE0 or E_now_C2R_mvE0                                              or E_now_C0R_accE   or E_now_C1R_accE or E_now_C2R_accE                               	                         or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1 or E_now_R_accE;
	model.addClause(c);
	c = not(Here_now_C0R_accE) or E_now_nobodyhome                                                                                                    or E_now_C0R_accE   or E_now_C1R_accE or E_now_C2R_accE                               	                         or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1 or E_now_R_accE;
	model.addClause(c);
	c = not(Here_now_C0R_mvE1) or E_now_nobodyhome                                                                   	                          or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1;
	model.addClause(c);
	c = not(Here_now_C0R_mvE0) or W_now_nobodyhome or E_now_C0R_mvE0 or E_now_C1R_mvE0 or E_now_C2R_mvE0;
	model.addClause(c);
	c = not(Here_now_C0R_mvW0) or W_now_nobodyhome or W_now_C0R_mvW0 or W_now_C1R_mvW0 or W_now_C2R_mvW0                                              or W_now_C0R_accE   or W_now_C1R_accW or W_now_C2R_accW                               	                         or W_now_C0R_mvE1   or W_now_C1R_mvW1 or W_now_C2R_mvW1 or W_now_R_accW;
	model.addClause(c);
	c = not(Here_now_C0R_accW) or W_now_nobodyhome                                                                                                    or W_now_C0R_accW   or W_now_C1R_accW or W_now_C2R_accW                               	                         or W_now_C0R_mvW1   or W_now_C1R_mvW1 or W_now_C2R_mvW1 or W_now_R_accW;
	model.addClause(c);
	c = not(Here_now_C0R_mvW1) or W_now_nobodyhome                                                                   	                          or W_now_C0R_mvW1   or W_now_C1R_mvW1 or W_now_C2R_mvW1;
	model.addClause(c);
	c = not(Here_now_C0R_mvW0) or E_now_nobodyhome or W_now_C0R_mvW0 or W_now_C1R_mvW0 or W_now_C2R_mvW0;


	c = not(Here_now_C1R_mvN0) or N_now_nobodyhome or N_now_C0R_mvN0 or N_now_C1R_mvN0 or N_now_C2R_mvN0                                              or N_now_C0R_accN   or N_now_C1R_accN or N_now_C2R_accN                               	                         or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                        or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                               or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_accN                                                or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C1R_accN) or N_now_nobodyhome                                                                                                    or N_now_C0R_accN   or N_now_C1R_accN or N_now_C2R_accN                               	                         or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                        or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                               or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_accN                                                or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C1R_mvN1) or N_now_nobodyhome                                                                  	                          or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                         or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                        or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C1R_mvN2) or N_now_nobodyhome                                                                   	                          or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                         or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(Here_now_C1R_mvN3) or N_now_nobodyhome                                                                   	                          or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(Here_now_C1R_mvN0) or S_now_nobodyhome or N_now_C0R_mvN0 or N_now_C1R_mvN0 or N_now_C2R_mvN0;
	model.addClause(c);

	c = not(Here_now_C1R_mvS0) or S_now_nobodyhome or S_now_C0R_mvS0 or S_now_C1R_mvS0 or S_now_C2R_mvS0                                              or S_now_C0R_accS   or S_now_C1R_accS or S_now_C2R_accS                               	                         or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                        or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                               or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_accS                                                or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C1R_accS) or S_now_nobodyhome                                                                                                    or S_now_C0R_accS   or S_now_C1R_accS or S_now_C2R_accS                               	                         or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                        or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                               or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_accS                                                or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C1R_mvS1) or S_now_nobodyhome                                                                  	                          or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                         or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                        or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C1R_mvS2) or S_now_nobodyhome                                                                   	                          or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                         or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_C1R_mvS3) or S_now_nobodyhome                                                                   	                          or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_C1R_mvS0) or S_now_nobodyhome or S_now_C0R_mvS0 or S_now_C1R_mvS0 or S_now_C2R_mvS0;
	model.addClause(c);    

	c = not(Here_now_C1R_mvE0) or E_now_nobodyhome or E_now_C0R_mvE0 or E_now_C1R_mvE0 or E_now_C2R_mvE0                                              or E_now_C0R_accE   or E_now_C1R_accE or E_now_C2R_accE                               	                         or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1 or E_now_R_accE;
	model.addClause(c);
	c = not(Here_now_C1R_accE) or E_now_nobodyhome                                                                                                    or E_now_C0R_accE   or E_now_C1R_accE or E_now_C2R_accE                               	                         or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1 or E_now_R_accE;
	model.addClause(c);
	c = not(Here_now_C1R_mvE1) or E_now_nobodyhome                                                                   	                          or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1;
	model.addClause(c);
	c = not(Here_now_C1R_mvE0) or W_now_nobodyhome or E_now_C0R_mvE0 or E_now_C1R_mvE0 or E_now_C2R_mvE0;
	model.addClause(c);
	c = not(Here_now_C1R_mvW0) or W_now_nobodyhome or W_now_C0R_mvW0 or W_now_C1R_mvW0 or W_now_C2R_mvW0                                              or W_now_C0R_accE   or W_now_C1R_accW or W_now_C2R_accW                               	                         or W_now_C0R_mvE1   or W_now_C1R_mvW1 or W_now_C2R_mvW1 or W_now_R_accW;
	model.addClause(c);
	c = not(Here_now_C1R_accW) or W_now_nobodyhome                                                                                                    or W_now_C0R_accW   or W_now_C1R_accW or W_now_C2R_accW                               	                         or W_now_C0R_mvW1   or W_now_C1R_mvW1 or W_now_C2R_mvW1 or W_now_R_accW;
	model.addClause(c);
	c = not(Here_now_C1R_mvW1) or W_now_nobodyhome                                                                   	                          or W_now_C0R_mvW1   or W_now_C1R_mvW1 or W_now_C2R_mvW1;
	model.addClause(c);
	c = not(Here_now_C1R_mvW0) or E_now_nobodyhome or W_now_C0R_mvW0 or W_now_C1R_mvW0 or W_now_C2R_mvW0;


	c = not(Here_now_C2R_mvN0) or N_now_nobodyhome or N_now_C0R_mvN0 or N_now_C1R_mvN0 or N_now_C2R_mvN0                                              or N_now_C0R_accN   or N_now_C1R_accN or N_now_C2R_accN                               	                         or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                        or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                               or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_accN                                                or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C2R_accN) or N_now_nobodyhome                                                                                                    or N_now_C0R_accN   or N_now_C1R_accN or N_now_C2R_accN                               	                         or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                        or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                               or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_accN                                                or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C2R_mvN1) or N_now_nobodyhome                                                                  	                          or N_now_C0R_mvN1   or N_now_C1R_mvN1 or N_now_C2R_mvN1                                	                         or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                        or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3 or N_now_R_mvN1;
	model.addClause(c);
	c = not(Here_now_C2R_mvN2) or N_now_nobodyhome                                                                   	                          or N_now_C0R_mvN2   or N_now_C1R_mvN2 or N_now_C2R_mvN2                                	                         or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(Here_now_C2R_mvN3) or N_now_nobodyhome                                                                   	                          or N_now_C0R_mvN3   or N_now_C1R_mvN3 or N_now_C2R_mvN3;
	model.addClause(c);
	c = not(Here_now_C2R_mvN0) or S_now_nobodyhome or N_now_C0R_mvN0 or N_now_C1R_mvN0 or N_now_C2R_mvN0;
	model.addClause(c);

	c = not(Here_now_C2R_mvS0) or S_now_nobodyhome or S_now_C0R_mvS0 or S_now_C1R_mvS0 or S_now_C2R_mvS0                                              or S_now_C0R_accS   or S_now_C1R_accS or S_now_C2R_accS                               	                         or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                        or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                               or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_accS                                                or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C2R_accS) or S_now_nobodyhome                                                                                                    or S_now_C0R_accS   or S_now_C1R_accS or S_now_C2R_accS                               	                         or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                        or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                               or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_accS                                                or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C2R_mvS1) or S_now_nobodyhome                                                                  	                          or S_now_C0R_mvS1   or S_now_C1R_mvS1 or S_now_C2R_mvS1                                	                         or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                        or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3 or S_now_R_mvS1;
	model.addClause(c);
	c = not(Here_now_C2R_mvS2) or S_now_nobodyhome                                                                   	                          or S_now_C0R_mvS2   or S_now_C1R_mvS2 or S_now_C2R_mvS2                                	                         or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_C2R_mvS3) or S_now_nobodyhome                                                                   	                          or S_now_C0R_mvS3   or S_now_C1R_mvS3 or S_now_C2R_mvS3;
	model.addClause(c);
	c = not(Here_now_C2R_mvS0) or S_now_nobodyhome or S_now_C0R_mvS0 or S_now_C1R_mvS0 or S_now_C2R_mvS0;
	model.addClause(c);    

	c = not(Here_now_C2R_mvE0) or E_now_nobodyhome or E_now_C0R_mvE0 or E_now_C1R_mvE0 or E_now_C2R_mvE0                                              or E_now_C0R_accE   or E_now_C1R_accE or E_now_C2R_accE                               	                         or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1 or E_now_R_accE;
	model.addClause(c);
	c = not(Here_now_C2R_accE) or E_now_nobodyhome                                                                                                    or E_now_C0R_accE   or E_now_C1R_accE or E_now_C2R_accE                               	                         or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1 or E_now_R_accE;
	model.addClause(c);
	c = not(Here_now_C2R_mvE1) or E_now_nobodyhome                                                                   	                          or E_now_C0R_mvE1   or E_now_C1R_mvE1 or E_now_C2R_mvE1;
	model.addClause(c);
	c = not(Here_now_C2R_mvE0) or W_now_nobodyhome or E_now_C0R_mvE0 or E_now_C1R_mvE0 or E_now_C2R_mvE0;
	model.addClause(c);
	c = not(Here_now_C2R_mvW0) or W_now_nobodyhome or W_now_C0R_mvW0 or W_now_C1R_mvW0 or W_now_C2R_mvW0                                              or W_now_C0R_accE   or W_now_C1R_accW or W_now_C2R_accW                               	                         or W_now_C0R_mvE1   or W_now_C1R_mvW1 or W_now_C2R_mvW1 or W_now_R_accW;
	model.addClause(c);
	c = not(Here_now_C2R_accW) or W_now_nobodyhome                                                                                                    or W_now_C0R_accW   or W_now_C1R_accW or W_now_C2R_accW                               	                         or W_now_C0R_mvW1   or W_now_C1R_mvW1 or W_now_C2R_mvW1 or W_now_R_accW;
	model.addClause(c);
	c = not(Here_now_C2R_mvW1) or W_now_nobodyhome                                                                   	                          or W_now_C0R_mvW1   or W_now_C1R_mvW1 or W_now_C2R_mvW1;
	model.addClause(c);
	c = not(Here_now_C2R_mvW0) or E_now_nobodyhome or W_now_C0R_mvW0 or W_now_C1R_mvW0 or W_now_C2R_mvW0;
	model.addClause(c);

	model.dump(out);
    }


    // L I F T I N G + D R O P P I N G
    {
    	CNF::Clause c;
        const CNF::Var Here_now_empty       = var(v,       t,    On_Node::empty);

	//const CNF::Var Here_now_R_ready     = var(v,       t,    NdStat::R_ready);
	const CNF::Var Here_now_R_lift      = var(v,       t,    R_Vertical::lift);
	const CNF::Var Here_now_R_lifting1  = var(v,       t,    R_Vertical::l1);
	const CNF::Var Here_now_R_lifting2  = var(v,       t,    R_Vertical::l2);
	const CNF::Var Here_now_R_lifting3  = var(v,       t,    R_Vertical::l3);
	const CNF::Var Here_now_R_lifting4  = var(v,       t,    R_Vertical::l4);
	const CNF::Var Here_now_R_drop      = var(v,       t,    R_Vertical::drop);

	c = not(Here_now_R_lift)     or  not(Here_now_empty);
	model.addClause(c);
	c = not(Here_now_R_lifting1) or  not(Here_now_empty);
	model.addClause(c);
	c = not(Here_now_R_lifting2) or  not(Here_now_empty);
	model.addClause(c);
	c = not(Here_now_R_lifting3) or  not(Here_now_empty);
	model.addClause(c);
	c = not(Here_now_R_lifting4) or  not(Here_now_empty);
	model.addClause(c);
	c = not(Here_now_R_drop)     or  not(Here_now_empty);
	model.addClause(c);

	model.dump(out);
    }

} // atom_constraints()


//********************************************************************************************************************************************************************************************************
//       T I M E  L I N K     C O N S T R A I N T S
//********************************************************************************************************************************************************************************************************

void GridSpace::Grid_Sat::time_link_constraints(const XY v, const unsigned t)
{
    if (t>=t_max) throw std::range_error("Grid_Sat::time_link_constraints(): It's too late!");
    
    // B A S I C S
    {
    	CNF::Clause c;
        //const CNF::Var Here_now_nobodyhome= var(v,    t,     NdStat::nobodyhome);
	const CNF::Var Here_now_R_ready   = var(v,    t,     NdStat::R_ready);
	//const CNF::Var Here_now_R_accE    = var(v,    t,     R_Move::accE);
	//const CNF::Var Here_now_R_mvE0    = var(v,    t,     R_Move::mvE0);
	//const CNF::Var Here_now_R_accW    = var(v,    t,     R_Move::accW);
	//const CNF::Var Here_now_R_mvW0    = var(v,    t,     R_Move::mvW0);
	//const CNF::Var Here_now_R_accN    = var(v,    t,     R_Move::accN);
	//const CNF::Var Here_now_R_mvN1    = var(v,    t,     R_Move::mvN1);
	//const CNF::Var Here_now_R_mvN0    = var(v,    t,     R_Move::mvN0);
	//const CNF::Var Here_now_R_accS    = var(v,    t,     R_Move::accS);
	//const CNF::Var Here_now_R_mvS1    = var(v,    t,     R_Move::mvS1);
	//const CNF::Var Here_now_R_mvS0    = var(v,    t,     R_Move::mvS0);
	//const CNF::Var Here_now_R_lift    = var(v,    t,     R_Vertical::lift);
	//const CNF::Var Here_now_R_lifting1= var(v,    t,     R_Vertical::l1);
	//const CNF::Var Here_now_R_lifting2= var(v,    t,     R_Vertical::l2);
	//const CNF::Var Here_now_R_lifting3= var(v,    t,     R_Vertical::l3);
	const CNF::Var Here_now_R_lifting4= var(v,    t,     R_Vertical::l4);
	const CNF::Var Here_now_R_drop    = var(v,    t,     R_Vertical::drop);
	const CNF::Var Here_now_C0R_ready = var(v,    t,     NdStat::C0R_ready);
	//const CNF::Var Here_now_C0R_accE  = var(v,    t,     R_Move::w0_accE);
	//const CNF::Var Here_now_C0R_mvE1  = var(v,    t,     R_Move::w0_mvE1);
	//const CNF::Var Here_now_C0R_mvE0  = var(v,    t,     R_Move::w0_mvE0);
	//const CNF::Var Here_now_C0R_accW  = var(v,    t,     R_Move::w0_accW);
	//const CNF::Var Here_now_C0R_mvW1  = var(v,    t,     R_Move::w0_mvW1);
	//const CNF::Var Here_now_C0R_mvW0  = var(v,    t,     R_Move::w0_mvW0);
	//const CNF::Var Here_now_C0R_accN  = var(v,    t,     R_Move::w0_accN);
	//const CNF::Var Here_now_C0R_mvN1  = var(v,    t,     R_Move::w0_mvN1);
	//const CNF::Var Here_now_C0R_mvN2  = var(v,    t,     R_Move::w0_mvN2);
	//const CNF::Var Here_now_C0R_mvN3  = var(v,    t,     R_Move::w0_mvN3);
	//const CNF::Var Here_now_C0R_mvN0  = var(v,    t,     R_Move::w0_mvN0);
	//const CNF::Var Here_now_C0R_accS  = var(v,    t,     R_Move::w0_accS);
	//const CNF::Var Here_now_C0R_mvS1  = var(v,    t,     R_Move::w0_mvS1);
	//const CNF::Var Here_now_C0R_mvS2  = var(v,    t,     R_Move::w0_mvS2);
	//const CNF::Var Here_now_C0R_mvS3  = var(v,    t,     R_Move::w0_mvS3);
	//const CNF::Var Here_now_C0R_mvS0  = var(v,    t,     R_Move::w0_mvS0);
	const CNF::Var Here_now_C1R_ready = var(v,    t,     NdStat::C1R_ready);
	//const CNF::Var Here_now_C1R_accE  = var(v,    t,     R_Move::w1_accE);
	//const CNF::Var Here_now_C1R_mvE1  = var(v,    t,     R_Move::w1_mvE1);
	//const CNF::Var Here_now_C1R_mvE0  = var(v,    t,     R_Move::w1_mvE0);
	//const CNF::Var Here_now_C1R_accW  = var(v,    t,     R_Move::w1_accW);
	//const CNF::Var Here_now_C1R_mvW1  = var(v,    t,     R_Move::w1_mvW1);
	//const CNF::Var Here_now_C1R_mvW0  = var(v,    t,     R_Move::w1_mvW0);
	//const CNF::Var Here_now_C1R_accN  = var(v,    t,     R_Move::w1_accN);
	//const CNF::Var Here_now_C1R_mvN1  = var(v,    t,     R_Move::w1_mvN1);
	//const CNF::Var Here_now_C1R_mvN2  = var(v,    t,     R_Move::w1_mvN2);
	//const CNF::Var Here_now_C1R_mvN3  = var(v,    t,     R_Move::w1_mvN3);
	//const CNF::Var Here_now_C1R_mvN0  = var(v,    t,     R_Move::w1_mvN0);
	//const CNF::Var Here_now_C1R_accS  = var(v,    t,     R_Move::w1_accS);
	//const CNF::Var Here_now_C1R_mvS1  = var(v,    t,     R_Move::w1_mvS1);
	//const CNF::Var Here_now_C1R_mvS2  = var(v,    t,     R_Move::w1_mvS2);
	//const CNF::Var Here_now_C1R_mvS3  = var(v,    t,     R_Move::w1_mvS3);
	//const CNF::Var Here_now_C1R_mvS0  = var(v,    t,     R_Move::w1_mvS0);
	const CNF::Var Here_now_C2R_ready = var(v,    t,     NdStat::C2R_ready);
	//const CNF::Var Here_now_C2R_accE  = var(v,    t,     R_Move::w2_accE);
	//const CNF::Var Here_now_C2R_mvE1  = var(v,    t,     R_Move::w2_mvE1);
	//const CNF::Var Here_now_C2R_mvE0  = var(v,    t,     R_Move::w2_mvE0);
	//const CNF::Var Here_now_C2R_accW  = var(v,    t,     R_Move::w2_accW);
	//const CNF::Var Here_now_C2R_mvW1  = var(v,    t,     R_Move::w2_mvW1);
	//const CNF::Var Here_now_C2R_mvW0  = var(v,    t,     R_Move::w2_mvW0);
	//const CNF::Var Here_now_C2R_accN  = var(v,    t,     R_Move::w2_accN);
	//const CNF::Var Here_now_C2R_mvN1  = var(v,    t,     R_Move::w2_mvN1);
	//const CNF::Var Here_now_C2R_mvN2  = var(v,    t,     R_Move::w2_mvN2);
	//const CNF::Var Here_now_C2R_mvN3  = var(v,    t,     R_Move::w2_mvN3);
	//const CNF::Var Here_now_C2R_mvN0  = var(v,    t,     R_Move::w2_mvN0);
	//const CNF::Var Here_now_C2R_accS  = var(v,    t,     R_Move::w2_accS);
	//const CNF::Var Here_now_C2R_mvS1  = var(v,    t,     R_Move::w2_mvS1);
	//const CNF::Var Here_now_C2R_mvS2  = var(v,    t,     R_Move::w2_mvS2);
	//const CNF::Var Here_now_C2R_mvS3  = var(v,    t,     R_Move::w2_mvS3);
	//const CNF::Var Here_now_C2R_mvS0  = var(v,    t,     R_Move::w2_mvS0);


	//const CNF::Var Here_will_nobodyhome= var(v,    t+1,   NdStat::nobodyhome);
	const CNF::Var Here_will_R_ready   = var(v,    t+1,   NdStat::R_ready);
	const CNF::Var Here_will_R_accE    = var(v,    t+1,   R_Move::accE);
	//const CNF::Var Here_will_R_mvE0    = var(v,    t+1,   R_Move::mvE0);
	const CNF::Var Here_will_R_accW    = var(v,    t+1,   R_Move::accW);
	//const CNF::Var Here_will_R_mvW0    = var(v,    t+1,   R_Move::mvW0);
	const CNF::Var Here_will_R_accN    = var(v,    t+1,   R_Move::accN);
	//const CNF::Var Here_will_R_mvN1    = var(v,    t+1,   R_Move::mvN1);
	//const CNF::Var Here_will_R_mvN0    = var(v,    t+1,   R_Move::mvN0);
	const CNF::Var Here_will_R_accS    = var(v,    t+1,   R_Move::accS);
	//const CNF::Var Here_will_R_mvS1    = var(v,    t+1,   R_Move::mvS1);
	//const CNF::Var Here_will_R_mvS0    = var(v,    t+1,   R_Move::mvS0);
	const CNF::Var Here_will_R_lift    = var(v,    t+1,   R_Vertical::lift);
	//const CNF::Var Here_will_R_lifting1= var(v,    t+1,   R_Vertical::l1);
	//const CNF::Var Here_will_R_lifting2= var(v,    t+1,   R_Vertical::l2);
	//const CNF::Var Here_will_R_lifting3= var(v,    t+1,   R_Vertical::l3);
	//const CNF::Var Here_will_R_lifting4= var(v,    t+1,   R_Vertical::l4);
	const CNF::Var Here_will_R_drop    = var(v,    t+1,   R_Vertical::drop);
	const CNF::Var Here_will_C0R_ready = var(v,    t+1,   NdStat::C0R_ready);
	const CNF::Var Here_will_C0R_accE  = var(v,    t+1,   R_Move::w0_accE);
	//const CNF::Var Here_will_C0R_mvE1  = var(v,    t+1,   R_Move::w0_mvE1);
	//const CNF::Var Here_will_C0R_mvE0  = var(v,    t+1,   R_Move::w0_mvE0);
	const CNF::Var Here_will_C0R_accW  = var(v,    t+1,   R_Move::w0_accW);
	//const CNF::Var Here_will_C0R_mvW1  = var(v,    t+1,   R_Move::w0_mvW1);
	//const CNF::Var Here_will_C0R_mvW0  = var(v,    t+1,   R_Move::w0_mvW0);
	const CNF::Var Here_will_C0R_accN  = var(v,    t+1,   R_Move::w0_accN);
	//const CNF::Var Here_will_C0R_mvN1  = var(v,    t+1,   R_Move::w0_mvN1);
	//const CNF::Var Here_will_C0R_mvN2  = var(v,    t+1,   R_Move::w0_mvN2);
	//const CNF::Var Here_will_C0R_mvN3  = var(v,    t+1,   R_Move::w0_mvN3);
	//const CNF::Var Here_will_C0R_mvN0  = var(v,    t+1,   R_Move::w0_mvN0);
	const CNF::Var Here_will_C0R_accS  = var(v,    t+1,   R_Move::w0_accS);
	//const CNF::Var Here_will_C0R_mvS1  = var(v,    t+1,   R_Move::w0_mvS1);
	//const CNF::Var Here_will_C0R_mvS2  = var(v,    t+1,   R_Move::w0_mvS2);
	//const CNF::Var Here_will_C0R_mvS3  = var(v,    t+1,   R_Move::w0_mvS3);
	//const CNF::Var Here_will_C0R_mvS0  = var(v,    t+1,   R_Move::w0_mvS0);
	const CNF::Var Here_will_C1R_ready = var(v,    t+1,   NdStat::C1R_ready);
	const CNF::Var Here_will_C1R_accE  = var(v,    t+1,   R_Move::w1_accE);
	//const CNF::Var Here_will_C1R_mvE1  = var(v,    t+1,   R_Move::w1_mvE1);
	//const CNF::Var Here_will_C1R_mvE0  = var(v,    t+1,   R_Move::w1_mvE0);
	const CNF::Var Here_will_C1R_accW  = var(v,    t+1,   R_Move::w1_accW);
	//const CNF::Var Here_will_C1R_mvW1  = var(v,    t+1,   R_Move::w1_mvW1);
	//const CNF::Var Here_will_C1R_mvW0  = var(v,    t+1,   R_Move::w1_mvW0);
	const CNF::Var Here_will_C1R_accN  = var(v,    t+1,   R_Move::w1_accN);
	//const CNF::Var Here_will_C1R_mvN1  = var(v,    t+1,   R_Move::w1_mvN1);
	//const CNF::Var Here_will_C1R_mvN2  = var(v,    t+1,   R_Move::w1_mvN2);
	//const CNF::Var Here_will_C1R_mvN3  = var(v,    t+1,   R_Move::w1_mvN3);
	//const CNF::Var Here_will_C1R_mvN0  = var(v,    t+1,   R_Move::w1_mvN0);
	const CNF::Var Here_will_C1R_accS  = var(v,    t+1,   R_Move::w1_accS);
	//const CNF::Var Here_will_C1R_mvS1  = var(v,    t+1,   R_Move::w1_mvS1);
	//const CNF::Var Here_will_C1R_mvS2  = var(v,    t+1,   R_Move::w1_mvS2);
	//const CNF::Var Here_will_C1R_mvS3  = var(v,    t+1,   R_Move::w1_mvS3);
	//const CNF::Var Here_will_C1R_mvS0  = var(v,    t+1,   R_Move::w1_mvS0);
	const CNF::Var Here_will_C2R_ready = var(v,    t+1,   NdStat::C2R_ready);
	const CNF::Var Here_will_C2R_accE  = var(v,    t+1,   R_Move::w2_accE);
	//const CNF::Var Here_will_C2R_mvE1  = var(v,    t+1,   R_Move::w2_mvE1);
	//const CNF::Var Here_will_C2R_mvE0  = var(v,    t+1,   R_Move::w2_mvE0);
	const CNF::Var Here_will_C2R_accW  = var(v,    t+1,   R_Move::w2_accW);
	//const CNF::Var Here_will_C2R_mvW1  = var(v,    t+1,   R_Move::w2_mvW1);
	//const CNF::Var Here_will_C2R_mvW0  = var(v,    t+1,   R_Move::w2_mvW0);
	const CNF::Var Here_will_C2R_accN  = var(v,    t+1,   R_Move::w2_accN);
	//const CNF::Var Here_will_C2R_mvN1  = var(v,    t+1,   R_Move::w2_mvN1);
	//const CNF::Var Here_will_C2R_mvN2  = var(v,    t+1,   R_Move::w2_mvN2);
	//const CNF::Var Here_will_C2R_mvN3  = var(v,    t+1,   R_Move::w2_mvN3);
	//const CNF::Var Here_will_C2R_mvN0  = var(v,    t+1,   R_Move::w2_mvN0);
	const CNF::Var Here_will_C2R_accS  = var(v,    t+1,   R_Move::w2_accS);
	//const CNF::Var Here_will_C2R_mvS1  = var(v,    t+1,   R_Move::w2_mvS1);
	//const CNF::Var Here_will_C2R_mvS2  = var(v,    t+1,   R_Move::w2_mvS2);
	//const CNF::Var Here_will_C2R_mvS3  = var(v,    t+1,   R_Move::w2_mvS3);
	//const CNF::Var Here_will_C2R_mvS0  = var(v,    t+1,   R_Move::w2_mvS0);



	const CNF::Var W_now_R_accE    = var( G.west(v),    t,    R_Move::accE);
	const CNF::Var W_now_R_mvE0    = var( G.west(v),    t,    R_Move::mvE0);
	const CNF::Var W_now_C0R_mvE1  = var( G.west(v),    t,    R_Move::w0_mvE1);
	const CNF::Var W_now_C1R_mvE1  = var( G.west(v),    t,    R_Move::w1_mvE1);
	const CNF::Var W_now_C2R_mvE1  = var( G.west(v),    t,    R_Move::w2_mvE1);

	const CNF::Var E_now_R_accW    = var( G.east(v),    t,    R_Move::accW);
	const CNF::Var E_now_R_mvW0    = var( G.east(v),    t,    R_Move::mvW0);
	const CNF::Var E_now_C0R_mvW1  = var( G.east(v),    t,    R_Move::w0_mvW1);
	const CNF::Var E_now_C1R_mvW1  = var( G.east(v),    t,    R_Move::w1_mvW1);
	const CNF::Var E_now_C2R_mvW1  = var( G.east(v),    t,    R_Move::w2_mvW1);

	const CNF::Var S_now_R_mvN1    = var( G.south(v),    t,    R_Move::mvN1);
	const CNF::Var S_now_C0R_mvN3  = var( G.south(v),    t,    R_Move::w0_mvN3);
	const CNF::Var S_now_C1R_mvN3  = var( G.south(v),    t,    R_Move::w1_mvN3);
	const CNF::Var S_now_C2R_mvN3  = var( G.south(v),    t,    R_Move::w2_mvN3);

	const CNF::Var N_now_R_mvS1    = var( G.north(v),    t,    R_Move::mvS1);
	const CNF::Var N_now_C0R_mvS3  = var( G.north(v),    t,    R_Move::w0_mvS3);
	const CNF::Var N_now_C1R_mvS3  = var( G.north(v),    t,    R_Move::w1_mvS3);
	const CNF::Var N_now_C2R_mvS3  = var( G.north(v),    t,    R_Move::w2_mvS3);

	c = not(Here_now_R_ready)    or Here_will_R_ready                                                                                                   or Here_will_R_accE    or Here_will_R_accN   or Here_will_R_accW   or Here_will_R_accS                                 or Here_will_R_lift;
	model.addClause(c);
	c = not(Here_will_R_ready)   or Here_now_R_ready                                                                                                    or W_now_R_mvE0        or W_now_R_accE                                                                                 or E_now_R_mvW0        or E_now_R_accW                                                                                 or S_now_R_mvN1                                                                                                        or N_now_R_mvS1                                                                                                        or Here_now_R_drop;
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or Here_will_C0R_ready                                                                                                 or Here_will_C0R_accE  or Here_will_C0R_accN or Here_will_C0R_accW or Here_will_C0R_accS                                                                                                                                                      or Here_will_R_drop;
	model.addClause(c);
	c = not(Here_will_C0R_ready) or Here_now_C0R_ready                                                                                                  or W_now_C0R_mvE1                                                                                                      or E_now_C0R_mvW1                                                                                                      or S_now_C0R_mvN3                                                                                                      or N_now_C0R_mvS3                                                                                                      or Here_now_R_lifting4;
	model.addClause(c);
	c = not(Here_now_C1R_ready)  or Here_will_C1R_ready                                                                                                 or Here_will_C1R_accE  or Here_will_C1R_accN or Here_will_C1R_accW or Here_will_C1R_accS                                                                                                                                                      or Here_will_R_drop;
	model.addClause(c);
	c = not(Here_will_C1R_ready) or Here_now_C1R_ready                                                                                                  or W_now_C1R_mvE1                                                                                                      or E_now_C1R_mvW1                                                                                                      or S_now_C1R_mvN3                                                                                                      or N_now_C1R_mvS3                                                                                                      or Here_now_R_lifting4;
	model.addClause(c);
	c = not(Here_now_C2R_ready)  or Here_will_C2R_ready                                                                                                 or Here_will_C2R_accE or Here_will_C2R_accN or Here_will_C2R_accW or Here_will_C2R_accS                                                                                                                                                       or Here_will_R_drop;
	model.addClause(c);
	c = not(Here_will_C2R_ready) or Here_now_C2R_ready                                                                                                  or W_now_C2R_mvE1                                                                                                      or E_now_C2R_mvW1                                                                                                      or S_now_C2R_mvN3                                                                                                      or N_now_C2R_mvS3                                                                                                      or Here_now_R_lifting4;
	model.addClause(c);

	model.dump(out);
    }
    
    // M O V E M E N T
    {
    	CNF::Clause c;
        //const CNF::Var Here_now_nobodyhome= var(v,    t,     NdStat::nobodyhome);
	const CNF::Var Here_now_R_ready   = var(v,    t,     NdStat::R_ready);
	const CNF::Var Here_now_R_accE    = var(v,    t,     R_Move::accE);
	const CNF::Var Here_now_R_mvE0    = var(v,    t,     R_Move::mvE0);
	const CNF::Var Here_now_R_accW    = var(v,    t,     R_Move::accW);
	const CNF::Var Here_now_R_mvW0    = var(v,    t,     R_Move::mvW0);
	const CNF::Var Here_now_R_accN    = var(v,    t,     R_Move::accN);
	const CNF::Var Here_now_R_mvN1    = var(v,    t,     R_Move::mvN1);
	const CNF::Var Here_now_R_mvN0    = var(v,    t,     R_Move::mvN0);
	const CNF::Var Here_now_R_accS    = var(v,    t,     R_Move::accS);
	const CNF::Var Here_now_R_mvS1    = var(v,    t,     R_Move::mvS1);
	const CNF::Var Here_now_R_mvS0    = var(v,    t,     R_Move::mvS0);
	//const CNF::Var Here_now_R_lift    = var(v,    t,     R_Vertical::lift);
	//const CNF::Var Here_now_R_lifting1= var(v,    t,     R_Vertical::l1);
	//const CNF::Var Here_now_R_lifting2= var(v,    t,     R_Vertical::l2);
	//const CNF::Var Here_now_R_lifting3= var(v,    t,     R_Vertical::l3);
	//const CNF::Var Here_now_R_lifting4= var(v,    t,     R_Vertical::l4);
	//const CNF::Var Here_now_R_drop    = var(v,    t,     R_Vertical::drop);
	const CNF::Var Here_now_C0R_ready = var(v,    t,     NdStat::C0R_ready);
	const CNF::Var Here_now_C0R_accE  = var(v,    t,     R_Move::w0_accE);
	const CNF::Var Here_now_C0R_mvE1  = var(v,    t,     R_Move::w0_mvE1);
	const CNF::Var Here_now_C0R_mvE0  = var(v,    t,     R_Move::w0_mvE0);
	const CNF::Var Here_now_C0R_accW  = var(v,    t,     R_Move::w0_accW);
	const CNF::Var Here_now_C0R_mvW1  = var(v,    t,     R_Move::w0_mvW1);
	const CNF::Var Here_now_C0R_mvW0  = var(v,    t,     R_Move::w0_mvW0);
	const CNF::Var Here_now_C0R_accN  = var(v,    t,     R_Move::w0_accN);
	const CNF::Var Here_now_C0R_mvN1  = var(v,    t,     R_Move::w0_mvN1);
	const CNF::Var Here_now_C0R_mvN2  = var(v,    t,     R_Move::w0_mvN2);
	const CNF::Var Here_now_C0R_mvN3  = var(v,    t,     R_Move::w0_mvN3);
	const CNF::Var Here_now_C0R_mvN0  = var(v,    t,     R_Move::w0_mvN0);
	const CNF::Var Here_now_C0R_accS  = var(v,    t,     R_Move::w0_accS);
	const CNF::Var Here_now_C0R_mvS1  = var(v,    t,     R_Move::w0_mvS1);
	const CNF::Var Here_now_C0R_mvS2  = var(v,    t,     R_Move::w0_mvS2);
	const CNF::Var Here_now_C0R_mvS3  = var(v,    t,     R_Move::w0_mvS3);
	const CNF::Var Here_now_C0R_mvS0  = var(v,    t,     R_Move::w0_mvS0);
	const CNF::Var Here_now_C1R_ready = var(v,    t,     NdStat::C1R_ready);
	const CNF::Var Here_now_C1R_accE  = var(v,    t,     R_Move::w1_accE);
	const CNF::Var Here_now_C1R_mvE1  = var(v,    t,     R_Move::w1_mvE1);
	const CNF::Var Here_now_C1R_mvE0  = var(v,    t,     R_Move::w1_mvE0);
	const CNF::Var Here_now_C1R_accW  = var(v,    t,     R_Move::w1_accW);
	const CNF::Var Here_now_C1R_mvW1  = var(v,    t,     R_Move::w1_mvW1);
	const CNF::Var Here_now_C1R_mvW0  = var(v,    t,     R_Move::w1_mvW0);
	const CNF::Var Here_now_C1R_accN  = var(v,    t,     R_Move::w1_accN);
	const CNF::Var Here_now_C1R_mvN1  = var(v,    t,     R_Move::w1_mvN1);
	const CNF::Var Here_now_C1R_mvN2  = var(v,    t,     R_Move::w1_mvN2);
	const CNF::Var Here_now_C1R_mvN3  = var(v,    t,     R_Move::w1_mvN3);
	const CNF::Var Here_now_C1R_mvN0  = var(v,    t,     R_Move::w1_mvN0);
	const CNF::Var Here_now_C1R_accS  = var(v,    t,     R_Move::w1_accS);
	const CNF::Var Here_now_C1R_mvS1  = var(v,    t,     R_Move::w1_mvS1);
	const CNF::Var Here_now_C1R_mvS2  = var(v,    t,     R_Move::w1_mvS2);
	const CNF::Var Here_now_C1R_mvS3  = var(v,    t,     R_Move::w1_mvS3);
	const CNF::Var Here_now_C1R_mvS0  = var(v,    t,     R_Move::w1_mvS0);
	const CNF::Var Here_now_C2R_ready = var(v,    t,     NdStat::C2R_ready);
	const CNF::Var Here_now_C2R_accE  = var(v,    t,     R_Move::w2_accE);
	const CNF::Var Here_now_C2R_mvE1  = var(v,    t,     R_Move::w2_mvE1);
	const CNF::Var Here_now_C2R_mvE0  = var(v,    t,     R_Move::w2_mvE0);
	const CNF::Var Here_now_C2R_accW  = var(v,    t,     R_Move::w2_accW);
	const CNF::Var Here_now_C2R_mvW1  = var(v,    t,     R_Move::w2_mvW1);
	const CNF::Var Here_now_C2R_mvW0  = var(v,    t,     R_Move::w2_mvW0);
	const CNF::Var Here_now_C2R_accN  = var(v,    t,     R_Move::w2_accN);
	const CNF::Var Here_now_C2R_mvN1  = var(v,    t,     R_Move::w2_mvN1);
	const CNF::Var Here_now_C2R_mvN2  = var(v,    t,     R_Move::w2_mvN2);
	const CNF::Var Here_now_C2R_mvN3  = var(v,    t,     R_Move::w2_mvN3);
	const CNF::Var Here_now_C2R_mvN0  = var(v,    t,     R_Move::w2_mvN0);
	const CNF::Var Here_now_C2R_accS  = var(v,    t,     R_Move::w2_accS);
	const CNF::Var Here_now_C2R_mvS1  = var(v,    t,     R_Move::w2_mvS1);
	const CNF::Var Here_now_C2R_mvS2  = var(v,    t,     R_Move::w2_mvS2);
	const CNF::Var Here_now_C2R_mvS3  = var(v,    t,     R_Move::w2_mvS3);
	const CNF::Var Here_now_C2R_mvS0  = var(v,    t,     R_Move::w2_mvS0);

	const CNF::Var E_now_nobodyhome= var( G.east(v),    t,    NdStat::nobodyhome);
	//const CNF::Var E_now_R_ready   = var( G.east(v),    t,    NdStat::R_ready);
	const CNF::Var E_now_R_accE    = var( G.east(v),    t,    R_Move::accE);
	//const CNF::Var E_now_R_mvE0    = var( G.east(v),    t,    R_Move::mvE0);
	const CNF::Var E_now_R_accW    = var( G.east(v),    t,    R_Move::accW);
	const CNF::Var E_now_R_mvW0    = var( G.east(v),    t,    R_Move::mvW0);
	//const CNF::Var E_now_C0R_ready = var( G.east(v),    t,    NdStat::C0R_ready);
	//const CNF::Var E_now_C0R_accE  = var( G.east(v),    t,    R_Move::w0_accE);
	const CNF::Var E_now_C0R_mvE1  = var( G.east(v),    t,    R_Move::w0_mvE1);
	//const CNF::Var E_now_C0R_mvE0  = var( G.east(v),    t,    R_Move::w0_mvE0);
	//const CNF::Var E_now_C0R_accW  = var( G.east(v),    t,    R_Move::w0_accW);
	const CNF::Var E_now_C0R_mvW1  = var( G.east(v),    t,    R_Move::w0_mvW1);
	//const CNF::Var E_now_C0R_mvW0  = var( G.east(v),    t,    R_Move::w0_mvW0);
	//const CNF::Var E_now_C1R_ready = var( G.east(v),    t,    NdStat::C1R_ready);
	//const CNF::Var E_now_C1R_accE  = var( G.east(v),    t,    R_Move::w1_accE);
	const CNF::Var E_now_C1R_mvE1  = var( G.east(v),    t,    R_Move::w1_mvE1);
	//const CNF::Var E_now_C1R_mvE0  = var( G.east(v),    t,    R_Move::w1_mvE0);
	//const CNF::Var E_now_C1R_accW  = var( G.east(v),    t,    R_Move::w1_accW);
	const CNF::Var E_now_C1R_mvW1  = var( G.east(v),    t,    R_Move::w1_mvW1);
	//const CNF::Var E_now_C1R_mvW0  = var( G.east(v),    t,    R_Move::w1_mvW0);
	//const CNF::Var E_now_C2R_ready = var( G.east(v),    t,    NdStat::C2R_ready);
	//const CNF::Var E_now_C2R_accE  = var( G.east(v),    t,    R_Move::w2_accE);
	const CNF::Var E_now_C2R_mvE1  = var( G.east(v),    t,    R_Move::w2_mvE1);
	//const CNF::Var E_now_C2R_mvE0  = var( G.east(v),    t,    R_Move::w2_mvE0);
	//const CNF::Var E_now_C2R_accW  = var( G.east(v),    t,    R_Move::w2_accW);
	const CNF::Var E_now_C2R_mvW1  = var( G.east(v),    t,    R_Move::w2_mvW1);
	//const CNF::Var E_now_C2R_mvW0  = var( G.east(v),    t,    R_Move::w2_mvW0);

	//const CNF::Var N_now_nobodyhome= var( G.north(v),    t,    NdStat::nobodyhome);
	//const CNF::Var N_now_R_ready   = var( G.north(v),    t,    NdStat::R_ready);
	//const CNF::Var N_now_R_accN    = var( G.north(v),    t,    R_Move::accN);
	//const CNF::Var N_now_R_mvN1    = var( G.north(v),    t,    R_Move::mvN1);
	//const CNF::Var N_now_R_mvN0    = var( G.north(v),    t,    R_Move::mvN0);
	//const CNF::Var N_now_R_accS    = var( G.north(v),    t,    R_Move::accS);
	const CNF::Var N_now_R_mvS1    = var( G.north(v),    t,    R_Move::mvS1);
	//const CNF::Var N_now_R_mvS0    = var( G.north(v),    t,    R_Move::mvS0);
	//const CNF::Var N_now_C0R_ready = var( G.north(v),    t,    NdStat::C0R_ready);
	//const CNF::Var N_now_C0R_accN  = var( G.north(v),    t,    R_Move::w0_accN);
	//const CNF::Var N_now_C0R_mvN1  = var( G.north(v),    t,    R_Move::w0_mvN1);
	//const CNF::Var N_now_C0R_mvN2  = var( G.north(v),    t,    R_Move::w0_mvN2);
	const CNF::Var N_now_C0R_mvN3  = var( G.north(v),    t,    R_Move::w0_mvN3);
	//const CNF::Var N_now_C0R_mvN0  = var( G.north(v),    t,    R_Move::w0_mvN0);
	//const CNF::Var N_now_C0R_accS  = var( G.north(v),    t,    R_Move::w0_accS);
	//const CNF::Var N_now_C0R_mvS1  = var( G.north(v),    t,    R_Move::w0_mvS1);
	//const CNF::Var N_now_C0R_mvS2  = var( G.north(v),    t,    R_Move::w0_mvS2);
	const CNF::Var N_now_C0R_mvS3  = var( G.north(v),    t,    R_Move::w0_mvS3);
	//const CNF::Var N_now_C0R_mvS0  = var( G.north(v),    t,    R_Move::w0_mvS0);
	//const CNF::Var N_now_C1R_ready = var( G.north(v),    t,    NdStat::C1R_ready);
	//const CNF::Var N_now_C1R_accN  = var( G.north(v),    t,    R_Move::w1_accN);
	//const CNF::Var N_now_C1R_mvN1  = var( G.north(v),    t,    R_Move::w1_mvN1);
	//const CNF::Var N_now_C1R_mvN2  = var( G.north(v),    t,    R_Move::w1_mvN2);
	const CNF::Var N_now_C1R_mvN3  = var( G.north(v),    t,    R_Move::w1_mvN3);
	//const CNF::Var N_now_C1R_mvN0  = var( G.north(v),    t,    R_Move::w1_mvN0);
	//const CNF::Var N_now_C1R_accS  = var( G.north(v),    t,    R_Move::w1_accS);
	//const CNF::Var N_now_C1R_mvS1  = var( G.north(v),    t,    R_Move::w1_mvS1);
	//const CNF::Var N_now_C1R_mvS2  = var( G.north(v),    t,    R_Move::w1_mvS2);
	const CNF::Var N_now_C1R_mvS3  = var( G.north(v),    t,    R_Move::w1_mvS3);
	//const CNF::Var N_now_C1R_mvS0  = var( G.north(v),    t,    R_Move::w1_mvS0);
	//const CNF::Var N_now_C2R_ready = var( G.north(v),    t,    NdStat::C2R_ready);
	//const CNF::Var N_now_C2R_accN  = var( G.north(v),    t,    R_Move::w2_accN);
	//const CNF::Var N_now_C2R_mvN1  = var( G.north(v),    t,    R_Move::w2_mvN1);
	//const CNF::Var N_now_C2R_mvN2  = var( G.north(v),    t,    R_Move::w2_mvN2);
	const CNF::Var N_now_C2R_mvN3  = var( G.north(v),    t,    R_Move::w2_mvN3);
	//const CNF::Var N_now_C2R_mvN0  = var( G.north(v),    t,    R_Move::w2_mvN0);
	//const CNF::Var N_now_C2R_accS  = var( G.north(v),    t,    R_Move::w2_accS);
	//const CNF::Var N_now_C2R_mvS1  = var( G.north(v),    t,    R_Move::w2_mvS1);
	//const CNF::Var N_now_C2R_mvS2  = var( G.north(v),    t,    R_Move::w2_mvS2);
	const CNF::Var N_now_C2R_mvS3  = var( G.north(v),    t,    R_Move::w2_mvS3);
	//const CNF::Var N_now_C2R_mvS0  = var( G.north(v),    t,    R_Move::w2_mvS0);

	const CNF::Var W_now_nobodyhome= var( G.west(v),    t,    NdStat::nobodyhome);
	//const CNF::Var W_now_R_ready   = var( G.west(v),    t,    NdStat::R_ready);
	const CNF::Var W_now_R_accE    = var( G.west(v),    t,    R_Move::accE);
	const CNF::Var W_now_R_mvE0    = var( G.west(v),    t,    R_Move::mvE0);
	const CNF::Var W_now_R_accW    = var( G.west(v),    t,    R_Move::accW);
	//const CNF::Var W_now_R_mvW0    = var( G.west(v),    t,    R_Move::mvW0);
	//const CNF::Var W_now_C0R_ready = var( G.west(v),    t,    NdStat::C0R_ready);
	//const CNF::Var W_now_C0R_accE  = var( G.west(v),    t,    R_Move::w0_accE);
	const CNF::Var W_now_C0R_mvE1  = var( G.west(v),    t,    R_Move::w0_mvE1);
	//const CNF::Var W_now_C0R_mvE0  = var( G.west(v),    t,    R_Move::w0_mvE0);
	//const CNF::Var W_now_C0R_accW  = var( G.west(v),    t,    R_Move::w0_accW);
	const CNF::Var W_now_C0R_mvW1  = var( G.west(v),    t,    R_Move::w0_mvW1);
	//const CNF::Var W_now_C0R_mvW0  = var( G.west(v),    t,    R_Move::w0_mvW0);
	//const CNF::Var W_now_C1R_ready = var( G.west(v),    t,    NdStat::C1R_ready);
	//const CNF::Var W_now_C1R_accE  = var( G.west(v),    t,    R_Move::w1_accE);
	const CNF::Var W_now_C1R_mvE1  = var( G.west(v),    t,    R_Move::w1_mvE1);
	//const CNF::Var W_now_C1R_mvE0  = var( G.west(v),    t,    R_Move::w1_mvE0);
	//const CNF::Var W_now_C1R_accW  = var( G.west(v),    t,    R_Move::w1_accW);
	const CNF::Var W_now_C1R_mvW1  = var( G.west(v),    t,    R_Move::w1_mvW1);
	//const CNF::Var W_now_C1R_mvW0  = var( G.west(v),    t,    R_Move::w1_mvW0);
	//const CNF::Var W_now_C2R_ready = var( G.west(v),    t,    NdStat::C2R_ready);
	//const CNF::Var W_now_C2R_accE  = var( G.west(v),    t,    R_Move::w2_accE);
	const CNF::Var W_now_C2R_mvE1  = var( G.west(v),    t,    R_Move::w2_mvE1);
	//const CNF::Var W_now_C2R_mvE0  = var( G.west(v),    t,    R_Move::w2_mvE0);
	//const CNF::Var W_now_C2R_accW  = var( G.west(v),    t,    R_Move::w2_accW);
	const CNF::Var W_now_C2R_mvW1  = var( G.west(v),    t,    R_Move::w2_mvW1);
	//const CNF::Var W_now_C2R_mvW0  = var( G.west(v),    t,    R_Move::w2_mvW0);

	//const CNF::Var S_now_nobodyhome= var( G.south(v),    t,    NdStat::nobodyhome);
	//const CNF::Var S_now_R_ready   = var( G.south(v),    t,    NdStat::R_ready);
	//const CNF::Var S_now_R_accN    = var( G.south(v),    t,    R_Move::accN);
	const CNF::Var S_now_R_mvN1    = var( G.south(v),    t,    R_Move::mvN1);
	//const CNF::Var S_now_R_mvN0    = var( G.south(v),    t,    R_Move::mvN0);
	//const CNF::Var S_now_R_accS    = var( G.south(v),    t,    R_Move::accS);
	//const CNF::Var S_now_R_mvS1    = var( G.south(v),    t,    R_Move::mvS1);
	//const CNF::Var S_now_R_mvS0    = var( G.south(v),    t,    R_Move::mvS0);
	//const CNF::Var S_now_C0R_ready = var( G.south(v),    t,    NdStat::C0R_ready);
	//const CNF::Var S_now_C0R_accN  = var( G.south(v),    t,    R_Move::w0_accN);
	//const CNF::Var S_now_C0R_mvN1  = var( G.south(v),    t,    R_Move::w0_mvN1);
	//const CNF::Var S_now_C0R_mvN2  = var( G.south(v),    t,    R_Move::w0_mvN2);
	const CNF::Var S_now_C0R_mvN3  = var( G.south(v),    t,    R_Move::w0_mvN3);
	//const CNF::Var S_now_C0R_mvN0  = var( G.south(v),    t,    R_Move::w0_mvN0);
	//const CNF::Var S_now_C0R_accS  = var( G.south(v),    t,    R_Move::w0_accS);
	//const CNF::Var S_now_C0R_mvS1  = var( G.south(v),    t,    R_Move::w0_mvS1);
	//const CNF::Var S_now_C0R_mvS2  = var( G.south(v),    t,    R_Move::w0_mvS2);
	const CNF::Var S_now_C0R_mvS3  = var( G.south(v),    t,    R_Move::w0_mvS3);
	//const CNF::Var S_now_C0R_mvS0  = var( G.south(v),    t,    R_Move::w0_mvS0);
	//const CNF::Var S_now_C1R_ready = var( G.south(v),    t,    NdStat::C1R_ready);
	//const CNF::Var S_now_C1R_accN  = var( G.south(v),    t,    R_Move::w1_accN);
	//const CNF::Var S_now_C1R_mvN1  = var( G.south(v),    t,    R_Move::w1_mvN1);
	//const CNF::Var S_now_C1R_mvN2  = var( G.south(v),    t,    R_Move::w1_mvN2);
	const CNF::Var S_now_C1R_mvN3  = var( G.south(v),    t,    R_Move::w1_mvN3);
	//const CNF::Var S_now_C1R_mvN0  = var( G.south(v),    t,    R_Move::w1_mvN0);
	//const CNF::Var S_now_C1R_accS  = var( G.south(v),    t,    R_Move::w1_accS);
	//const CNF::Var S_now_C1R_mvS1  = var( G.south(v),    t,    R_Move::w1_mvS1);
	//const CNF::Var S_now_C1R_mvS2  = var( G.south(v),    t,    R_Move::w1_mvS2);
	const CNF::Var S_now_C1R_mvS3  = var( G.south(v),    t,    R_Move::w1_mvS3);
	//const CNF::Var S_now_C1R_mvS0  = var( G.south(v),    t,    R_Move::w1_mvS0);
	//const CNF::Var S_now_C2R_ready = var( G.south(v),    t,    NdStat::C2R_ready);
	//const CNF::Var S_now_C2R_accN  = var( G.south(v),    t,    R_Move::w2_accN);
	//const CNF::Var S_now_C2R_mvN1  = var( G.south(v),    t,    R_Move::w2_mvN1);
	//const CNF::Var S_now_C2R_mvN2  = var( G.south(v),    t,    R_Move::w2_mvN2);
	const CNF::Var S_now_C2R_mvN3  = var( G.south(v),    t,    R_Move::w2_mvN3);
	//const CNF::Var S_now_C2R_mvN0  = var( G.south(v),    t,    R_Move::w2_mvN0);
	//const CNF::Var S_now_C2R_accS  = var( G.south(v),    t,    R_Move::w2_accS);
	//const CNF::Var S_now_C2R_mvS1  = var( G.south(v),    t,    R_Move::w2_mvS1);
	//const CNF::Var S_now_C2R_mvS2  = var( G.south(v),    t,    R_Move::w2_mvS2);
	const CNF::Var S_now_C2R_mvS3  = var( G.south(v),    t,    R_Move::w2_mvS3);
	//const CNF::Var S_now_C2R_mvS0  = var( G.south(v),    t,    R_Move::w2_mvS0);
        
        
        // Future
	const CNF::Var Here_will_nobodyhome= var(v,    t+1,     NdStat::nobodyhome);
	//const CNF::Var Here_will_R_ready   = var(v,    t+1,     NdStat::R_ready);
	const CNF::Var Here_will_R_accE    = var(v,    t+1,     R_Move::accE);
	const CNF::Var Here_will_R_mvE0    = var(v,    t+1,     R_Move::mvE0);
	const CNF::Var Here_will_R_accW    = var(v,    t+1,     R_Move::accW);
	const CNF::Var Here_will_R_mvW0    = var(v,    t+1,     R_Move::mvW0);
	const CNF::Var Here_will_R_accN    = var(v,    t+1,     R_Move::accN);
	const CNF::Var Here_will_R_mvN1    = var(v,    t+1,     R_Move::mvN1);
	const CNF::Var Here_will_R_mvN0    = var(v,    t+1,     R_Move::mvN0);
	const CNF::Var Here_will_R_accS    = var(v,    t+1,     R_Move::accS);
	const CNF::Var Here_will_R_mvS1    = var(v,    t+1,     R_Move::mvS1);
	const CNF::Var Here_will_R_mvS0    = var(v,    t+1,     R_Move::mvS0);
	//const CNF::Var Here_will_R_lift    = var(v,    t+1,     R_Vertical::lift);
	//const CNF::Var Here_will_R_lifting1= var(v,    t+1,     R_Vertical::l1);
	//const CNF::Var Here_will_R_lifting2= var(v,    t+1,     R_Vertical::l2);
	//const CNF::Var Here_will_R_lifting3= var(v,    t+1,     R_Vertical::l3);
	//const CNF::Var Here_will_R_lifting4= var(v,    t+1,     R_Vertical::l4);
	//const CNF::Var Here_will_R_drop    = var(v,    t+1,     R_Vertical::drop);
	//const CNF::Var Here_will_C0R_ready = var(v,    t+1,     NdStat::C0R_ready);
	const CNF::Var Here_will_C0R_accE  = var(v,    t+1,     R_Move::w0_accE);
	const CNF::Var Here_will_C0R_mvE1  = var(v,    t+1,     R_Move::w0_mvE1);
	const CNF::Var Here_will_C0R_mvE0  = var(v,    t+1,     R_Move::w0_mvE0);
	const CNF::Var Here_will_C0R_accW  = var(v,    t+1,     R_Move::w0_accW);
	const CNF::Var Here_will_C0R_mvW1  = var(v,    t+1,     R_Move::w0_mvW1);
	const CNF::Var Here_will_C0R_mvW0  = var(v,    t+1,     R_Move::w0_mvW0);
	const CNF::Var Here_will_C0R_accN  = var(v,    t+1,     R_Move::w0_accN);
	const CNF::Var Here_will_C0R_mvN1  = var(v,    t+1,     R_Move::w0_mvN1);
	const CNF::Var Here_will_C0R_mvN2  = var(v,    t+1,     R_Move::w0_mvN2);
	const CNF::Var Here_will_C0R_mvN3  = var(v,    t+1,     R_Move::w0_mvN3);
	const CNF::Var Here_will_C0R_mvN0  = var(v,    t+1,     R_Move::w0_mvN0);
	const CNF::Var Here_will_C0R_accS  = var(v,    t+1,     R_Move::w0_accS);
	const CNF::Var Here_will_C0R_mvS1  = var(v,    t+1,     R_Move::w0_mvS1);
	const CNF::Var Here_will_C0R_mvS2  = var(v,    t+1,     R_Move::w0_mvS2);
	const CNF::Var Here_will_C0R_mvS3  = var(v,    t+1,     R_Move::w0_mvS3);
	const CNF::Var Here_will_C0R_mvS0  = var(v,    t+1,     R_Move::w0_mvS0);
	//const CNF::Var Here_will_C1R_ready = var(v,    t+1,     NdStat::C1R_ready);
	const CNF::Var Here_will_C1R_accE  = var(v,    t+1,     R_Move::w1_accE);
	const CNF::Var Here_will_C1R_mvE1  = var(v,    t+1,     R_Move::w1_mvE1);
	const CNF::Var Here_will_C1R_mvE0  = var(v,    t+1,     R_Move::w1_mvE0);
	const CNF::Var Here_will_C1R_accW  = var(v,    t+1,     R_Move::w1_accW);
	const CNF::Var Here_will_C1R_mvW1  = var(v,    t+1,     R_Move::w1_mvW1);
	const CNF::Var Here_will_C1R_mvW0  = var(v,    t+1,     R_Move::w1_mvW0);
	const CNF::Var Here_will_C1R_accN  = var(v,    t+1,     R_Move::w1_accN);
	const CNF::Var Here_will_C1R_mvN1  = var(v,    t+1,     R_Move::w1_mvN1);
	const CNF::Var Here_will_C1R_mvN2  = var(v,    t+1,     R_Move::w1_mvN2);
	const CNF::Var Here_will_C1R_mvN3  = var(v,    t+1,     R_Move::w1_mvN3);
	const CNF::Var Here_will_C1R_mvN0  = var(v,    t+1,     R_Move::w1_mvN0);
	const CNF::Var Here_will_C1R_accS  = var(v,    t+1,     R_Move::w1_accS);
	const CNF::Var Here_will_C1R_mvS1  = var(v,    t+1,     R_Move::w1_mvS1);
	const CNF::Var Here_will_C1R_mvS2  = var(v,    t+1,     R_Move::w1_mvS2);
	const CNF::Var Here_will_C1R_mvS3  = var(v,    t+1,     R_Move::w1_mvS3);
	const CNF::Var Here_will_C1R_mvS0  = var(v,    t+1,     R_Move::w1_mvS0);
	//const CNF::Var Here_will_C2R_ready = var(v,    t+1,     NdStat::C2R_ready);
	const CNF::Var Here_will_C2R_accE  = var(v,    t+1,     R_Move::w2_accE);
	const CNF::Var Here_will_C2R_mvE1  = var(v,    t+1,     R_Move::w2_mvE1);
	const CNF::Var Here_will_C2R_mvE0  = var(v,    t+1,     R_Move::w2_mvE0);
	const CNF::Var Here_will_C2R_accW  = var(v,    t+1,     R_Move::w2_accW);
	const CNF::Var Here_will_C2R_mvW1  = var(v,    t+1,     R_Move::w2_mvW1);
	const CNF::Var Here_will_C2R_mvW0  = var(v,    t+1,     R_Move::w2_mvW0);
	const CNF::Var Here_will_C2R_accN  = var(v,    t+1,     R_Move::w2_accN);
	const CNF::Var Here_will_C2R_mvN1  = var(v,    t+1,     R_Move::w2_mvN1);
	const CNF::Var Here_will_C2R_mvN2  = var(v,    t+1,     R_Move::w2_mvN2);
	const CNF::Var Here_will_C2R_mvN3  = var(v,    t+1,     R_Move::w2_mvN3);
	const CNF::Var Here_will_C2R_mvN0  = var(v,    t+1,     R_Move::w2_mvN0);
	const CNF::Var Here_will_C2R_accS  = var(v,    t+1,     R_Move::w2_accS);
	const CNF::Var Here_will_C2R_mvS1  = var(v,    t+1,     R_Move::w2_mvS1);
	const CNF::Var Here_will_C2R_mvS2  = var(v,    t+1,     R_Move::w2_mvS2);
	const CNF::Var Here_will_C2R_mvS3  = var(v,    t+1,     R_Move::w2_mvS3);
	const CNF::Var Here_will_C2R_mvS0  = var(v,    t+1,     R_Move::w2_mvS0);

	//const CNF::Var E_will_nobodyhome= var( G.east(v),    t+1,    NdStat::nobodyhome);
	const CNF::Var E_will_R_ready   = var( G.east(v),    t+1,    NdStat::R_ready);
	//const CNF::Var E_will_R_accE    = var( G.east(v),    t+1,    R_Move::accE);
	const CNF::Var E_will_R_mvE0    = var( G.east(v),    t+1,    R_Move::mvE0);
	//const CNF::Var E_will_R_accW    = var( G.east(v),    t+1,    R_Move::accW);
	//const CNF::Var E_will_R_mvW0    = var( G.east(v),    t+1,    R_Move::mvW0);
	const CNF::Var E_will_C0R_ready = var( G.east(v),    t+1,    NdStat::C0R_ready);
	//const CNF::Var E_will_C0R_accE  = var( G.east(v),    t+1,    R_Move::w0_accE);
	//const CNF::Var E_will_C0R_mvE1  = var( G.east(v),    t+1,    R_Move::w0_mvE1);
	const CNF::Var E_will_C0R_mvE0  = var( G.east(v),    t+1,    R_Move::w0_mvE0);
	//const CNF::Var E_will_C0R_accW  = var( G.east(v),    t+1,    R_Move::w0_accW);
	//const CNF::Var E_will_C0R_mvW1  = var( G.east(v),    t+1,    R_Move::w0_mvW1);
	//const CNF::Var E_will_C0R_mvW0  = var( G.east(v),    t+1,    R_Move::w0_mvW0);
	const CNF::Var E_will_C1R_ready = var( G.east(v),    t+1,    NdStat::C1R_ready);
	//const CNF::Var E_will_C1R_accE  = var( G.east(v),    t+1,    R_Move::w1_accE);
	//const CNF::Var E_will_C1R_mvE1  = var( G.east(v),    t+1,    R_Move::w1_mvE1);
	const CNF::Var E_will_C1R_mvE0  = var( G.east(v),    t+1,    R_Move::w1_mvE0);
	//const CNF::Var E_will_C1R_accW  = var( G.east(v),    t+1,    R_Move::w1_accW);
	//const CNF::Var E_will_C1R_mvW1  = var( G.east(v),    t+1,    R_Move::w1_mvW1);
	//const CNF::Var E_will_C1R_mvW0  = var( G.east(v),    t+1,    R_Move::w1_mvW0);
	const CNF::Var E_will_C2R_ready = var( G.east(v),    t+1,    NdStat::C2R_ready);
	//const CNF::Var E_will_C2R_accE  = var( G.east(v),    t+1,    R_Move::w2_accE);
	//const CNF::Var E_will_C2R_mvE1  = var( G.east(v),    t+1,    R_Move::w2_mvE1);
	const CNF::Var E_will_C2R_mvE0  = var( G.east(v),    t+1,    R_Move::w2_mvE0);
	//const CNF::Var E_will_C2R_accW  = var( G.east(v),    t+1,    R_Move::w2_accW);
	//const CNF::Var E_will_C2R_mvW1  = var( G.east(v),    t+1,    R_Move::w2_mvW1);
	//const CNF::Var E_will_C2R_mvW0  = var( G.east(v),    t+1,    R_Move::w2_mvW0);

	//const CNF::Var N_will_nobodyhome= var( G.north(v),    t+1,    NdStat::nobodyhome);
	const CNF::Var N_will_R_ready   = var( G.north(v),    t+1,    NdStat::R_ready);
	//const CNF::Var N_will_R_accN    = var( G.north(v),    t+1,    R_Move::accN);
	//const CNF::Var N_will_R_mvN1    = var( G.north(v),    t+1,    R_Move::mvN1);
	const CNF::Var N_will_R_mvN0    = var( G.north(v),    t+1,    R_Move::mvN0);
	//const CNF::Var N_will_R_accS    = var( G.north(v),    t+1,    R_Move::accS);
	//const CNF::Var N_will_R_mvS1    = var( G.north(v),    t+1,    R_Move::mvS1);
	//const CNF::Var N_will_R_mvS0    = var( G.north(v),    t+1,    R_Move::mvS0);
	const CNF::Var N_will_C0R_ready = var( G.north(v),    t+1,    NdStat::C0R_ready);
	//const CNF::Var N_will_C0R_accN  = var( G.north(v),    t+1,    R_Move::w0_accN);
	//const CNF::Var N_will_C0R_mvN1  = var( G.north(v),    t+1,    R_Move::w0_mvN1);
	//const CNF::Var N_will_C0R_mvN2  = var( G.north(v),    t+1,    R_Move::w0_mvN2);
	//const CNF::Var N_will_C0R_mvN3  = var( G.north(v),    t+1,    R_Move::w0_mvN3);
	const CNF::Var N_will_C0R_mvN0  = var( G.north(v),    t+1,    R_Move::w0_mvN0);
	//const CNF::Var N_will_C0R_accS  = var( G.north(v),    t+1,    R_Move::w0_accS);
	//const CNF::Var N_will_C0R_mvS1  = var( G.north(v),    t+1,    R_Move::w0_mvS1);
	//const CNF::Var N_will_C0R_mvS2  = var( G.north(v),    t+1,    R_Move::w0_mvS2);
	//const CNF::Var N_will_C0R_mvS3  = var( G.north(v),    t+1,    R_Move::w0_mvS3);
	//const CNF::Var N_will_C0R_mvS0  = var( G.north(v),    t+1,    R_Move::w0_mvS0);
	const CNF::Var N_will_C1R_ready = var( G.north(v),    t+1,    NdStat::C1R_ready);
	//const CNF::Var N_will_C1R_accN  = var( G.north(v),    t+1,    R_Move::w1_accN);
	//const CNF::Var N_will_C1R_mvN1  = var( G.north(v),    t+1,    R_Move::w1_mvN1);
	//const CNF::Var N_will_C1R_mvN2  = var( G.north(v),    t+1,    R_Move::w1_mvN2);
	//const CNF::Var N_will_C1R_mvN3  = var( G.north(v),    t+1,    R_Move::w1_mvN3);
	const CNF::Var N_will_C1R_mvN0  = var( G.north(v),    t+1,    R_Move::w1_mvN0);
	//const CNF::Var N_will_C1R_accS  = var( G.north(v),    t+1,    R_Move::w1_accS);
	//const CNF::Var N_will_C1R_mvS1  = var( G.north(v),    t+1,    R_Move::w1_mvS1);
	//const CNF::Var N_will_C1R_mvS2  = var( G.north(v),    t+1,    R_Move::w1_mvS2);
	//const CNF::Var N_will_C1R_mvS3  = var( G.north(v),    t+1,    R_Move::w1_mvS3);
	//const CNF::Var N_will_C1R_mvS0  = var( G.north(v),    t+1,    R_Move::w1_mvS0);
	const CNF::Var N_will_C2R_ready = var( G.north(v),    t+1,    NdStat::C2R_ready);
	//const CNF::Var N_will_C2R_accN  = var( G.north(v),    t+1,    R_Move::w2_accN);
	//const CNF::Var N_will_C2R_mvN1  = var( G.north(v),    t+1,    R_Move::w2_mvN1);
	//const CNF::Var N_will_C2R_mvN2  = var( G.north(v),    t+1,    R_Move::w2_mvN2);
	//const CNF::Var N_will_C2R_mvN3  = var( G.north(v),    t+1,    R_Move::w2_mvN3);
	const CNF::Var N_will_C2R_mvN0  = var( G.north(v),    t+1,    R_Move::w2_mvN0);
	//const CNF::Var N_will_C2R_accS  = var( G.north(v),    t+1,    R_Move::w2_accS);
	//const CNF::Var N_will_C2R_mvS1  = var( G.north(v),    t+1,    R_Move::w2_mvS1);
	//const CNF::Var N_will_C2R_mvS2  = var( G.north(v),    t+1,    R_Move::w2_mvS2);
	//const CNF::Var N_will_C2R_mvS3  = var( G.north(v),    t+1,    R_Move::w2_mvS3);
	//const CNF::Var N_will_C2R_mvS0  = var( G.north(v),    t+1,    R_Move::w2_mvS0);

	//const CNF::Var W_will_nobodyhome= var( G.west(v),    t+1,    NdStat::nobodyhome);
	const CNF::Var W_will_R_ready   = var( G.west(v),    t+1,    NdStat::R_ready);
	//const CNF::Var W_will_R_accE    = var( G.west(v),    t+1,    R_Move::accE);
	const CNF::Var W_will_R_mvE0    = var( G.west(v),    t+1,    R_Move::mvE0);
	//const CNF::Var W_will_R_accW    = var( G.west(v),    t+1,    R_Move::accW);
	const CNF::Var W_will_R_mvW0    = var( G.west(v),    t+1,    R_Move::mvW0);
	const CNF::Var W_will_C0R_ready = var( G.west(v),    t+1,    NdStat::C0R_ready);
	//const CNF::Var W_will_C0R_accE  = var( G.west(v),    t+1,    R_Move::w0_accE);
	//const CNF::Var W_will_C0R_mvE1  = var( G.west(v),    t+1,    R_Move::w0_mvE1);
	//const CNF::Var W_will_C0R_mvE0  = var( G.west(v),    t+1,    R_Move::w0_mvE0);
	//const CNF::Var W_will_C0R_accW  = var( G.west(v),    t+1,    R_Move::w0_accW);
	//const CNF::Var W_will_C0R_mvW1  = var( G.west(v),    t+1,    R_Move::w0_mvW1);
	const CNF::Var W_will_C0R_mvW0  = var( G.west(v),    t+1,    R_Move::w0_mvW0);
	const CNF::Var W_will_C1R_ready = var( G.west(v),    t+1,    NdStat::C1R_ready);
	//const CNF::Var W_will_C1R_accE  = var( G.west(v),    t+1,    R_Move::w1_accE);
	//const CNF::Var W_will_C1R_mvE1  = var( G.west(v),    t+1,    R_Move::w1_mvE1);
	//const CNF::Var W_will_C1R_mvE0  = var( G.west(v),    t+1,    R_Move::w1_mvE0);
	//const CNF::Var W_will_C1R_accW  = var( G.west(v),    t+1,    R_Move::w1_accW);
	//const CNF::Var W_will_C1R_mvW1  = var( G.west(v),    t+1,    R_Move::w1_mvW1);
	const CNF::Var W_will_C1R_mvW0  = var( G.west(v),    t+1,    R_Move::w1_mvW0);
	const CNF::Var W_will_C2R_ready = var( G.west(v),    t+1,    NdStat::C2R_ready);
	//const CNF::Var W_will_C2R_accE  = var( G.west(v),    t+1,    R_Move::w2_accE);
	//const CNF::Var W_will_C2R_mvE1  = var( G.west(v),    t+1,    R_Move::w2_mvE1);
	//const CNF::Var W_will_C2R_mvE0  = var( G.west(v),    t+1,    R_Move::w2_mvE0);
	//const CNF::Var W_will_C2R_accW  = var( G.west(v),    t+1,    R_Move::w2_accW);
	//const CNF::Var W_will_C2R_mvW1  = var( G.west(v),    t+1,    R_Move::w2_mvW1);
	const CNF::Var W_will_C2R_mvW0  = var( G.west(v),    t+1,    R_Move::w2_mvW0);

	//const CNF::Var S_will_nobodyhome= var( G.south(v),    t+1,    NdStat::nobodyhome);
	const CNF::Var S_will_R_ready   = var( G.south(v),    t+1,    NdStat::R_ready);
	//const CNF::Var S_will_R_accN    = var( G.south(v),    t+1,    R_Move::accN);
	//const CNF::Var S_will_R_mvN1    = var( G.south(v),    t+1,    R_Move::mvN1);
	//const CNF::Var S_will_R_mvN0    = var( G.south(v),    t+1,    R_Move::mvN0);
	//const CNF::Var S_will_R_accS    = var( G.south(v),    t+1,    R_Move::accS);
	//const CNF::Var S_will_R_mvS1    = var( G.south(v),    t+1,    R_Move::mvS1);
	const CNF::Var S_will_R_mvS0    = var( G.south(v),    t+1,    R_Move::mvS0);
	const CNF::Var S_will_C0R_ready = var( G.south(v),    t+1,    NdStat::C0R_ready);
	//const CNF::Var S_will_C0R_accN  = var( G.south(v),    t+1,    R_Move::w0_accN);
	//const CNF::Var S_will_C0R_mvN1  = var( G.south(v),    t+1,    R_Move::w0_mvN1);
	//const CNF::Var S_will_C0R_mvN2  = var( G.south(v),    t+1,    R_Move::w0_mvN2);
	//const CNF::Var S_will_C0R_mvN3  = var( G.south(v),    t+1,    R_Move::w0_mvN3);
	//const CNF::Var S_will_C0R_mvN0  = var( G.south(v),    t+1,    R_Move::w0_mvN0);
	//const CNF::Var S_will_C0R_accS  = var( G.south(v),    t+1,    R_Move::w0_accS);
	//const CNF::Var S_will_C0R_mvS1  = var( G.south(v),    t+1,    R_Move::w0_mvS1);
	//const CNF::Var S_will_C0R_mvS2  = var( G.south(v),    t+1,    R_Move::w0_mvS2);
	//const CNF::Var S_will_C0R_mvS3  = var( G.south(v),    t+1,    R_Move::w0_mvS3);
	const CNF::Var S_will_C0R_mvS0  = var( G.south(v),    t+1,    R_Move::w0_mvS0);
	const CNF::Var S_will_C1R_ready = var( G.south(v),    t+1,    NdStat::C1R_ready);
	//const CNF::Var S_will_C1R_accN  = var( G.south(v),    t+1,    R_Move::w1_accN);
	//const CNF::Var S_will_C1R_mvN1  = var( G.south(v),    t+1,    R_Move::w1_mvN1);
	//const CNF::Var S_will_C1R_mvN2  = var( G.south(v),    t+1,    R_Move::w1_mvN2);
	//const CNF::Var S_will_C1R_mvN3  = var( G.south(v),    t+1,    R_Move::w1_mvN3);
	//const CNF::Var S_will_C1R_mvN0  = var( G.south(v),    t+1,    R_Move::w1_mvN0);
	//const CNF::Var S_will_C1R_accS  = var( G.south(v),    t+1,    R_Move::w1_accS);
	//const CNF::Var S_will_C1R_mvS1  = var( G.south(v),    t+1,    R_Move::w1_mvS1);
	//const CNF::Var S_will_C1R_mvS2  = var( G.south(v),    t+1,    R_Move::w1_mvS2);
	//const CNF::Var S_will_C1R_mvS3  = var( G.south(v),    t+1,    R_Move::w1_mvS3);
	const CNF::Var S_will_C1R_mvS0  = var( G.south(v),    t+1,    R_Move::w1_mvS0);
	const CNF::Var S_will_C2R_ready = var( G.south(v),    t+1,    NdStat::C2R_ready);
	//const CNF::Var S_will_C2R_accN  = var( G.south(v),    t+1,    R_Move::w2_accN);
	//const CNF::Var S_will_C2R_mvN1  = var( G.south(v),    t+1,    R_Move::w2_mvN1);
	//const CNF::Var S_will_C2R_mvN2  = var( G.south(v),    t+1,    R_Move::w2_mvN2);
	//const CNF::Var S_will_C2R_mvN3  = var( G.south(v),    t+1,    R_Move::w2_mvN3);
	//const CNF::Var S_will_C2R_mvN0  = var( G.south(v),    t+1,    R_Move::w2_mvN0);
	//const CNF::Var S_will_C2R_accS  = var( G.south(v),    t+1,    R_Move::w2_accS);
	//const CNF::Var S_will_C2R_mvS1  = var( G.south(v),    t+1,    R_Move::w2_mvS1);
	//const CNF::Var S_will_C2R_mvS2  = var( G.south(v),    t+1,    R_Move::w2_mvS2);
	//const CNF::Var S_will_C2R_mvS3  = var( G.south(v),    t+1,    R_Move::w2_mvS3);
	const CNF::Var S_will_C2R_mvS0  = var( G.south(v),    t+1,    R_Move::w2_mvS0);


	

        // Robot alone
        c = not(Here_will_R_accE) or Here_now_R_ready;
	model.addClause(c);//
	c = not(Here_will_R_mvE0) or W_now_R_accE              or W_now_R_mvE0;
	model.addClause(c);//
	c = not(Here_now_R_mvE0)  or Here_now_R_accE           or E_will_R_ready       or E_will_R_mvE0;
	model.addClause(c);
	c = Here_now_R_mvE0       or not(Here_now_R_accE)      or E_will_R_ready       or E_will_R_mvE0;
	model.addClause(c);
	c = not(Here_now_R_mvE0)  or not(Here_now_R_accE);
	model.addClause(c);//
	c = not(Here_now_R_accE)  or Here_will_nobodyhome      or Here_will_R_mvE0;
	model.addClause(c);//
	c = not(Here_will_R_accE) or E_now_nobodyhome          or E_now_R_accE                                                                           or E_now_C0R_mvE1            or E_now_C1R_mvE1       or E_now_C2R_mvE1;
	model.addClause(c);//
	c = E_will_R_ready        or not(Here_now_R_accE)      or not(E_now_C0R_mvE1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = E_will_R_ready        or not(Here_now_R_accE)      or not(E_now_C1R_mvE1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = E_will_R_ready        or not(Here_now_R_accE)      or not(E_now_C2R_mvE1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive	
	c = E_will_R_ready        or not(Here_now_R_mvE0)      or not(E_now_C0R_mvE1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = E_will_R_ready        or not(Here_now_R_mvE0)      or not(E_now_C1R_mvE1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = E_will_R_ready        or not(Here_now_R_mvE0)      or not(E_now_C2R_mvE1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive

	
	c = not(Here_will_R_accW) or Here_now_R_ready;
	model.addClause(c);//
	c = not(Here_will_R_mvW0) or E_now_R_accW              or E_now_R_mvW0;
	model.addClause(c);//
	c = not(Here_now_R_mvW0)  or Here_now_R_accW           or W_will_R_ready       or W_will_R_mvE0;
	model.addClause(c);
	c = Here_now_R_mvW0       or not(Here_now_R_accW)      or W_will_R_ready       or W_will_R_mvW0;
	model.addClause(c);
	c = not(Here_now_R_mvW0)  or not(Here_now_R_accW);
	model.addClause(c);//
	c = not(Here_now_R_accW)  or Here_will_nobodyhome      or Here_will_R_mvW0;
	model.addClause(c);//
	c = not(Here_will_R_accW) or W_now_nobodyhome          or W_now_R_accW                                                                           or W_now_C0R_mvW1            or W_now_C1R_mvW1       or W_now_C2R_mvW1;
	model.addClause(c);//
	c = W_will_R_ready        or not(Here_now_R_accW)      or not(W_now_C0R_mvW1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = W_will_R_ready        or not(Here_now_R_accW)      or not(W_now_C1R_mvW1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = W_will_R_ready        or not(Here_now_R_accW)      or not(W_now_C2R_mvW1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = W_will_R_ready        or not(Here_now_R_mvW0)      or not(W_now_C0R_mvW1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = W_will_R_ready        or not(Here_now_R_mvW0)      or not(W_now_C1R_mvW1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = W_will_R_ready        or not(Here_now_R_mvW0)      or not(W_now_C2R_mvW1);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive


	
	c = not(Here_will_R_accN) or Here_now_R_ready;
	model.addClause(c);//
	// neccessarly
	c = not(Here_will_R_mvN1) or Here_now_R_accN           or Here_now_R_mvN0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_R_accN)  or Here_now_R_mvN0           or Here_will_R_mvN1;
	model.addClause(c);
	c = Here_now_R_accN       or not(Here_now_R_mvN0)      or Here_will_R_mvN1;
	model.addClause(c);
	c = not(Here_now_R_accN)  or not(Here_now_R_mvN0);
	model.addClause(c);//	
	c = not(Here_will_R_mvN0) or S_now_R_mvN1;
	model.addClause(c);//
	c = not(Here_now_R_mvN0)  or Here_now_R_accN           or Here_will_R_mvN1;
	model.addClause(c);
	c = Here_now_R_mvN0       or not(Here_now_R_accN)      or Here_will_R_mvN1;
	model.addClause(c);
	c = not(Here_now_R_mvN0)  or not(Here_now_R_accN);
	model.addClause(c);//
	c = not(Here_now_R_mvN1)  or N_will_R_mvN0             or N_will_R_ready;
	model.addClause(c);//
	c = not(Here_now_R_mvN1)  or Here_will_nobodyhome      or Here_will_R_mvN0;
	model.addClause(c);//
	c = N_will_R_ready        or not(Here_now_R_mvN1)      or not(N_now_C0R_mvN3);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = N_will_R_ready        or not(Here_now_R_mvN1)      or not(N_now_C1R_mvN3);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = N_will_R_ready        or not(Here_now_R_mvN1)      or not(N_now_C2R_mvN3);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive	

	
	c = not(Here_will_R_accS) or Here_now_R_ready;
	model.addClause(c);//
	// neccessarly
	c = not(Here_will_R_mvS1) or Here_now_R_accS           or Here_now_R_mvS0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_R_accS)  or Here_now_R_mvS0           or Here_will_R_mvS1;
	model.addClause(c);
	c = Here_now_R_accS       or not(Here_now_R_mvS0)      or Here_will_R_mvS1;
	model.addClause(c);
	c = not(Here_now_R_accS)  or not(Here_now_R_mvS0);
	model.addClause(c);//!!!!!!!!!!!!!!
	c = not(Here_will_R_mvS0) or N_now_R_mvS1;
	model.addClause(c);//
	c = not(Here_now_R_mvS0)  or Here_now_R_accS           or Here_will_R_mvS1;
	model.addClause(c);
	c = Here_now_R_mvS0       or not(Here_now_R_accS)      or Here_will_R_mvS1;
	model.addClause(c);
	c = not(Here_now_R_mvS0)  or not(Here_now_R_accS);
	model.addClause(c);//
	c = not(Here_now_R_mvS1)  or S_will_R_mvS0             or S_will_R_ready;
	model.addClause(c);//
	c = not(Here_now_R_mvS1)  or Here_will_nobodyhome      or Here_will_R_mvS0;
	model.addClause(c);//
	c = S_will_R_ready        or not(Here_now_R_mvS1)      or not(S_now_C0R_mvS3);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = S_will_R_ready        or not(Here_now_R_mvS1)      or not(S_now_C1R_mvS3);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = S_will_R_ready        or not(Here_now_R_mvS1)      or not(S_now_C2R_mvS3);
	model.addClause(c);// Crobot ahead.  Hint: contrapositive

        // C r o b o t  0
	c = not(Here_will_C0R_accN) or Here_now_C0R_ready;
	model.addClause(c); //
	// neccesarly
	c = not(Here_will_C0R_mvN1) or Here_now_C0R_accN       or Here_now_C0R_mvN0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accN)  or Here_now_C0R_mvN0       or Here_will_C0R_mvN1;
	model.addClause(c);
	c = Here_now_C0R_accN       or not(Here_now_C0R_mvN0)  or Here_will_C0R_mvN1;
	model.addClause(c);
	c = not(Here_now_C0R_accN)  or not(Here_now_C0R_mvN0);
	model.addClause(c); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C0R_mvN2) or Here_now_C0R_mvN1;
	model.addClause(c);
	c = not(Here_now_C0R_mvN1)  or Here_will_C0R_mvN2;
	model.addClause(c); //
	c = not(Here_will_C0R_mvN3) or Here_now_C0R_mvN2;
	model.addClause(c);
	c = not(Here_now_C0R_mvN2)  or Here_will_C0R_mvN3;
	model.addClause(c); //
	c = not(Here_will_C0R_mvN0) or S_now_C0R_mvN3;
	model.addClause(c); //
	c = not(Here_now_C0R_accN)  or Here_now_C0R_mvN0       or Here_will_C0R_mvN1;
	model.addClause(c);
	c = Here_now_C0R_accN       or not(Here_now_C0R_mvN0)  or Here_will_C0R_mvN1;
	model.addClause(c);
	c = not(Here_now_C0R_accN)  or not(Here_now_C0R_mvN0);
	model.addClause(c); //
	c = not(Here_now_C0R_mvN3)  or N_will_C0R_ready        or N_will_C0R_mvN0;
	model.addClause(c); //
	c = not(Here_now_C0R_mvN3)  or Here_will_nobodyhome    or Here_will_R_mvN0;
	model.addClause(c); //

	
	c = not(Here_will_C0R_accS) or Here_now_C0R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C0R_mvS1) or Here_now_C0R_accS       or Here_now_C0R_mvS0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accS)  or  Here_now_C0R_mvS0      or Here_will_C0R_mvS1;
	model.addClause(c);
	c = Here_now_C0R_accS       or not(Here_now_C0R_mvS0)  or Here_will_C0R_mvS1;
	model.addClause(c);
	c = not(Here_now_C0R_accS)  or not(Here_now_C0R_mvS0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C0R_mvS2) or Here_now_C0R_mvS1;
	model.addClause(c);
	c = not(Here_now_C0R_mvS1)  or Here_will_C0R_mvS2;
	model.addClause(c); //
	c = not(Here_will_C0R_mvS3) or Here_now_C0R_mvS2;
	model.addClause(c);
	c = not(Here_now_C0R_mvS2)  or Here_will_C0R_mvS3;
	model.addClause(c); //
	c = not(Here_will_C0R_mvS0) or N_now_C0R_mvS3;
	model.addClause(c); //
	c = not(Here_now_C0R_accS)  or Here_now_C0R_mvS0       or Here_will_C0R_mvS1;
	model.addClause(c);
	c = Here_now_C0R_accS       or not(Here_now_C0R_mvS0)  or Here_will_C0R_mvS1;
	model.addClause(c);
	c = not(Here_now_C0R_accS)  or not(Here_now_C0R_mvS0);
	model.addClause(c); //
	c = not(Here_now_C0R_mvS3)  or S_will_C0R_ready        or S_will_C0R_mvS0;
	model.addClause(c); //
	c = not(Here_now_C0R_mvS3)  or Here_will_nobodyhome    or Here_will_R_mvS0;
	model.addClause(c); //


	
	c = not(Here_will_C0R_accE) or Here_now_C0R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C0R_mvE1) or Here_now_C0R_accE       or Here_now_C0R_mvE0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accE)  or Here_now_C0R_mvE0       or Here_will_C0R_mvE1;
	model.addClause(c);
	c = Here_now_C0R_accE       or not(Here_now_C0R_mvE0)  or Here_will_C0R_mvE1;
	model.addClause(c);
	c = not(Here_now_C0R_accE)  or not(Here_now_C0R_mvE0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C0R_mvE0) or W_now_C0R_mvE1;
	model.addClause(c); //
	c = not(Here_now_C0R_accE)  or Here_now_C0R_mvE0       or Here_will_C0R_mvE1;
	model.addClause(c);
	c = Here_now_C0R_accE       or not(Here_now_C0R_mvE0)  or Here_will_C0R_mvE1;
	model.addClause(c);
	c = not(Here_now_C0R_accE)  or not(Here_now_C0R_mvE0);
	model.addClause(c); //
	c = not(Here_now_C0R_mvE1)  or E_will_C0R_ready        or E_will_C0R_mvE0;
	model.addClause(c); //
	c = not(Here_now_C0R_mvE1)  or Here_will_nobodyhome    or Here_will_R_mvE0;
	model.addClause(c); //


	
	c = not(Here_will_C0R_accW) or Here_now_C0R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C0R_mvW1) or Here_now_C0R_accW       or Here_now_C0R_mvW0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accW)  or Here_now_C0R_mvW0       or Here_will_C0R_mvW1;
	model.addClause(c);
	c = Here_now_C0R_accW       or not(Here_now_C0R_mvW0)  or Here_will_C0R_mvW1;
	model.addClause(c);
	c = not(Here_now_C0R_accW)  or not(Here_now_C0R_mvW0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C0R_mvW0) or E_now_C0R_mvW1;
	model.addClause(c); //
	c = not(Here_now_C0R_accW)  or Here_now_C0R_mvW0       or Here_will_C0R_mvW1;
	model.addClause(c);
	c = Here_now_C0R_accW       or not(Here_now_C0R_mvW0)  or Here_will_C0R_mvW1;
	model.addClause(c);
	c = not(Here_now_C0R_accW)  or not(Here_now_C0R_mvW0);
	model.addClause(c); //
	c = not(Here_now_C0R_mvW1)  or W_will_C0R_ready        or W_will_C0R_mvW0;
	model.addClause(c); //
	c = not(Here_now_C0R_mvW1)  or Here_will_nobodyhome    or Here_will_R_mvW0;
	model.addClause(c); //


	
        // C r o b o t  1
	c = not(Here_will_C1R_accN) or Here_now_C1R_ready;
	model.addClause(c); //
	// neccesarly
	c = not(Here_will_C1R_mvN1) or Here_now_C1R_accN       or Here_now_C1R_mvN0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C1R_accN)  or Here_now_C1R_mvN0       or Here_will_C1R_mvN1;
	model.addClause(c);
	c = Here_now_C1R_accN       or not(Here_now_C1R_mvN0)  or Here_will_C1R_mvN1;
	model.addClause(c);
	c = not(Here_now_C1R_accN)  or not(Here_now_C1R_mvN0);
	model.addClause(c); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C1R_mvN2) or Here_now_C1R_mvN1;
	model.addClause(c);
	c = not(Here_now_C1R_mvN1)  or Here_will_C1R_mvN2;
	model.addClause(c); //
	c = not(Here_will_C1R_mvN3) or Here_now_C1R_mvN2;
	model.addClause(c);
	c = not(Here_now_C1R_mvN2)  or Here_will_C1R_mvN3;
	model.addClause(c); //
	c = not(Here_will_C1R_mvN0) or S_now_C1R_mvN3;
	model.addClause(c); //
	c = not(Here_now_C1R_accN)  or Here_now_C1R_mvN0       or Here_will_C1R_mvN1;
	model.addClause(c);
	c = Here_now_C1R_accN       or not(Here_now_C1R_mvN0)  or Here_will_C1R_mvN1;
	model.addClause(c);
	c = not(Here_now_C1R_accN)  or not(Here_now_C1R_mvN0);
	model.addClause(c); //
	c = not(Here_now_C1R_mvN3)  or N_will_C1R_ready        or N_will_C1R_mvN0;
	model.addClause(c); //
	c = not(Here_now_C1R_mvN3)  or Here_will_nobodyhome    or Here_will_R_mvN0;
	model.addClause(c); //

	
	c = not(Here_will_C1R_accS) or Here_now_C1R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C1R_mvS1) or Here_now_C1R_accS       or Here_now_C1R_mvS0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C1R_accS)  or  Here_now_C1R_mvS0      or Here_will_C1R_mvS1;
	model.addClause(c);
	c = Here_now_C1R_accS       or not(Here_now_C1R_mvS0)  or Here_will_C1R_mvS1;
	model.addClause(c);
	c = not(Here_now_C1R_accS)  or not(Here_now_C1R_mvS0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C1R_mvS2) or Here_now_C1R_mvS1;
	model.addClause(c);
	c = not(Here_now_C1R_mvS1)  or Here_will_C1R_mvS2;
	model.addClause(c); //
	c = not(Here_will_C1R_mvS3) or Here_now_C1R_mvS2;
	model.addClause(c);
	c = not(Here_now_C1R_mvS2)  or Here_will_C1R_mvS3;
	model.addClause(c); //
	c = not(Here_will_C1R_mvS0) or N_now_C1R_mvS3;
	model.addClause(c); //
	c = not(Here_now_C1R_accS)  or Here_now_C1R_mvS0       or Here_will_C1R_mvS1;
	model.addClause(c);
	c = Here_now_C1R_accS       or not(Here_now_C1R_mvS0)  or Here_will_C1R_mvS1;
	model.addClause(c);
	c = not(Here_now_C1R_accS)  or not(Here_now_C1R_mvS0);
	model.addClause(c); //
	c = not(Here_now_C1R_mvS3)  or S_will_C1R_ready        or S_will_C1R_mvS0;
	model.addClause(c); //
	c = not(Here_now_C1R_mvS3)  or Here_will_nobodyhome    or Here_will_R_mvS0;
	model.addClause(c); //


	
	c = not(Here_will_C1R_accE) or Here_now_C1R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C1R_mvE1) or Here_now_C1R_accE       or Here_now_C1R_mvE0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C1R_accE)  or Here_now_C1R_mvE0       or Here_will_C1R_mvE1;
	model.addClause(c);
	c = Here_now_C1R_accE       or not(Here_now_C1R_mvE0)  or Here_will_C1R_mvE1;
	model.addClause(c);
	c = not(Here_now_C1R_accE)  or not(Here_now_C1R_mvE0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C1R_mvE0) or W_now_C1R_mvE1;
	model.addClause(c); //
	c = not(Here_now_C1R_accE)  or Here_now_C1R_mvE0       or Here_will_C1R_mvE1;
	model.addClause(c);
	c = Here_now_C1R_accE       or not(Here_now_C1R_mvE0)  or Here_will_C1R_mvE1;
	model.addClause(c);
	c = not(Here_now_C1R_accE)  or not(Here_now_C1R_mvE0);
	model.addClause(c); //
	c = not(Here_now_C1R_mvE1)  or E_will_C1R_ready        or E_will_C1R_mvE0;
	model.addClause(c); //
	c = not(Here_now_C1R_mvE1)  or Here_will_nobodyhome    or Here_will_R_mvE0;
	model.addClause(c); //


	
	c = not(Here_will_C1R_accW) or Here_now_C1R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C1R_mvW1) or Here_now_C1R_accW       or Here_now_C1R_mvW0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C1R_accW)  or Here_now_C1R_mvW0       or Here_will_C1R_mvW1;
	model.addClause(c);
	c = Here_now_C1R_accW       or not(Here_now_C1R_mvW0)  or Here_will_C1R_mvW1;
	model.addClause(c);
	c = not(Here_now_C1R_accW)  or not(Here_now_C1R_mvW0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C1R_mvW0) or E_now_C1R_mvW1;
	model.addClause(c); //
	c = not(Here_now_C1R_accW)  or Here_now_C1R_mvW0       or Here_will_C1R_mvW1;
	model.addClause(c);
	c = Here_now_C1R_accW       or not(Here_now_C1R_mvW0)  or Here_will_C1R_mvW1;
	model.addClause(c);
	c = not(Here_now_C1R_accW)  or not(Here_now_C1R_mvW0);
	model.addClause(c); //
	c = not(Here_now_C1R_mvW1)  or W_will_C1R_ready        or W_will_C1R_mvW0;
	model.addClause(c); //
	c = not(Here_now_C1R_mvW1)  or Here_will_nobodyhome    or Here_will_R_mvW0;
	model.addClause(c); //




        // C r o b o t  2
	c = not(Here_will_C2R_accN) or Here_now_C2R_ready;
	model.addClause(c); //
	// neccesarly
	c = not(Here_will_C2R_mvN1) or Here_now_C2R_accN       or Here_now_C2R_mvN0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C2R_accN)  or Here_now_C2R_mvN0       or Here_will_C2R_mvN1;
	model.addClause(c);
	c = Here_now_C2R_accN       or not(Here_now_C2R_mvN0)  or Here_will_C2R_mvN1;
	model.addClause(c);
	c = not(Here_now_C2R_accN)  or not(Here_now_C2R_mvN0);
	model.addClause(c); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C2R_mvN2) or Here_now_C2R_mvN1;
	model.addClause(c);
	c = not(Here_now_C2R_mvN1)  or Here_will_C2R_mvN2;
	model.addClause(c); //
	c = not(Here_will_C2R_mvN3) or Here_now_C2R_mvN2;
	model.addClause(c);
	c = not(Here_now_C2R_mvN2)  or Here_will_C2R_mvN3;
	model.addClause(c); //
	c = not(Here_will_C2R_mvN0) or S_now_C2R_mvN3;
	model.addClause(c); //
	c = not(Here_now_C2R_accN)  or Here_now_C2R_mvN0       or Here_will_C2R_mvN1;
	model.addClause(c);
	c = Here_now_C2R_accN       or not(Here_now_C2R_mvN0)  or Here_will_C2R_mvN1;
	model.addClause(c);
	c = not(Here_now_C2R_accN)  or not(Here_now_C2R_mvN0);
	model.addClause(c); //
	c = not(Here_now_C2R_mvN3)  or N_will_C2R_ready        or N_will_C2R_mvN0;
	model.addClause(c); //
	c = not(Here_now_C2R_mvN3)  or Here_will_nobodyhome    or Here_will_R_mvN0;
	model.addClause(c); //

	
	c = not(Here_will_C2R_accS) or Here_now_C2R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C2R_mvS1) or Here_now_C2R_accS       or Here_now_C2R_mvS0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C2R_accS)  or  Here_now_C2R_mvS0      or Here_will_C2R_mvS1;
	model.addClause(c);
	c = Here_now_C2R_accS       or not(Here_now_C2R_mvS0)  or Here_will_C2R_mvS1;
	model.addClause(c);
	c = not(Here_now_C2R_accS)  or not(Here_now_C2R_mvS0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C2R_mvS2) or Here_now_C2R_mvS1;
	model.addClause(c);
	c = not(Here_now_C2R_mvS1)  or Here_will_C2R_mvS2;
	model.addClause(c); //
	c = not(Here_will_C2R_mvS3) or Here_now_C2R_mvS2;
	model.addClause(c);
	c = not(Here_now_C2R_mvS2)  or Here_will_C2R_mvS3;
	model.addClause(c); //
	c = not(Here_will_C2R_mvS0) or N_now_C2R_mvS3;
	model.addClause(c); //
	c = not(Here_now_C2R_accS)  or Here_now_C2R_mvS0       or Here_will_C2R_mvS1;
	model.addClause(c);
	c = Here_now_C2R_accS       or not(Here_now_C2R_mvS0)  or Here_will_C2R_mvS1;
	model.addClause(c);
	c = not(Here_now_C2R_accS)  or not(Here_now_C2R_mvS0);
	model.addClause(c); //
	c = not(Here_now_C2R_mvS3)  or S_will_C2R_ready        or S_will_C2R_mvS0;
	model.addClause(c); //
	c = not(Here_now_C2R_mvS3)  or Here_will_nobodyhome    or Here_will_R_mvS0;
	model.addClause(c); //


	
	c = not(Here_will_C2R_accE) or Here_now_C2R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C2R_mvE1) or Here_now_C2R_accE       or Here_now_C2R_mvE0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C2R_accE)  or Here_now_C2R_mvE0       or Here_will_C2R_mvE1;
	model.addClause(c);
	c = Here_now_C2R_accE       or not(Here_now_C2R_mvE0)  or Here_will_C2R_mvE1;
	model.addClause(c);
	c = not(Here_now_C2R_accE)  or not(Here_now_C2R_mvE0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C2R_mvE0) or W_now_C2R_mvE1;
	model.addClause(c); //
	c = not(Here_now_C2R_accE)  or Here_now_C2R_mvE0       or Here_will_C2R_mvE1;
	model.addClause(c);
	c = Here_now_C2R_accE       or not(Here_now_C2R_mvE0)  or Here_will_C2R_mvE1;
	model.addClause(c);
	c = not(Here_now_C2R_accE)  or not(Here_now_C2R_mvE0);
	model.addClause(c); //
	c = not(Here_now_C2R_mvE1)  or E_will_C2R_ready        or E_will_C2R_mvE0;
	model.addClause(c); //
	c = not(Here_now_C2R_mvE1)  or Here_will_nobodyhome    or Here_will_R_mvE0;
	model.addClause(c); //


	
	c = not(Here_will_C2R_accW) or Here_now_C2R_ready;
	model.addClause(c); //
	// neccessarly
	c = not(Here_will_C2R_mvW1) or Here_now_C2R_accW       or Here_now_C2R_mvW0;
	model.addClause(c);
	// sufficient
	c = not(Here_now_C2R_accW)  or Here_now_C2R_mvW0       or Here_will_C2R_mvW1;
	model.addClause(c);
	c = Here_now_C2R_accW       or not(Here_now_C2R_mvW0)  or Here_will_C2R_mvW1;
	model.addClause(c);
	c = not(Here_now_C2R_accW)  or not(Here_now_C2R_mvW0);
	model.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C2R_mvW0) or E_now_C2R_mvW1;
	model.addClause(c); //
	c = not(Here_now_C2R_accW)  or Here_now_C2R_mvW0       or Here_will_C2R_mvW1;
	model.addClause(c);
	c = Here_now_C2R_accW       or not(Here_now_C2R_mvW0)  or Here_will_C2R_mvW1;
	model.addClause(c);
	c = not(Here_now_C2R_accW)  or not(Here_now_C2R_mvW0);
	model.addClause(c); //
	c = not(Here_now_C2R_mvW1)  or W_will_C2R_ready        or W_will_C2R_mvW0;
	model.addClause(c); //
	c = not(Here_now_C2R_mvW1)  or Here_will_nobodyhome    or Here_will_R_mvW0;
	model.addClause(c); //

	model.dump(out);	
    }


    // L I F T I N G + D R O P P I N G
    { // Lifting process
    	CNF::Clause c;
        const CNF::Var Here_now_empty            = var(v,       t,    On_Node::empty);
	const CNF::Var Here_now_C0               = var(v,       t,    On_Node::Car0);
	const CNF::Var Here_now_C1               = var(v,       t,    On_Node::Car1);
	const CNF::Var Here_now_C2               = var(v,       t,    On_Node::Car2);

	const CNF::Var Here_now_R_ready          = var(v,       t,    NdStat::R_ready);
	const CNF::Var Here_now_R_lift           = var(v,       t,    R_Vertical::lift);
	const CNF::Var Here_now_R_lifting1       = var(v,       t,    R_Vertical::l1);
	const CNF::Var Here_now_R_lifting2       = var(v,       t,    R_Vertical::l2);
	const CNF::Var Here_now_R_lifting3       = var(v,       t,    R_Vertical::l3);
	const CNF::Var Here_now_R_lifting4       = var(v,       t,    R_Vertical::l4);

	const CNF::Var Here_will_R_lift          = var(v,       t+1,  R_Vertical::lift);
	const CNF::Var Here_will_R_lifting1      = var(v,       t+1,  R_Vertical::l1);
	const CNF::Var Here_will_R_lifting2      = var(v,       t+1,  R_Vertical::l2);
	const CNF::Var Here_will_R_lifting3      = var(v,       t+1,  R_Vertical::l3);
	const CNF::Var Here_will_R_lifting4      = var(v,       t+1,  R_Vertical::l4);

	const CNF::Var Here_will_C0R_ready       = var(v,       t+1,  NdStat::C0R_ready);
	const CNF::Var Here_will_C1R_ready       = var(v,       t+1,  NdStat::C1R_ready);
	const CNF::Var Here_will_C2R_ready       = var(v,       t+1,  NdStat::C2R_ready);


	c = not(Here_will_R_lift)     or Here_now_R_ready;
	model.addClause(c); //

	c = not(Here_now_R_lift)      or Here_will_R_lifting1;
	model.addClause(c);
	c = Here_now_R_lift           or not(Here_will_R_lifting1);
	model.addClause(c); //
	c = not(Here_now_R_lifting1)  or Here_will_R_lifting2;
	model.addClause(c);
	c = Here_now_R_lifting1       or not(Here_will_R_lifting2);
	model.addClause(c); //
	c = not(Here_now_R_lifting2)  or Here_will_R_lifting3;
	model.addClause(c);
	c = Here_now_R_lifting2       or not(Here_will_R_lifting3);
	model.addClause(c); //
	c = not(Here_now_R_lifting3)  or Here_will_R_lifting4;
	model.addClause(c);
	c = Here_now_R_lifting3       or not(Here_will_R_lifting4);
	model.addClause(c); //
	
	c = not(Here_will_C0R_ready)  or Here_now_empty             or Here_now_R_lifting4;
	model.addClause(c); //
	c = not(Here_will_C1R_ready)  or Here_now_empty             or Here_now_R_lifting4;
	model.addClause(c); //
	c = not(Here_will_C2R_ready)  or Here_now_empty             or Here_now_R_lifting4;
	model.addClause(c); //

	c = not(Here_now_R_lifting4)  or Here_will_C0R_ready        or Here_now_C1         or Here_now_C2;
	model.addClause(c);  // maybe make these lazy?!?  
	c = not(Here_now_R_lifting4)  or Here_now_C0                or Here_now_C1         or Here_will_C2R_ready;
	model.addClause(c); //
	c = not(Here_now_R_lifting4)  or Here_will_C0R_ready        or Here_now_C1         or Here_will_C2R_ready;
	model.addClause(c); //
	c = not(Here_now_R_lifting4)  or Here_will_C0R_ready        or Here_will_C1R_ready or Here_now_C2;
	model.addClause(c); //
	c = not(Here_now_R_lifting4)  or Here_now_C0                or Here_will_C1R_ready or Here_now_C2;
	model.addClause(c); //
	c = not(Here_now_R_lifting4)  or Here_now_C0                or Here_will_C1R_ready or Here_will_C2R_ready;
	model.addClause(c); //
	c = not(Here_now_R_lifting4)  or Here_will_C0R_ready        or Here_will_C1R_ready or Here_will_C2R_ready;
	model.addClause(c); //

	model.dump(out);
	
    }
    { // Dropping process
    	CNF::Clause c;
        const CNF::Var Here_now_C0R_ready        = var(v,       t,    NdStat::C0R_ready);
	const CNF::Var Here_now_C1R_ready        = var(v,       t,    NdStat::C1R_ready);
	const CNF::Var Here_now_C2R_ready        = var(v,       t,    NdStat::C2R_ready);
	const CNF::Var Here_now_R_drop           = var(v,       t,    R_Vertical::drop);

	const CNF::Var Here_will_R_drop          = var(v,       t+1,  R_Vertical::drop);
	const CNF::Var Here_will_R_ready         = var(v,       t+1,  NdStat::R_ready);

	const CNF::Var Here_will_empty           = var(v,       t+1,  On_Node::empty);
	const CNF::Var Here_will_C0              = var(v,       t+1,  On_Node::Car0);
	const CNF::Var Here_will_C1              = var(v,       t+1,  On_Node::Car1);
	const CNF::Var Here_will_C2              = var(v,       t+1,  On_Node::Car2);



	c = not(Here_now_R_drop)     or Here_will_R_ready;
	model.addClause(c); //
	c = not(Here_now_C0R_ready)  or Here_now_C1R_ready       or Here_now_C2R_ready      or Here_will_empty                     or Here_will_R_drop;
	model.addClause(c);
	c = Here_now_C0R_ready       or not(Here_now_C1R_ready)  or Here_now_C2R_ready      or Here_will_empty                     or Here_will_R_drop;
	model.addClause(c);
	c = Here_now_C0R_ready       or Here_now_C1R_ready       or not(Here_now_C2R_ready) or Here_will_empty                     or Here_will_R_drop;
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C1R_ready);
	model.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C2R_ready);
	model.addClause(c);
        c = not(Here_now_C1R_ready)  or not(Here_now_C2R_ready);
	model.addClause(c); //
	

	c = not(Here_now_R_drop)     or Here_will_C0              or Here_will_C1             or Here_will_C2;
	model.addClause(c); // maybe make these lazy?!?
	c = not(Here_now_R_drop)     or Here_now_C0R_ready        or Here_will_C1             or Here_will_C2;
	model.addClause(c); //
	c = not(Here_now_R_drop)     or Here_will_C0              or Here_will_C1             or Here_now_C2R_ready;
	model.addClause(c); //
	c = not(Here_now_R_drop)     or Here_now_C0R_ready        or Here_will_C1             or Here_now_C2R_ready;
	model.addClause(c); //
	c = not(Here_now_R_drop)     or Here_now_C0R_ready        or Here_now_C1R_ready       or Here_will_C2;
	model.addClause(c); //
	c = not(Here_now_R_drop)     or Here_will_C0              or Here_now_C1R_ready       or Here_will_C2;
	model.addClause(c); //
	c = not(Here_now_R_drop)     or Here_will_C0              or Here_now_C1R_ready       or Here_now_C2R_ready;
	model.addClause(c); //
	c = not(Here_now_R_drop)     or Here_now_C0R_ready        or Here_now_C1R_ready       or Here_now_C2R_ready;
	model.addClause(c); //

	model.dump(out);	
    }
    {// Lift/drop and  PARKED cars
    	CNF::Clause c;
        const CNF::Var Here_now_R_lifting4  = var(v,       t,      R_Vertical::l4);
	const CNF::Var Here_now_R_drop      = var(v,       t,      R_Vertical::drop);
	const CNF::Var Here_now_empty       = var(v,       t,      On_Node::empty);
	const CNF::Var Here_now_C0          = var(v,       t,      On_Node::Car0);
	const CNF::Var Here_now_C1          = var(v,       t,      On_Node::Car1);
	const CNF::Var Here_now_C2          = var(v,       t,      On_Node::Car2);

	const CNF::Var Here_will_R_lifting4 = var(v,       t+1,    R_Vertical::l4);
	const CNF::Var Here_will_R_drop     = var(v,       t+1,    R_Vertical::drop);
	const CNF::Var Here_will_empty      = var(v,       t+1,    On_Node::empty);
	const CNF::Var Here_will_C0         = var(v,       t+1,    On_Node::Car0);
	const CNF::Var Here_will_C1         = var(v,       t+1,    On_Node::Car1);
	const CNF::Var Here_will_C2         = var(v,       t+1,    On_Node::Car2);


	// neccessarly
	c = not(Here_now_R_lifting4) or Here_now_empty            or Here_will_R_drop         or Here_will_empty;
	model.addClause(c);
	c = Here_now_R_lifting4      or not(Here_now_empty)       or Here_will_R_drop         or Here_will_empty;
	model.addClause(c);
	c = not(Here_now_R_lifting4) or not(Here_now_empty);
	model.addClause(c);
	// sufficient
	c = not(Here_will_R_drop)    or Here_will_empty           or Here_now_R_lifting4      or Here_now_empty;
	model.addClause(c);
	c = Here_will_R_drop         or not(Here_will_empty)      or Here_now_R_lifting4      or Here_now_empty;
	model.addClause(c);
	c = not(Here_will_R_drop)    or not(Here_will_empty);
	model.addClause(c); //


	
	c = not(Here_now_R_drop)     or Here_will_R_drop          or not(Here_will_empty);
	model.addClause(c);
	c = Here_now_R_drop          or not(Here_will_R_drop)     or not(Here_will_empty);
	model.addClause(c);
	c = not(Here_now_R_drop)     or not(Here_will_R_drop);
	model.addClause(c); //
	c = not(Here_will_R_lifting4)or Here_now_R_lifting4       or not(Here_now_empty);
	model.addClause(c);
	c = Here_will_R_lifting4     or not(Here_now_R_lifting4)  or not(Here_now_empty);
	model.addClause(c);
	c = not(Here_will_R_lifting4)or not(Here_now_R_lifting4);
	model.addClause(c); //


	
	c = not(Here_will_R_drop)    or Here_now_empty;
	model.addClause(c); //
	c = not( Here_now_R_lifting4)or Here_will_empty;
	model.addClause(c); //


	
	c = not(Here_will_C0)        or Here_will_C1              or Here_now_C0              or Here_now_C1                       or Here_will_R_drop;
	model.addClause(c);
	c = Here_will_C0             or not(Here_will_C1)         or Here_now_C0              or Here_now_C1                       or Here_will_R_drop;
	model.addClause(c);
	c = not(Here_will_C0)        or not(Here_will_C1);
	model.addClause(c); //
	c = not(Here_will_C0)        or Here_will_C2              or Here_now_C0              or Here_now_C2                       or Here_will_R_drop;
	model.addClause(c);
	c = Here_will_C0             or not(Here_will_C2)         or Here_now_C0              or Here_now_C2                       or Here_will_R_drop;
	model.addClause(c);
	c = not(Here_will_C0)        or not(Here_will_C2);
	model.addClause(c); //
	c = not(Here_will_C1)        or Here_will_C2              or Here_now_C1              or Here_now_C2                       or Here_will_R_drop;
	model.addClause(c);
	c = Here_will_C1             or not(Here_will_C2)         or Here_now_C1              or Here_now_C2                       or Here_will_R_drop;
	model.addClause(c);
	c = not(Here_will_C1)        or not(Here_will_C2);
	model.addClause(c); //


	
	c = not(Here_now_C0)         or Here_now_C1               or Here_will_C0             or Here_will_C1                      or Here_will_R_lifting4;
	model.addClause(c);
	c = Here_now_C0              or not(Here_now_C1)          or Here_will_C0             or Here_will_C1                      or Here_will_R_lifting4;
	model.addClause(c);
	c = not(Here_now_C0)         or not(Here_now_C1);
	model.addClause(c); //
	c = not(Here_now_C0)         or Here_now_C2               or Here_will_C0             or Here_will_C2                      or Here_will_R_lifting4;
	model.addClause(c);
	c = Here_now_C0              or not(Here_now_C2)          or Here_will_C0             or Here_will_C2                      or Here_will_R_lifting4;
	model.addClause(c);
	c = not(Here_now_C0)         or not(Here_now_C2);
	model.addClause(c); //
	c = not(Here_now_C1)         or Here_now_C2               or Here_will_C1             or Here_will_C2                      or Here_will_R_lifting4;
	model.addClause(c);
	c = Here_now_C1              or not(Here_now_C2)          or Here_will_C1             or Here_will_C2                      or Here_will_R_lifting4;
	model.addClause(c);
	c = not(Here_now_C1)         or not(Here_now_C2);
	model.addClause(c); //


	
	c = Here_now_empty           or not(Here_will_empty)      or (Here_now_R_lifting4);
	model.addClause(c); //

	model.dump(out);
    }

} // time_link_constraints()

// EOF grid_gurobi.cc
