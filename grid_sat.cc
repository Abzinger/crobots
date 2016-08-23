// grid_gurobi.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#include "grid_gurobi.hh"
#include "gurobi_c++.h"
#include <stdexcept>
#include <cstdio>
#include <cmath>

// *****************************************************************************************************************************
// *    Grid_Gurobi  member functions
// *****************************************************************************************************************************

GridSpace::Grid_Gurobi::Grid_Gurobi(const Grid & _G, const unsigned _t_max):
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
} // Grid_Gurobi---constructor

GridSpace::Grid_Gurobi::~Grid_Gurobi()
{
    // delete GRBVar arrays
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
} //    ~Grid_Gurobi

//********************************************************************************************************************************************************************************************************
void GridSpace::Grid_Gurobi::set_initial_state(const Stat_Vector_t *p_stat0)
{
    if (p_initial_state) throw std::runtime_error("Grid_Gurobi::set_initial_state(): Attempt to set initial state a 2nd time.");
    p_initial_state = p_stat0;

    for (short y=0; y<G.NS_sz(); ++y) {
        for (short x=0; x<G.EW_sz(); ++x) {
            XY xy {x,y};
            if ( G.exists(xy) )  {
                const Full_Stat s = (*p_stat0)[xy];

                for (On_Node    i=begin_On_Node();    i!=end_On_Node();    ++i) {
                    const double RHS = (s.on_node==i ? 1 : 0 );
                    model.addConstr( RHS == var(xy,0,i) );
                }
                // // punish not leaving the state through large cost
                // for (unsigned t=1; t<t_max; ++t) {
                //     GRBLinExpr _x = var(xy,t,s.on_node);
                //     if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_initial_state(): There's something wrong with this On_Node variable (GRBLinExpr has !=1 terms).");
                //     // GRBVar x = _x.getVar(0);
                //     // x.set( GRB_DoubleAttr_Obj, x.get(GRB_DoubleAttr_Obj) + 100. );
                // }
                for (NdStat     i=begin_NdStat();     i!=end_NdStat();     ++i) {
                    const double RHS = (s.ndstat==i ? 1 : 0 );
                    model.addConstr( RHS == var(xy,0,i) );
                }
                for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
                    const double RHS = (s.r_vert==i ? 1 : 0 );
                    model.addConstr( RHS == var(xy,0,i) );
                }
                for (R_Move       i=begin_R_Move();       i!=end_R_Move();       ++i) {
                    const double RHS = (s.r_mv==i ? 1 : 0 );
                    model.addConstr( RHS == var(xy,0,i) );
                }
            } // if exists
        } // for x
    } // for y
} //^ set_initial_state()

void GridSpace::Grid_Gurobi::set_terminal_state(const Stat_Vector_t * p_state)
{
    if (p_terminal_state) throw std::runtime_error("Grid_Gurobi::set_initial_state(): Attempt to set terminal state a 2nd time.");
    p_terminal_state = p_state;

    for (short y=0; y<G.NS; ++y) {
        for (short x=0; x<G.EW; ++x) {
            XY xy {x,y};
            if ( G.exists(xy) )  {
                const Full_Stat s = (*p_state)[xy];

                if ( s.on_node!=On_Node::empty && ( !my_opts.ignore_C0 || s.on_node!=On_Node::Car0 ) ) {
                    for (    On_Node    i=begin_On_Node();    i!=end_On_Node();    ++i) {
                        if (my_opts.hardwire) {
                            const double RHS = (s.on_node==i ? 1 : 0 );
                            model.addConstr(     RHS == var(xy,t_max,i) );
                        } else {
                            for (unsigned t = 1 + (1-my_opts.punish_mismatch)*(t_max-1); t<=t_max; ++t) {
                                GRBLinExpr _x = var(xy,t,i);
                                if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_terminal_state(): There's something wrong with this On_Node variable (GRBLinExpr has !=1 terms).");
                                GRBVar x = _x.getVar(0);
                                x.set( GRB_DoubleAttr_Obj, (s.on_node==i ? 0 : (t==t_max ? mismatch_cost__t_max : mismatch_cost__t) ) );
                            } //^ for t
                        } //^ if/else
                    } //^ for stat
                } //^ if whether to ignore C0
                if (!my_opts.ignore_robots) {
                    for (NdStat     i=begin_NdStat();     i!=end_NdStat();     ++i) {
                        if (my_opts.hardwire) {
                            const double RHS = (s.ndstat==i ? 1 : 0 );
                            model.addConstr( RHS == var(xy,t_max,i) );
                        } else {
                            for (unsigned t = 1 + (1-my_opts.punish_mismatch)*(t_max-1); t<=t_max; ++t) {
                                GRBLinExpr _x = var(xy,t,i);
                                if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_terminal_state(): There's something wrong with this NdStat variable (GRBLinExpr has !=1 terms).");
                                GRBVar x = _x.getVar(0);
                                x.set( GRB_DoubleAttr_Obj, (s.ndstat==i ? 0 : (t==t_max ? mismatch_cost__t_max : mismatch_cost__t) ) );
                            } //^ for
                        }
                    }
                    for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
                        if (my_opts.hardwire) {
                            const double RHS = (s.r_vert==i ? 1 : 0 );
                            model.addConstr( RHS == var(xy,t_max,i) );
                        } else {
                            for (unsigned t = 1 + (1-my_opts.punish_mismatch)*(t_max-1); t<=t_max; ++t) {
                                GRBLinExpr _x = var(xy,t,i);
                                if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_terminal_state(): There's something wrong with this R_Vertical variable (GRBLinExpr has !=1 terms).");
                                GRBVar x = _x.getVar(0);
                                x.set( GRB_DoubleAttr_Obj, (s.r_vert==i ? 0 : (t==t_max ? mismatch_cost__t_max : mismatch_cost__t) ) );
                            } //^ for
                        }
                    }
                    for (R_Move       i=begin_R_Move();       i!=end_R_Move();       ++i) {
                        const Direction d = get_direction(i);
                        if ( G.move(xy,d)!=nowhere ) {
                            if (my_opts.hardwire) {
                                const double RHS = (s.r_mv==i ? 1 : 0 );
                                model.addConstr( RHS == var(xy,t_max,i) );
                            } else {
                                for (unsigned t = 1 + (1-my_opts.punish_mismatch)*(t_max-1); t<=t_max; ++t) {
                                    GRBLinExpr _x = var(xy,t,i);
                                    if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_terminal_state(): There's something wrong with this R_Move variable (GRBLinExpr has !=1 terms).");
                                    GRBVar x = _x.getVar(0);
                                    x.set( GRB_DoubleAttr_Obj, (s.r_mv==i ? 0 : (t==t_max ? mismatch_cost__t_max : mismatch_cost__t) ) );
                                } //^ for
                            } //^ if/else
                        } //^ if dir exists
                    }
                } // if (do robots)
            } // if exists
        } // for x
    } // for y
} // set_terminal_state()

//********************************************************************************************************************************************************************************************************

void GridSpace::Grid_Gurobi::optimize()
{
    Grid_Gurobi_Callback my_callback {*this};
    model.setCallback(&my_callback);

    model.getEnv().set(GRB_IntParam_Threads, 1);

    model.optimize();
} //^ optimize()

//********************************************************************************************************************************************************************************************************

std::vector< GridSpace::Stat_Vector_t > GridSpace::Grid_Gurobi::get_solution()  const
{
    if (model.get(GRB_IntAttr_SolCount) > 0) {
        std::vector< Stat_Vector_t > fullsol (t_max+1, G);
        for (unsigned t=0; t<=t_max; ++t) {
            XY v {0,0};
            for (v.y=0; v.y<G.NS_sz(); ++v.y) {
                for (v.x=0; v.x<G.EW_sz(); ++v.x) {
                    if ( G.exists(v) ) {
                        On_Node on_node = On_Node::SIZE;
                        for (On_Node    i=begin_On_Node();    i!=end_On_Node();    ++i) {
                            GRBLinExpr _x = var(v,t,i);
                            if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::get_solution(): There's something wrong with this On_Node variable (GRBLinExpr has !=1 terms).");
                            GRBVar x = _x.getVar(0);
                            const double val = x.get(GRB_DoubleAttr_X);
                            if (val>.1 && val<.9)    throw std::runtime_error("Grid_Gurobi::get_solution(): This On_Node variable doesn't appear to be integral.");
                            if (val > .5) {
                                if (on_node!=On_Node::SIZE) throw std::runtime_error("Grid_Gurobi::get_solution(): There seem to be >1 On_Node variables with value 1.");
                                on_node=i;
                            }
                        } // for  On_Node

                        NdStat ndstat = NdStat::SIZE;
                        for (NdStat     i=begin_NdStat();     i!=end_NdStat();     ++i) {
                            GRBLinExpr _x = var(v,t,i);
                            if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::get_solution(): There's something wrong with this NdStat variable (GRBLinExpr has !=1 terms).");
                            GRBVar x = _x.getVar(0);
                            const double val = x.get(GRB_DoubleAttr_X);
                            if (val>.1 && val<.9) throw std::runtime_error("Grid_Gurobi::get_solution(): This NdStat variable doesn't appear to be integral.");
                            if (val > .5) {
                                if (ndstat!=NdStat::SIZE) throw std::runtime_error("Grid_Gurobi::get_solution(): There seem to be >1 NdStat variables with value 1.");
                                ndstat=i;
                            }
                        } // for  NdStat

                        R_Vertical r_vert = R_Vertical::SIZE;
                        for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
                            GRBLinExpr _x = var(v,t,i);
                            if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::get_solution(): There's something wrong with this R_Vertical variable (GRBLinExpr has !=1 terms).");
                            GRBVar x = _x.getVar(0);
                            const double val = x.get(GRB_DoubleAttr_X);
                            if (val>.1 && val<.9) throw std::runtime_error("Grid_Gurobi::get_solution(): This R_Vertical variable doesn't appear to be integral.");
                            if (val > .5) {
                                if (r_vert!=R_Vertical::SIZE) throw std::runtime_error("Grid_Gurobi::get_solution(): There seem to be >1 R_Vertical variables with value 1.");
                                r_vert=i;
                            }
                        } // for  R_Vertical

                        R_Move r_mv = R_Move::SIZE;
                        for (R_Move     i=begin_R_Move();     i!=end_R_Move();     ++i) {
                            const Direction d = get_direction(i);
                            double val = 0.;
                            if ( G.move(v,d)!=nowhere ) {
                                GRBLinExpr _x = var(v,t,i);
                                if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::get_solution(): There's something wrong with this R_Move variable (GRBLinExpr has !=1 terms).");
                                GRBVar x = _x.getVar(0);
                                val = x.get(GRB_DoubleAttr_X);
                            }
                            if (val>.1 && val<.9) throw std::runtime_error("Grid_Gurobi::get_solution(): This R_Move variable doesn't appear to be integral.");
                            if (val > .5) {
                                if (r_mv!=R_Move::SIZE) throw std::runtime_error("Grid_Gurobi::get_solution(): There seem to be >1 R_Move variables with value 1.");
                                r_mv=i;
                            }
                        } // for  R_Move

                        fullsol[t][v] = Full_Stat{on_node,ndstat,r_vert,r_mv};
                    } // if exists
                } // for x
            } // for y
        } // for t

        return fullsol;
    } // if  there's a solution
    else return std::vector< Stat_Vector_t >{};

} // get_solution()


//********************************************************************************************************************************************************************************************************
//  A C C E S S    T O   V A R I B L E S
//********************************************************************************************************************************************************************************************************


inline
CNF::Var GridSpace::Grid_Gurobi::var(const XY v, const unsigned t, const On_Node what) const
{
    if ((unsigned)what == (unsigned)On_Node::SIZE) throw std::range_error  ("Grid_Gurobi::var(On_Node): On_Node argument is out of range");
    if ((unsigned)what >  (unsigned)On_Node::SIZE) throw std::runtime_error("Grid_Gurobi::var(On_Node): On_Node argument is broken (BAD BUG)");

    if (v==nowhere) {
        if (what==On_Node::empty) return 1;
        else                      return 0;
    } else {
        if (! G.exists(v) ) throw std::range_error  ("Grid_Gurobi::var(On_Node): node does not exist.");
        return onnode_vars[v][t][(int)what];
    }
} // var()

inline
CNF::Var GridSpace::Grid_Gurobi::var(const XY v, const unsigned t, const NdStat who) const
{
    if ((unsigned)who == (unsigned)NdStat::SIZE) throw std::range_error  ("Grid_Gurobi::var(NdStat): NdStat argument is out of range");
    if ((unsigned)who >  (unsigned)NdStat::SIZE) throw std::runtime_error("Grid_Gurobi::var(NdStat): NdStat argument is broken (BAD BUG)");

    if (v==nowhere) {
        if (who==NdStat::nobodyhome) return 1;
        else                         return 0;
    } else {
        if (! G.exists(v) ) throw std::range_error  ("Grid_Gurobi::var(NdStat): node does not exist.");
        return ndstat_vars[v][t][(int)who];
    }
} // var()

inline
CNF::Var GridSpace::Grid_Gurobi::var(const XY v, const unsigned t, const R_Vertical vert) const
{
    if ((unsigned)vert == (unsigned)R_Vertical::SIZE) throw std::range_error  ("Grid_Gurobi::var(R_Vertical): R_Vertical argument is out of range");
    if ((unsigned)vert >  (unsigned)R_Vertical::SIZE) throw std::runtime_error("Grid_Gurobi::var(R_Vertical): R_Vertical argument is broken (BAD BUG)");

    if (v==nowhere) return 0;
    else {
        if (! G.exists(v) ) throw std::range_error  ("Grid_Gurobi::var(R_Vertical): node does not exist.");
        return rvertical_vars[v][t][(int)vert];
    }
} // var()

inline
CNF::Var GridSpace::Grid_Gurobi::var(const XY v, const unsigned t, const R_Move where) const
{
    if ((unsigned)where == (unsigned)R_Move::SIZE) throw std::range_error  ("Grid_Gurobi::var(R_Move): R_Move argument is out of range");
    if ((unsigned)where >  (unsigned)R_Move::SIZE) throw std::runtime_error("Grid_Gurobi::var(R_Move): R_Move argument is broken (BAD BUG)");

    if (v==nowhere) return 0;
    else {
        if (! G.exists(v) ) throw std::range_error  ("Grid_Gurobi::var(R_Move): node does not exist.");
        const Direction d = get_direction(where);
        return ( G.move(v,d)==nowhere ?    (GRBLinExpr)0.   :   (GRBLinExpr)rmv_vars[v][t][(int)where]  );
    }
} // var()


//********************************************************************************************************************************************************************************************************
//  C R E A T E   T H E   M O D E L
//********************************************************************************************************************************************************************************************************
void GridSpace::Grid_Gurobi::make_model()
{
    make_vars();
    model.update();
    make_constraints();
    model.update();
} // make_model()


//********************************************************************************************************************************************************************************************************
//  V A R I A B L E S
//********************************************************************************************************************************************************************************************************
void GridSpace::Grid_Gurobi::make_vars()
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

void GridSpace::Grid_Gurobi::atom_vars(const XY v, const unsigned t)
{
    auto vartype = GRB_BINARY;
    // auto vartype = GRB_CONTINUOUS;

    // how many vars per node,time ---in total?
    constexpr int total_size = (int)On_Node::SIZE + (int)NdStat::SIZE + (int)R_Vertical::SIZE + (int)R_Move::SIZE;

    // allocate the mem
    CNF::Var * var_array = new GRBVar[total_size];

    // store the vars
    char var_name_buffer[1024];  // for the variable names

    int offset = 0;

    onnode_vars[v][t] = var_array+offset;
    for (On_Node i=begin_On_Node(); i!=end_On_Node(); ++i) {
        std::sprintf(var_name_buffer, "onnd[%.3d:(%.2d,%.2d):%s]",t,v.x,v.y, to_string(i) );
        var_array[offset++] = model.addVar(        0,         1,          0.,    vartype,  var_name_buffer);
        //                   GRBVar addVar(double lb, double ub,  double obj,  char type,  string name    )
    }

    ndstat_vars[v][t] = var_array+offset;
    for (NdStat i=begin_NdStat(); i!=end_NdStat(); ++i) {
        std::sprintf(var_name_buffer, "ndst[%.3d:(%.2d,%.2d):%s]",t,v.x,v.y, to_string(i) );
        double cost = (  i==NdStat::R_ready || i==NdStat::nobodyhome  ?  0.  :  t/(100.*t_max)  );
        var_array[offset++] = model.addVar(        0,         1,        cost,    vartype,  var_name_buffer);
        //                   GRBVar addVar(double lb, double ub,  double obj,  char type,  string name    )
    } // for (ndstat)

    rvertical_vars[v][t] = var_array+offset;
    for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
        std::sprintf(var_name_buffer, "rvrt[%.3d:(%.2d,%.2d):%s]",t,v.x,v.y, to_string(i) );
        var_array[offset++] = model.addVar(0,1,0.,vartype, var_name_buffer);
    }


    rmv_vars[v][t] = var_array+offset;
    for (R_Move i=begin_R_Move(); i!=end_R_Move(); ++i) {
        const Direction d = get_direction(i);
        if ( G.move(v,d)!=nowhere ) {
            std::sprintf(var_name_buffer, "rmv[%.3d:(%.2d,%.2d):%s]",t,v.x,v.y, to_string(i) );
            var_array[offset++] = model.addVar(        0,         1,           0,    vartype,  var_name_buffer);
            //                   GRBVar addVar(double lb, double ub,  double obj,  char type,  string name    )
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

void GridSpace::Grid_Gurobi::make_constraints()
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

#define IMPLIES       <=
#define REQUIRES      <=
#define FOLLOWS_FROM  >=
#define EQUIVALENT_TO ==
#define OR            +
#define NOT           1-

//********************************************************************************************************************************************************************************************************
//       A T O M   C O N S T R A I N T S
//********************************************************************************************************************************************************************************************************


void GridSpace::Grid_Gurobi::atom_constraints(const XY v, const unsigned t)
{
      // B A S I C
      {
	Model m;
	CNF::Var Here_now_empty       = var(v,       t,    On_Node::empty);
	CNF::Var Here_now_car0        = var(v,       t,    On_Node::Car0);
	CNF::Var Here_now_car1        = var(v,       t,    On_Node::Car1);
	CNF::Var Here_now_car2        = var(v,       t,    On_Node::Car2);
	
	Clause c;
	c = Here_now_empty      or Here_now_car0       or Here_now_car1  or Here_now_car2;
	m.addClause(c);
	c = not(Here_now_empty) or not(Here_now_car0);
	m.addClause(c);
	c = not(Here_now_empty) or not(Here_now_car1);
	m.addClause(c);
	c = not(Here_now_empty) or not(Here_now_car2);
	m.addClause(c);
	c = not(ere_now_car0)   or not(Here_now_car1);
	m.addClause(c);
	c = not(Here_now_car0)  or not(Here_now_car2);
	m.addClause(c);
	c = not(Here_now_car1)  or not(Here_now_car2);
	m.addClause(c);
	
	CNF::Var Here_now_nobodyhome       = var(v,       t,    NdStat::nobodyhome  );
	CNF::Var Here_now_R_ready          = var(v,       t,    NdStat::R_ready     );
	CNF::Var Here_now_C0R_ready        = var(v,       t,    NdStat::C0R_ready   );
	CNF::Var Here_now_C1R_ready        = var(v,       t,    NdStat::C1R_ready   );
	CNF::Var Here_now_C2R_ready        = var(v,       t,    NdStat::C2R_ready   );
	CNF::Var Here_now_R_moving         = var(v,       t,    NdStat::R_moving    );
	CNF::Var Here_now_C0R_moving       = var(v,       t,    NdStat::C0R_moving  );
	CNF::Var Here_now_C1R_moving       = var(v,       t,    NdStat::C1R_moving  );
	CNF::Var Here_now_C2R_moving       = var(v,       t,    NdStat::C2R_moving  );
	CNF::Var Here_now_R_vertical       = var(v,       t,    NdStat::R_vertical  );
	
	c = Here_now_nobodyhome
	  or Here_now_R_ready     or Here_now_C0R_ready  or Here_now_C1R_ready  or Here_now_C2R_ready
	  or Here_now_R_moving    or Here_now_C0R_moving or Here_now_C1R_moving or Here_now_C2R_moving
	  or Here_now_R_vertical;
	m.addClause(c);      
	c = not(Here_now_nobodyhome) or not(Here_now_R_ready);
	m.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C0R_ready);
	m.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C1R_ready);
	m.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C2R_ready);
	m.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_R_moving);
	m.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_nobodyhome) or not(Here_now_R_vertical);
	m.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C0R_ready);
	m.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C1R_ready);
	m.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C2R_ready);
	m.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_R_moving);
	m.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_R_ready)    or not(Here_now_R_vertical);
	m.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C1R_ready);
	m.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C2R_ready);
	m.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_ready)  or not(Here_now_R_vertical);
	m.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_C2R_ready);
	m.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_ready)  or not(Here_now_R_vertical);
	m.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_R_moving);
	m.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C2R_ready)  or not(Here_now_R_vertical);
	m.addClause(c);
	c = not(Here_now_R_moving)   or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_R_moving)   or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_R_moving)   or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_R_moving)   or not(Here_now_R_vertical);
	m.addClause(c);
	c = not(Here_now_C0R_moving) or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_moving) or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_moving) or not(Here_now_R_vertical);
	m.addClause(c);
	c = not(Here_now_C1R_moving) or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_moving) or not(Here_now_R_vertical);
	m.addClause(c);
	c = not(Here_now_C2R_moving) or not(Here_now_R_vertical);
	m.addClause(c);


	// at most one of crobot or car:
	
	c = not(Here_now_C0R_ready)     or Here_now_C1R_ready       or Here_now_C2R_ready      or Here_now_C0R_moving              or Here_now_C1R_moving      or Here_now_C2R_moving      or Here_now_empty;
	m.addClause(c);
	c = Here_now_C0R_ready          or not(Here_now_C1R_ready)  or Here_now_C2R_ready      or Here_now_C0R_moving              or Here_now_C1R_moving      or Here_now_C2R_moving      or Here_now_empty;
	m.addClause(c);
	c = Here_now_C0R_ready          or Here_now_C1R_ready       or not(Here_now_C2R_ready) or Here_now_C0R_moving              or Here_now_C1R_moving      or Here_now_C2R_moving      or Here_now_empty;
	m.addClause(c);
	c = Here_now_C0R_ready          or Here_now_C1R_ready       or Here_now_C2R_ready      or not(Here_now_C0R_moving)         or Here_now_C1R_moving      or Here_now_C2R_moving      or Here_now_empty;
	m.addClause(c);
	c = Here_now_C0R_ready          or Here_now_C1R_ready       or Here_now_C2R_ready      or Here_now_C0R_moving              or not(Here_now_C1R_moving) or Here_now_C2R_moving      or Here_now_empty;
	m.addClause(c);
	c = Here_now_C0R_ready          or Here_now_C1R_ready       or Here_now_C2R_ready      or Here_now_C0R_moving              or Here_now_C1R_moving      or not(Here_now_C2R_moving) or Here_now_empty;
	m.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C1R_ready);
	m.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C2R_ready);
	m.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_ready)     or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_ready)     or not(Here_now_C2R_ready);
	m.addClause(c);
	c = not(Here_now_C1R_ready)     or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_ready)     or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_ready)     or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C2R_ready)     or not(Here_now_C0R_moving);
	m.addClause(c);
	c = not(Here_now_C2R_ready)     or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_C2R_ready)     or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_moving)    or not(Here_now_C1R_moving);
	m.addClause(c);
	c = not(Here_now_C0R_moving)    or not(Here_now_C2R_moving);
	m.addClause(c);
	c = not(Here_now_C1R_moving)    or not(Here_now_C2R_moving);
	m.addClause(c);


	CNF::Var Here_now_R_lift           = var(v,       t,    R_Vertical::lift);
	CNF::Var Here_now_R_lifting1       = var(v,       t,    R_Vertical::l1);
	CNF::Var Here_now_R_lifting2       = var(v,       t,    R_Vertical::l2);
	CNF::Var Here_now_R_lifting3       = var(v,       t,    R_Vertical::l3);
	CNF::Var Here_now_R_lifting4       = var(v,       t,    R_Vertical::l4);
	CNF::Var Here_now_R_drop           = var(v,       t,    R_Vertical::l4);
	// neccessarly 
	c = not(Here_now_R_vertical)    or Here_now_R_lift          or Here_now_R_lifting1     or Here_now_R_lifting2              or Here_now_R_lifting3      or Here_now_R_lifting4      or Here_now_R_drop;
	m.addClause(c);
	// sufficient 
	// 6 choose 1
	c = not(Here_now_R_lift)        or Here_now_R_lifting1      or Here_now_R_lifting2     or Here_now_R_lifting3              or Here_now_R_lifting4      or Here_now_R_drop          or Here_now_R_vertical;
	m.addClause(c);
	c = Here_now_R_lift             or not(Here_now_R_lifting1) or Here_now_R_lifting2     or Here_now_R_lifting3              or Here_now_R_lifting4      or Here_now_R_drop          or Here_now_R_vertical;
	m.addClause(c);
	c = Here_now_R_lift             or Here_now_R_lifting1      or not(Here_now_R_lifting2)or Here_now_R_lifting3              or Here_now_R_lifting4      or Here_now_R_drop          or Here_now_R_vertical;
	m.addClause(c);
	c = Here_now_R_lift             or Here_now_R_lifting1      or Here_now_R_lifting2     or not(Here_now_R_lifting3)         or Here_now_R_lifting4      or Here_now_R_drop          or Here_now_R_vertical;
	m.addClause(c);
	c = Here_now_R_lift             or Here_now_R_lifting1      or Here_now_R_lifting2     or Here_now_R_lifting3              or not(Here_now_R_lifting4) or Here_now_R_drop          or Here_now_R_vertical;
	m.addClause(c);
	c = Here_now_R_lift             or Here_now_R_lifting1      or Here_now_R_lifting2     or Here_now_R_lifting3              or Here_now_R_lifting4      or not(Here_now_R_drop)     or Here_now_R_vertical;
	m.addClause(c);
	// 6 choose 2
	c = not(Here_now_R_lift)        or not(Here_now_R_lifting1);
	m.addClause(c);
	c = not(Here_now_R_lift)        or not(Here_now_R_lifting2);
	m.addClause(c);
	c = not(Here_now_R_lift)        or not(Here_now_R_lifting3);
	m.addClause(c);
	c = not(Here_now_R_lift)        or not(Here_now_R_lifting4);
	m.addClause(c);
	c = not(Here_now_R_lift)        or not(Here_now_R_drop);
	m.addClause(c);
	c = not(Here_now_R_lifting1)    or not(Here_now_R_lifting2);
	m.addClause(c);
	c = not(Here_now_R_lifting1)    or not(Here_now_R_lifting3);
	m.addClause(c);
	c = not(Here_now_R_lifting1)    or not(Here_now_R_lifting4);
	m.addClause(c);
	c = not(Here_now_R_lifting1)    or not(Here_now_R_drop);
	m.addClause(c);
	c = not(Here_now_R_lifting2)    or not(Here_now_R_lifting3);
	m.addClause(c);
	c = not(Here_now_R_lifting2)    or not(Here_now_R_lifting4);
	m.addClause(c);
	c = not(Here_now_R_lifting2)    or not(Here_now_R_drop);
	m.addClause(c);
	c = not(Here_now_R_lifting3)    or not(Here_now_R_lifting4);
	m.addClause(c);
	c = not(Here_now_R_lifting3)    or not(Here_now_R_drop);
	m.addClause(c);
	c = not(Here_now_R_lifting4)    or not(Here_now_R_drop);


	CNF::Var  Here_now_R_accE = var(v,    t,     R_Move::accE);
	CNF::Var  Here_now_R_mvE0 = var(v,    t,     R_Move::mvE0);
	CNF::Var  Here_now_R_accW = var(v,    t,     R_Move::accW);
	CNF::Var  Here_now_R_mvW0 = var(v,    t,     R_Move::mvW0);
	CNF::Var  Here_now_R_accN = var(v,    t,     R_Move::accN);
	CNF::Var  Here_now_R_mvN1 = var(v,    t,     R_Move::mvN1);
	CNF::Var  Here_now_R_mvN0 = var(v,    t,     R_Move::mvN0);
	CNF::Var  Here_now_R_accS = var(v,    t,     R_Move::accS);
	CNF::Var  Here_now_R_mvS1 = var(v,    t,     R_Move::mvS1);
	CNF::Var  Here_now_R_mvS0 = var(v,    t,     R_Move::mvS0);
	// necessarly
	c = not(Here_now_R_moving)                                                                                                 or Here_now_R_accE      or                    Here_now_R_mvE0                                                          or Here_now_R_accW      or                    Here_now_R_mvW0                                                          or Here_now_R_accN      or Here_now_R_mvN1 or Here_now_R_mvN0                                                          or Here_now_R_accS      or Here_now_R_mvS1 or Here_now_R_mvS0;
	m.addClause(c);
	// sufficient
	// 10 choose 1
	c = Here_now_R_moving                                                                                                      or not(Here_now_R_accE) or                    Here_now_R_mvE0                                                          or Here_now_R_accW      or                    Here_now_R_mvW0                                                          or Here_now_R_accN      or Here_now_R_mvN1 or Here_now_R_mvN0                                                          or Here_now_R_accS      or Here_now_R_mvS1 or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                    not(Here_now_R_mvE0)                                                     or Here_now_R_accW      or                    Here_now_R_mvW0                                                          or Here_now_R_accN      or Here_now_R_mvN1 or Here_now_R_mvN0                                                          or Here_now_R_accS      or Here_now_R_mvS1 or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                    Here_now_R_mvE0                                                          or not(Here_now_R_accW) or                    Here_now_R_mvW0                                                          or Here_now_R_accN      or Here_now_R_mvN1 or Here_now_R_mvN0                                                          or Here_now_R_accS      or Here_now_R_mvS1 or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                    Here_now_R_mvE0                                                          or Here_now_R_accW      or                    not(Here_now_R_mvW0)                                                     or Here_now_R_accN      or Here_now_R_mvN1 or Here_now_R_mvN0                                                          or Here_now_R_accS      or Here_now_R_mvS1 or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                    Here_now_R_mvE0                                                          or Here_now_R_accW      or                    Here_now_R_mvW0                                                          or not(Here_now_R_accN) or Here_now_R_mvN1 or Here_now_R_mvN0                                                          or Here_now_R_accS      or Here_now_R_mvS1 or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                         Here_now_R_mvE0                                                     or Here_now_R_accW      or                         Here_now_R_mvW0                                                     or Here_now_R_accN      or not(Here_now_R_mvN1) or Here_now_R_mvN0                                                     or Here_now_R_accS      or Here_now_R_mvS1      or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                    Here_now_R_mvE0                                                          or Here_now_R_accW      or                    Here_now_R_mvW0                                                          or Here_now_R_accN      or Here_now_R_mvN1 or not(Here_now_R_mvN0)                                                     or Here_now_R_accS      or Here_now_R_mvS1 or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE      or                    Here_now_R_mvE0                                                          or Here_now_R_accW      or                    Here_now_R_mvW0                                                          or Here_now_R_accN      or Here_now_R_mvN1 or Here_now_R_mvN0                                                          or not(Here_now_R_accS) or Here_now_R_mvS1 or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or Here_now_R_accE or                         Here_now_R_mvE0                                                          or Here_now_R_accW or                         Here_now_R_mvW0                                                          or Here_now_R_accN or Here_now_R_mvN1      or Here_now_R_mvN0                                                          or Here_now_R_accS or not(Here_now_R_mvS1) or Here_now_R_mvS0;
	m.addClause(c);
	c = Here_now_R_moving                                                                                                      or  Here_now_R_accE or                    Here_now_R_mvE0                                                              or  Here_now_R_accW or                    Here_now_R_mvW0                                                              or  Here_now_R_accN or Here_now_R_mvN1 or Here_now_R_mvN0                                                              or  Here_now_R_accS or Here_now_R_mvS1 or not(Here_now_R_mvS0);
	m.addClause(c);
	//10 choose 2
	c = not(Here_now_R_accE) or not(Here_now_R_mvE0);
	m.addClause(c);
	c = not(Here_now_R_accE) or not(Here_now_R_accW);
	m.addClause(c);
	c = not(Here_now_R_accE) or not(Here_now_R_mvW0);
	m.addClause(c);
	c = not(Here_now_R_accE) or not(Here_now_R_accN);
	m.addClause(c);
	c = not(Here_now_R_accE) or not(Here_now_R_mvN1);
	m.addClause(c);
	c = not(Here_now_R_accE) or not(Here_now_R_mvN0);
	m.addClause(c);
	c = not(Here_now_R_accE) or not(Here_now_R_accS);
	m.addClause(c);
	c = not(Here_now_R_accE) or not(Here_now_R_mvS1);
	m.addClause(c);
	c = not(Here_now_R_accE) or not(Here_now_R_mvS0);
	m.addClause(c);
	c = not(Here_now_R_mvE0) or not(Here_now_R_accW);
	m.addClause(c);
	c = not(Here_now_R_mvE0) or not(Here_now_R_mvW0);
	m.addClause(c);
	c = not(Here_now_R_mvE0) or not(Here_now_R_accN);
	m.addClause(c);
	c = not(Here_now_R_mvE0) or not(Here_now_R_mvN1);
	m.addClause(c);
	c = not(Here_now_R_mvE0) or not(Here_now_R_mvN0);
	m.addClause(c);
	c = not(Here_now_R_mvE0) or not(Here_now_R_accS);
	m.addClause(c);
	c = not(Here_now_R_mvE0) or not(Here_now_R_mvS1);
	m.addClause(c);
	c = not(Here_now_R_mvE0) or not(Here_now_R_mvS0);
	m.addClause(c);
	c = not(Here_now_R_accW) or not(Here_now_R_mvW0);
	m.addClause(c);
	c = not(Here_now_R_accW) or not(Here_now_R_accN);
	m.addClause(c);
	c = not(Here_now_R_accW) or not(Here_now_R_mvN1);
	m.addClause(c);
	c = not(Here_now_R_accW) or not(Here_now_R_mvN0);
	m.addClause(c);
	c = not(Here_now_R_accW) or not(Here_now_R_accS);
	m.addClause(c);
	c = not(Here_now_R_accW) or not(Here_now_R_mvS1);
	m.addClause(c);
	c = not(Here_now_R_accW) or not(Here_now_R_mvS0);
	m.addClause(c);
	c = not(Here_now_R_mvW0) or not(Here_now_R_accN);
	m.addClause(c);
	c = not(Here_now_R_mvW0) or not(Here_now_R_mvN1);
	m.addClause(c);
	c = not(Here_now_R_mvW0) or not(Here_now_R_mvN0);
	m.addClause(c);
	c = not(Here_now_R_mvW0) or not(Here_now_R_accS);
	m.addClause(c);
	c = not(Here_now_R_mvW0) or not(Here_now_R_mvS1);
	m.addClause(c);
	c = not(Here_now_R_mvW0) or not(Here_now_R_mvS0);
	m.addClause(c);
	c = not(Here_now_R_accN) or not(Here_now_R_mvN1);
	m.addClause(c);
	c = not(Here_now_R_accN) or not(Here_now_R_mvN0);
	m.addClause(c);
	c = not(Here_now_R_accN) or not(Here_now_R_accS);
	m.addClause(c);
	c = not(Here_now_R_accN) or not(Here_now_R_mvS1);
	m.addClause(c);
	c = not(Here_now_R_accN) or not(Here_now_R_mvS0);
	m.addClause(c);
	c = not(Here_now_R_mvN1) or not(Here_now_R_mvN0);
	m.addClause(c);
	c = not(Here_now_R_mvN1) or not(Here_now_R_accS);
	m.addClause(c);
	c = not(Here_now_R_mvN1) or not(Here_now_R_mvS1);
	m.addClause(c);
	c = not(Here_now_R_mvN1) or not(Here_now_R_mvS0);
	m.addClause(c);
	c = not(Here_now_R_mvN0) or not(Here_now_R_accS);
	m.addClause(c);
	c = not(Here_now_R_mvN0) or not(Here_now_R_mvS1);
	m.addClause(c);
	c = not(Here_now_R_mvN0) or not(Here_now_R_mvS0);
	m.addClause(c);
	c = not(Here_now_R_accS) or not(Here_now_R_mvS1);
	m.addClause(c);
	c = not(Here_now_R_accS) or not(Here_now_R_mvS0);
	m.addClause(c);
	c = not(Here_now_R_mvS1) or not(Here_now_R_mvS0);
	m.addClause(c);

	// Car 0
	
	CNF::Var Here_now_C0R_accE = var(v,    t,     R_Move::w0_accE);
	CNF::Var Here_now_C0R_mvE1 = var(v,    t,     R_Move::w0_mvE1);
	CNF::Var Here_now_C0R_mvE0 = var(v,    t,     R_Move::w0_mvE0);
	CNF::Var Here_now_C0R_accW = var(v,    t,     R_Move::w0_accW);
	CNF::Var Here_now_C0R_mvW1 = var(v,    t,     R_Move::w0_mvW1);
	CNF::Var Here_now_C0R_mvW0 = var(v,    t,     R_Move::w0_mvW0);
	CNF::Var Here_now_C0R_accN = var(v,    t,     R_Move::w0_accN);
	CNF::Var Here_now_C0R_mvN1 = var(v,    t,     R_Move::w0_mvN1);
	CNF::Var Here_now_C0R_mvN2 = var(v,    t,     R_Move::w0_mvN2);
	CNF::Var Here_now_C0R_mvN3 = var(v,    t,     R_Move::w0_mvN3);
	CNF::Var Here_now_C0R_mvN0 = var(v,    t,     R_Move::w0_mvN0);
	CNF::Var Here_now_C0R_accS = var(v,    t,     R_Move::w0_accS);
	CNF::Var Here_now_C0R_mvS1 = var(v,    t,     R_Move::w0_mvS1);
	CNF::Var Here_now_C0R_mvS2 = var(v,    t,     R_Move::w0_mvS2);
	CNF::Var Here_now_C0R_mvS3 = var(v,    t,     R_Move::w0_mvS3);
	CNF::Var Here_now_C0R_mvS0 = var(v,    t,     R_Move::w0_mvS0);

	//necceraly
	c = not(Here_now_C0R_moving)
	                 or  Here_now_C0R_accE or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	// sufficient
	// 16 choose 1
	c = Here_now_C0R_moving or
	                 not(Here_now_C0R_accE) or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW      or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN      or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS      or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
		         Here_now_C0R_accE or not(Here_now_C0R_mvE1) or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW or Here_now_C0R_mvW1      or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN or Here_now_C0R_mvN1      or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1      or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1 or not(Here_now_C0R_mvE0) or
                         Here_now_C0R_accW or Here_now_C0R_mvW1 or Here_now_C0R_mvW0      or
                         Here_now_C0R_accN or Here_now_C0R_mvN1 or Here_now_C0R_mvN2      or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or Here_now_C0R_mvS2      or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE      or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
	                 not(Here_now_C0R_accW) or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN      or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS      or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1      or Here_now_C0R_mvE0 or
	                 Here_now_C0R_accW or not(Here_now_C0R_mvW1) or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN or Here_now_C0R_mvN1      or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1      or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1 or Here_now_C0R_mvE0      or
	                 Here_now_C0R_accW or Here_now_C0R_mvW1 or not(Here_now_C0R_mvW0) or
                         Here_now_C0R_accN or Here_now_C0R_mvN1 or Here_now_C0R_mvN2      or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or Here_now_C0R_mvS2      or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE      or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW      or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
	                 not(Here_now_C0R_accN) or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS      or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1      or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW or Here_now_C0R_mvW1      or Here_now_C0R_mvW0 or
	                 Here_now_C0R_accN or not(Here_now_C0R_mvN1) or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1      or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1 or Here_now_C0R_mvE0      or
                         Here_now_C0R_accW or Here_now_C0R_mvW1 or Here_now_C0R_mvW0      or
	                 Here_now_C0R_accN or Here_now_C0R_mvN1 or not(Here_now_C0R_mvN2) or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
	                 Here_now_C0R_accN or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or not(Here_now_C0R_mvN3) or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3      or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
	                 Here_now_C0R_accN or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or not(Here_now_C0R_mvN0) or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE      or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW      or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN      or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 not(Here_now_C0R_accS) or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1      or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW or Here_now_C0R_mvW1      or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN or Here_now_C0R_mvN1      or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or not(Here_now_C0R_mvS1) or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1 or Here_now_C0R_mvE0      or
                         Here_now_C0R_accW or Here_now_C0R_mvW1 or Here_now_C0R_mvW0      or
                         Here_now_C0R_accN or Here_now_C0R_mvN1 or Here_now_C0R_mvN2      or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or not(Here_now_C0R_mvS2) or Here_now_C0R_mvS3 or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving or
	                 Here_now_C0R_accE or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or Here_now_C0R_mvN3      or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or not(Here_now_C0R_mvS3) or Here_now_C0R_mvS0;
	m.addClause(c);
	c = Here_now_C0R_moving
	                 or  Here_now_C0R_accE or Here_now_C0R_mvE1 or Here_now_C0R_mvE0 or
                         Here_now_C0R_accW or Here_now_C0R_mvW1 or Here_now_C0R_mvW0 or
                         Here_now_C0R_accN or Here_now_C0R_mvN1 or Here_now_C0R_mvN2 or Here_now_C0R_mvN3 or Here_now_C0R_mvN0 or
	                 Here_now_C0R_accS or Here_now_C0R_mvS1 or Here_now_C0R_mvS2 or Here_now_C0R_mvS3 or not(Here_now_C0R_mvS0);
	m.addClause(c);
	// 16 choose 2
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvE1);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvE0);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_accW);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvW1);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvW0);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_accN);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvE0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_accW);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvW1);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvW0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_accN);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_accW);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvW1);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvW0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_accN);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvW1);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvW0);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_accN);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvW0);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_accN);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_accN);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C0R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C0R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C0R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C0R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_accS) or not(Here_now_C0R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_accS) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_accS) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_accS) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvS1) or not(Here_now_C0R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvS1) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvS1) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvS2) or not(Here_now_C0R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvS2) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvS3) or not(Here_now_C0R_mvS0);
	m.addClause(c);
	
        // Car 1

	CNF::Var Here_now_C1R_accE = var(v,    t,     R_Move::w1_accE);
	CNF::Var Here_now_C1R_mvE1 = var(v,    t,     R_Move::w1_mvE1);
	CNF::Var Here_now_C1R_mvE0 = var(v,    t,     R_Move::w1_mvE0);
	CNF::Var Here_now_C1R_accW = var(v,    t,     R_Move::w1_accW);
	CNF::Var Here_now_C1R_mvW1 = var(v,    t,     R_Move::w1_mvW1);
	CNF::Var Here_now_C1R_mvW0 = var(v,    t,     R_Move::w1_mvW0);
	CNF::Var Here_now_C1R_accN = var(v,    t,     R_Move::w1_accN);
	CNF::Var Here_now_C1R_mvN1 = var(v,    t,     R_Move::w1_mvN1);
	CNF::Var Here_now_C1R_mvN2 = var(v,    t,     R_Move::w1_mvN2);
	CNF::Var Here_now_C1R_mvN3 = var(v,    t,     R_Move::w1_mvN3);
	CNF::Var Here_now_C1R_mvN0 = var(v,    t,     R_Move::w1_mvN0);
	CNF::Var Here_now_C1R_accS = var(v,    t,     R_Move::w1_accS);
	CNF::Var Here_now_C1R_mvS1 = var(v,    t,     R_Move::w1_mvS1);
	CNF::Var Here_now_C1R_mvS2 = var(v,    t,     R_Move::w1_mvS2);
	CNF::Var Here_now_C1R_mvS3 = var(v,    t,     R_Move::w1_mvS3);
	CNF::Var Here_now_C1R_mvS0 = var(v,    t,     R_Move::w1_mvS0);

	//necceraly
	c = not(Here_now_C1R_moving) or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	// sufficient
	// 16 choose 1
	c = Here_now_C1R_moving or
	                 not(Here_now_C1R_accE) or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW      or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN      or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS      or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
		         Here_now_C1R_accE or not(Here_now_C1R_mvE1) or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW or Here_now_C1R_mvW1      or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN or Here_now_C1R_mvN1      or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1      or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or not(Here_now_C1R_mvE0) or
                         Here_now_C1R_accW or Here_now_C1R_mvW1 or Here_now_C1R_mvW0      or
                         Here_now_C1R_accN or Here_now_C1R_mvN1 or Here_now_C1R_mvN2      or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or Here_now_C1R_mvS2      or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE      or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
	                 not(Here_now_C1R_accW) or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN      or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS      or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1      or Here_now_C1R_mvE0 or
	                 Here_now_C1R_accW or not(Here_now_C1R_mvW1) or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN or Here_now_C1R_mvN1      or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1      or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or Here_now_C1R_mvE0      or
	                 Here_now_C1R_accW or Here_now_C1R_mvW1 or not(Here_now_C1R_mvW0) or
                         Here_now_C1R_accN or Here_now_C1R_mvN1 or Here_now_C1R_mvN2      or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or Here_now_C1R_mvS2      or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE      or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW      or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
	                 not(Here_now_C1R_accN) or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS      or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1      or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW or Here_now_C1R_mvW1      or Here_now_C1R_mvW0 or
	                 Here_now_C1R_accN or not(Here_now_C1R_mvN1) or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1      or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or Here_now_C1R_mvE0      or
                         Here_now_C1R_accW or Here_now_C1R_mvW1 or Here_now_C1R_mvW0      or
	                 Here_now_C1R_accN or Here_now_C1R_mvN1 or not(Here_now_C1R_mvN2) or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or Here_now_C1R_mvS2      or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
	                 Here_now_C1R_accN or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or not(Here_now_C1R_mvN3) or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or Here_now_C1R_mvS3      or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
	                 Here_now_C1R_accN or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or not(Here_now_C1R_mvN0) or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE      or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW      or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN      or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 not(Here_now_C1R_accS) or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1      or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW or Here_now_C1R_mvW1      or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN or Here_now_C1R_mvN1      or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or not(Here_now_C1R_mvS1) or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or Here_now_C1R_mvE0      or
                         Here_now_C1R_accW or Here_now_C1R_mvW1 or Here_now_C1R_mvW0      or
                         Here_now_C1R_accN or Here_now_C1R_mvN1 or Here_now_C1R_mvN2      or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or not(Here_now_C1R_mvS2) or Here_now_C1R_mvS3 or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or Here_now_C1R_mvN3      or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or not(Here_now_C1R_mvS3) or Here_now_C1R_mvS0;
	m.addClause(c);
	c = Here_now_C1R_moving or
	                 Here_now_C1R_accE or Here_now_C1R_mvE1 or Here_now_C1R_mvE0 or
                         Here_now_C1R_accW or Here_now_C1R_mvW1 or Here_now_C1R_mvW0 or
                         Here_now_C1R_accN or Here_now_C1R_mvN1 or Here_now_C1R_mvN2 or Here_now_C1R_mvN3 or Here_now_C1R_mvN0 or
	                 Here_now_C1R_accS or Here_now_C1R_mvS1 or Here_now_C1R_mvS2 or Here_now_C1R_mvS3 or not(Here_now_C1R_mvS0);
	m.addClause(c);
	// 16 choose 2
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvE1);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvE0);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_accW);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvW1);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvW0);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_accN);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvS1);
	m.addClause(c)
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvE0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_accW);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvW1);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvW0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_accN);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_accW);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvW1);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvW0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_accN);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvW1);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvW0);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_accN);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvW0);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_accN);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_accN);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN3) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN3) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_mvN3) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvN3) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvN3) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvN3) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN0) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_mvN0) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvN0) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvN0) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvN0) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_accS) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_accS) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_accS) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_accS) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvS1) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvS1) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvS1) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvS2) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvS2) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvS3) or not(Here_now_C1R_mvS0);
	m.addClause(c);
        // Car 2

	CNF::Var Here_now_C2R_accE = var(v,    t,     R_Move::w2_accE);
	CNF::Var Here_now_C2R_mvE1 = var(v,    t,     R_Move::w2_mvE1);
	CNF::Var Here_now_C2R_mvE0 = var(v,    t,     R_Move::w2_mvE0);
	CNF::Var Here_now_C2R_accW = var(v,    t,     R_Move::w2_accW);
	CNF::Var Here_now_C2R_mvW1 = var(v,    t,     R_Move::w2_mvW1);
	CNF::Var Here_now_C2R_mvW0 = var(v,    t,     R_Move::w2_mvW0);
	CNF::Var Here_now_C2R_accN = var(v,    t,     R_Move::w2_accN);
	CNF::Var Here_now_C2R_mvN1 = var(v,    t,     R_Move::w2_mvN1);
	CNF::Var Here_now_C2R_mvN2 = var(v,    t,     R_Move::w2_mvN2);
	CNF::Var Here_now_C2R_mvN3 = var(v,    t,     R_Move::w2_mvN3);
	CNF::Var Here_now_C2R_mvN0 = var(v,    t,     R_Move::w2_mvN0);
	CNF::Var Here_now_C2R_accS = var(v,    t,     R_Move::w2_accS);
	CNF::Var Here_now_C2R_mvS1 = var(v,    t,     R_Move::w2_mvS1);
	CNF::Var Here_now_C2R_mvS2 = var(v,    t,     R_Move::w2_mvS2);
	CNF::Var Here_now_C2R_mvS3 = var(v,    t,     R_Move::w2_mvS3);
	CNF::Var Here_now_C2R_mvS0 = var(v,    t,     R_Move::w2_mvS0);

	//necceraly
	c = not(Here_now_C2R_moving) or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	// sufficient
	// 16 choose 1
	c = Here_now_C2R_moving or
	                 not(Here_now_C2R_accE) or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW      or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN      or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS      or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
		         Here_now_C2R_accE or not(Here_now_C2R_mvE1) or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW or Here_now_C2R_mvW1      or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN or Here_now_C2R_mvN1      or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1      or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or not(Here_now_C2R_mvE0) or
                         Here_now_C2R_accW or Here_now_C2R_mvW1 or Here_now_C2R_mvW0      or
                         Here_now_C2R_accN or Here_now_C2R_mvN1 or Here_now_C2R_mvN2      or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or Here_now_C2R_mvS2      or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE      or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
	                 not(Here_now_C0R_accW) or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN      or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS      or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1      or Here_now_C2R_mvE0 or
	                 Here_now_C2R_accW or not(Here_now_C2R_mvW1) or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN or Here_now_C2R_mvN1      or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1      or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or Here_now_C2R_mvE0      or
	                 Here_now_C2R_accW or Here_now_C2R_mvW1 or not(Here_now_C2R_mvW0) or
                         Here_now_C2R_accN or Here_now_C2R_mvN1 or Here_now_C2R_mvN2      or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or Here_now_C2R_mvS2      or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE      or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW      or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
	                 not(Here_now_C2R_accN) or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS      or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1      or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW or Here_now_C2R_mvW1      or Here_now_C2R_mvW0 or
	                 Here_now_C2R_accN or not(Here_now_C2R_mvN1) or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1      or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or Here_now_C2R_mvE0      or
                         Here_now_C2R_accW or Here_now_C2R_mvW1 or Here_now_C2R_mvW0      or
	                 Here_now_C2R_accN or Here_now_C2R_mvN1 or not(Here_now_C2R_mvN2) or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or Here_now_C2R_mvS2      or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
	                 Here_now_C2R_accN or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or not(Here_now_C2R_mvN3) or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or Here_now_C2R_mvS3      or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
	                 Here_now_C2R_accN or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or not(Here_now_C2R_mvN0) or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE      or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW      or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN      or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 not(Here_now_C2R_accS) or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1      or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW or Here_now_C2R_mvW1      or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN or Here_now_C2R_mvN1      or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or not(Here_now_C2R_mvS1) or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or Here_now_C2R_mvE0      or
                         Here_now_C2R_accW or Here_now_C2R_mvW1 or Here_now_C2R_mvW0      or
                         Here_now_C2R_accN or Here_now_C2R_mvN1 or Here_now_C2R_mvN2      or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or not(Here_now_C2R_mvS2) or Here_now_C2R_mvS3 or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or Here_now_C2R_mvN3      or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or not(Here_now_C2R_mvS3) or Here_now_C2R_mvS0;
	m.addClause(c);
	c = Here_now_C2R_moving or
	                 Here_now_C2R_accE or Here_now_C2R_mvE1 or Here_now_C2R_mvE0 or
                         Here_now_C2R_accW or Here_now_C2R_mvW1 or Here_now_C2R_mvW0 or
                         Here_now_C2R_accN or Here_now_C2R_mvN1 or Here_now_C2R_mvN2 or Here_now_C2R_mvN3 or Here_now_C2R_mvN0 or
	                 Here_now_C2R_accS or Here_now_C2R_mvS1 or Here_now_C2R_mvS2 or Here_now_C2R_mvS3 or not(Here_now_C2R_mvS0);
	m.addClause(c);
	// 16 choose 2
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvE1);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvE0);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_accW);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvW1);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvW0);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_accN);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_accE) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvE0);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_accW);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvW1);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvW0);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_accN);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvE1) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_accW);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvW1);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvW0);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_accN);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvE0) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvW1);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvW0);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_accN);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_accW) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvW0);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_accN);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvW1) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_accN);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvW0) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_accN) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvN1) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C2R_mvN1) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_mvN1) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_mvN1) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_mvN1) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_mvN1) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvN1) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvN1) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvN2) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C2R_mvN2) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_mvN2) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_mvN2) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_mvN2) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvN2) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvN2) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvN3) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C2R_mvN3) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_mvN3) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_mvN3) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvN3) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvN3) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvN0) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C2R_mvN0) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_mvN0) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvN0) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvN0) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_accS) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C2R_accS) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_accS) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_accS) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvS1) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C2R_mvS1) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvS1) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvS2) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C2R_mvS2) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C2R_mvS3) or not(Here_now_C2R_mvS0);
	m.addClause(c);
    } // endof B A S I C

    // M O V E M E N T
    {
        CNF::Var Here_now_nobodyhome= var(v,    t,     NdStat::nobodyhome);
	CNF::Var Here_now_R_ready   = var(v,    t,     NdStat::R_ready);
	CNF::Var Here_now_R_accE    = var(v,    t,     R_Move::accE);
	CNF::Var Here_now_R_mvE0    = var(v,    t,     R_Move::mvE0);
	CNF::Var Here_now_R_accW    = var(v,    t,     R_Move::accW);
	CNF::Var Here_now_R_mvW0    = var(v,    t,     R_Move::mvW0);
	CNF::Var Here_now_R_accN    = var(v,    t,     R_Move::accN);
	CNF::Var Here_now_R_mvN1    = var(v,    t,     R_Move::mvN1);
	CNF::Var Here_now_R_mvN0    = var(v,    t,     R_Move::mvN0);
	CNF::Var Here_now_R_accS    = var(v,    t,     R_Move::accS);
	CNF::Var Here_now_R_mvS1    = var(v,    t,     R_Move::mvS1);
	CNF::Var Here_now_R_mvS0    = var(v,    t,     R_Move::mvS0);
	CNF::Var Here_now_R_lift    = var(v,    t,     R_Vertical::lift);
	CNF::Var Here_now_R_lifting1= var(v,    t,     R_Vertical::l1);
	CNF::Var Here_now_R_lifting2= var(v,    t,     R_Vertical::l2);
	CNF::Var Here_now_R_lifting3= var(v,    t,     R_Vertical::l3);
	CNF::Var Here_now_R_lifting4= var(v,    t,     R_Vertical::l4);
	CNF::Var Here_now_R_drop    = var(v,    t,     R_Vertical::drop);
	CNF::Var Here_now_C0R_ready = var(v,    t,     NdStat::C0R_ready);
	CNF::Var Here_now_C0R_accE  = var(v,    t,     R_Move::w0_accE);
	CNF::Var Here_now_C0R_mvE1  = var(v,    t,     R_Move::w0_mvE1);
	CNF::Var Here_now_C0R_mvE0  = var(v,    t,     R_Move::w0_mvE0);
	CNF::Var Here_now_C0R_accW  = var(v,    t,     R_Move::w0_accW);
	CNF::Var Here_now_C0R_mvW1  = var(v,    t,     R_Move::w0_mvW1);
	CNF::Var Here_now_C0R_mvW0  = var(v,    t,     R_Move::w0_mvW0);
	CNF::Var Here_now_C0R_accN  = var(v,    t,     R_Move::w0_accN);
	CNF::Var Here_now_C0R_mvN1  = var(v,    t,     R_Move::w0_mvN1);
	CNF::Var Here_now_C0R_mvN2  = var(v,    t,     R_Move::w0_mvN2);
	CNF::Var Here_now_C0R_mvN3  = var(v,    t,     R_Move::w0_mvN3);
	CNF::Var Here_now_C0R_mvN0  = var(v,    t,     R_Move::w0_mvN0);
	CNF::Var Here_now_C0R_accS  = var(v,    t,     R_Move::w0_accS);
	CNF::Var Here_now_C0R_mvS1  = var(v,    t,     R_Move::w0_mvS1);
	CNF::Var Here_now_C0R_mvS2  = var(v,    t,     R_Move::w0_mvS2);
	CNF::Var Here_now_C0R_mvS3  = var(v,    t,     R_Move::w0_mvS3);
	CNF::Var Here_now_C0R_mvS0  = var(v,    t,     R_Move::w0_mvS0);
	CNF::Var Here_now_C1R_ready = var(v,    t,     NdStat::C1R_ready);
	CNF::Var Here_now_C1R_accE  = var(v,    t,     R_Move::w1_accE);
	CNF::Var Here_now_C1R_mvE1  = var(v,    t,     R_Move::w1_mvE1);
	CNF::Var Here_now_C1R_mvE0  = var(v,    t,     R_Move::w1_mvE0);
	CNF::Var Here_now_C1R_accW  = var(v,    t,     R_Move::w1_accW);
	CNF::Var Here_now_C1R_mvW1  = var(v,    t,     R_Move::w1_mvW1);
	CNF::Var Here_now_C1R_mvW0  = var(v,    t,     R_Move::w1_mvW0);
	CNF::Var Here_now_C1R_accN  = var(v,    t,     R_Move::w1_accN);
	CNF::Var Here_now_C1R_mvN1  = var(v,    t,     R_Move::w1_mvN1);
	CNF::Var Here_now_C1R_mvN2  = var(v,    t,     R_Move::w1_mvN2);
	CNF::Var Here_now_C1R_mvN3  = var(v,    t,     R_Move::w1_mvN3);
	CNF::Var Here_now_C1R_mvN0  = var(v,    t,     R_Move::w1_mvN0);
	CNF::Var Here_now_C1R_accS  = var(v,    t,     R_Move::w1_accS);
	CNF::Var Here_now_C1R_mvS1  = var(v,    t,     R_Move::w1_mvS1);
	CNF::Var Here_now_C1R_mvS2  = var(v,    t,     R_Move::w1_mvS2);
	CNF::Var Here_now_C1R_mvS3  = var(v,    t,     R_Move::w1_mvS3);
	CNF::Var Here_now_C1R_mvS0  = var(v,    t,     R_Move::w1_mvS0);
	CNF::Var Here_now_C2R_ready = var(v,    t,     NdStat::C2R_ready);
	CNF::Var Here_now_C2R_accE  = var(v,    t,     R_Move::w2_accE);
	CNF::Var Here_now_C2R_mvE1  = var(v,    t,     R_Move::w2_mvE1);
	CNF::Var Here_now_C2R_mvE0  = var(v,    t,     R_Move::w2_mvE0);
	CNF::Var Here_now_C2R_accW  = var(v,    t,     R_Move::w2_accW);
	CNF::Var Here_now_C2R_mvW1  = var(v,    t,     R_Move::w2_mvW1);
	CNF::Var Here_now_C2R_mvW0  = var(v,    t,     R_Move::w2_mvW0);
	CNF::Var Here_now_C2R_accN  = var(v,    t,     R_Move::w2_accN);
	CNF::Var Here_now_C2R_mvN1  = var(v,    t,     R_Move::w2_mvN1);
	CNF::Var Here_now_C2R_mvN2  = var(v,    t,     R_Move::w2_mvN2);
	CNF::Var Here_now_C2R_mvN3  = var(v,    t,     R_Move::w2_mvN3);
	CNF::Var Here_now_C2R_mvN0  = var(v,    t,     R_Move::w2_mvN0);
	CNF::Var Here_now_C2R_accS  = var(v,    t,     R_Move::w2_accS);
	CNF::Var Here_now_C2R_mvS1  = var(v,    t,     R_Move::w2_mvS1);
	CNF::Var Here_now_C2R_mvS2  = var(v,    t,     R_Move::w2_mvS2);
	CNF::Var Here_now_C2R_mvS3  = var(v,    t,     R_Move::w2_mvS3);
	CNF::Var Here_now_C2R_mvS0  = var(v,    t,     R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var Here_now_Cr_accE ;
	// neccessarly
	c = not(Here_now_CR_accE)  or Here_now_C0R_accE      or Here_now_C1R_accE      or Here_now_C2R_accE;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accE) or Here_now_C1R_accE      or Here_now_C2R_accE      or Here_now_CR_accE;
	m.addClause(c);
	c = Here_now_C0R_accE      or not(Here_now_C1R_accE) or Here_now_C2R_accE      or Here_now_CR_accE;
	m.addClause(c);
	c = Here_now_C0R_accE      or Here_now_C1R_accE      or not(Here_now_C2R_accE) or Here_now_CR_accE;
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C1R_accE);
	m.addClause(c);
	c = not(Here_now_C0R_accE) or not(Here_now_C2R_accE);
	m.addClause(c);
	c = not(Here_now_C1R_accE) or not(Here_now_C2R_accE);
	m.addClause(c);
	CNF::Var Here_now_CR_mvE1;
	// necessarly
	c = not(Here_now_CR_mvE1)  or Here_now_C0R_mvE1      or Here_now_C1R_mvE1      or Here_now_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvE1) or Here_now_C1R_mvE1      or Here_now_C2R_mvE1      or Here_now_CR_mvE1;
	m.addClause(c);
	c = Here_now_C0R_mvE1      or not(Here_now_C1R_mvE1) or Here_now_C2R_mvE1      or Here_now_CR_mvE1;
	m.addClause(c);
	c = Here_now_C0R_mvE1      or Here_now_C1R_mvE1      or not(Here_now_C2R_mvE1) or Here_now_CR_mvE1;
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C1R_mvE1);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C2R_mvE1);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C2R_mvE1);
	m.addClause(c);
	CNF::Var Here_now_CR_mvE0;
	// necessalry
	c = not(Here_now_CR_mvE0)  or Here_now_C0R_mvE0      or Here_now_C1R_mvE0      or Here_now_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvE0) or Here_now_C1R_mvE0      or Here_now_C2R_mvE0      or Here_now_CR_mvE0;
	m.addClause(c);
	c = Here_now_C0R_mvE0      or not(Here_now_C1R_mvE0) or Here_now_C2R_mvE0      or Here_now_CR_mvE0;
	m.addClause(c);
	c = Here_now_C0R_mvE0      or Here_now_C1R_mvE0      or not(Here_now_C2R_mvE0) or Here_now_CR_mvE0;
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C1R_mvE0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C2R_mvE0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C2R_mvE0);
	m.addClause(c);

	CNF::Var Here_now_CR_accW;
	// necessarly
	c = not(Here_now_CR_accW)  or Here_now_C0R_accW      or Here_now_C1R_accW      or Here_now_C2R_accW;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accW) or Here_now_C1R_accW      or Here_now_C2R_accW      or Here_now_CR_accW;
	m.addClause(c);
	c = Here_now_C0R_accW      or not(Here_now_C1R_accW) or Here_now_C2R_accW      or Here_now_CR_accW;
	m.addClause(c);
	c = Here_now_C0R_accW      or Here_now_C1R_accW      or not(Here_now_C2R_accW) or Here_now_CR_accW;
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C1R_accW);
	m.addClause(c);
	c = not(Here_now_C0R_accW) or not(Here_now_C2R_accW);
	m.addClause(c);
	c = not(Here_now_C1R_accW) or not(Here_now_C2R_accW);
	m.addClause(c);
	CNF::Var Here_now_CR_mvW1;
	// necessarly
	c = not(Here_now_CR_mvW1)  or Here_now_C0R_mvW1      or Here_now_C1R_mvW1      or Here_now_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvW1) or Here_now_C1R_mvW1      or Here_now_C2R_mvW1      or Here_now_CR_mvW1;
	m.addClause(c);
	c = Here_now_C0R_mvW1      or not(Here_now_C1R_mvW1) or Here_now_C2R_mvW1      or Here_now_CR_mvW1;
	m.addClause(c);
	c = Here_now_C0R_mvW1      or Here_now_C1R_mvW1      or not(Here_now_C2R_mvW1) or Here_now_CR_mvW1;
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C1R_mvW1);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C2R_mvW1);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C2R_mvW1);
	m.addClause(c);
	CNF::Var Here_now_CR_mvW0;
	// necessarly
	c = not(Here_now_CR_mvW0)  or Here_now_C0R_mvW0      or Here_now_C1R_mvW0      or Here_now_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvW0) or Here_now_C1R_mvW0      or Here_now_C2R_mvW0      or Here_now_CR_mvW0;
	m.addClause(c);
	c = Here_now_C0R_mvW0      or not(Here_now_C1R_mvW0) or Here_now_C2R_mvW0      or Here_now_CR_mvW0;
	m.addClause(c);
	c = Here_now_C0R_mvW0      or Here_now_C1R_mvW0      or not(Here_now_C2R_mvW0) or Here_now_CR_mvW0;
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C1R_mvW0);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C2R_mvW0);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C2R_mvW0);
	m.addClause(c);

	CNF::Var Here_now_CR_accN;
	// neccessarly
	c = not(Here_now_CR_accN)  or Here_now_C0R_accN      or Here_now_C1R_accN      or Here_now_C2R_accN;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accN) or Here_now_C1R_accN      or Here_now_C2R_accN      or Here_now_CR_accN;
	m.addClause(c);
	c = Here_now_C0R_accN      or not(Here_now_C1R_accN) or Here_now_C2R_accN      or Here_now_CR_accN;
	m.addClause(c);
	c = Here_now_C0R_accN      or Here_now_C1R_accN      or not(Here_now_C2R_accN) or Here_now_CR_accN;
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C1R_accN);
	m.addClause(c);
	c = not(Here_now_C0R_accN) or not(Here_now_C2R_accN);
	m.addClause(c);
	c = not(Here_now_C1R_accN) or not(Here_now_C2R_accN);
	m.addClause(c);
	CNF::Var Here_now_CR_mvN1;
	// neccessarly
	c = not(Here_now_CR_mvN1)  or Here_now_C0R_mvN1      or Here_now_C1R_mvN1      or Here_now_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvN1) or Here_now_C1R_mvN1      or Here_now_C2R_mvN1      or Here_now_CR_mvN1;
	m.addClause(c);
	c = Here_now_C0R_mvN1      or not(Here_now_C1R_mvN1) or Here_now_C2R_mvN1      or Here_now_CR_mvN1;
	m.addClause(c);
	c = Here_now_C0R_mvN1      or Here_now_C1R_mvN1      or not(Here_now_C2R_mvN1) or Here_now_CR_mvN1;
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	CNF::Var Here_now_CR_mvN2;
	// neccessarly
	c = not(Here_now_CR_mvN2)  or Here_now_C0R_mvN2      or Here_now_C1R_mvN2       or Here_now_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvN2) or Here_now_C1R_mvN2      or Here_now_C2R_mvN2       or Here_now_CR_mvN2;
	m.addClause(c);
	c = Here_now_C0R_mvN2      or not(Here_now_C1R_mvN2) or Here_now_C2R_mvN2       or Here_now_CR_mvN2;
	m.addClause(c);
	c = Here_now_C0R_mvN2      or Here_now_C1R_mvN2      or not(Here_now_C2R_mvN2)  or Here_now_CR_mvN2;
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	CNF::Var Here_now_CR_mvN3;
	// neccessarly
	c = not(Here_now_CR_mvN3)  or Here_now_C0R_mvN3      or Here_now_C1R_mvN3      or Here_now_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvN3) or Here_now_C1R_mvN3      or Here_now_C2R_mvN3      or Here_now_CR_mvN3;
	m.addClause(c);
	c = Here_now_C0R_mvN3      or not(Here_now_C1R_mvN3) or Here_now_C2R_mvN3      or Here_now_CR_mvN3;
	m.addClause(c);
	c = Here_now_C0R_mvN3      or Here_now_C1R_mvN3      or not(Here_now_C2R_mvN3) or Here_now_CR_mvN3;
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_mvN3) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	CNF::Var Here_now_CR_mvN0;
	// neccessarly
	c = not(Here_now_CR_mvN0)  or Here_now_C0R_mvN0      or Here_now_C1R_mvN0      or Here_now_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvN0) or Here_now_C1R_mvN0      or Here_now_C2R_mvN0      or Here_now_CR_mvN0;
	m.addClause(c);
	c = Here_now_C0R_mvN0      or not(Here_now_C1R_mvN0) or Here_now_C2R_mvN0      or Here_now_CR_mvN0;
	m.addClause(c);
	c = Here_now_C0R_mvN0      or Here_now_C1R_mvN0      or not(Here_now_C2R_mvN0) or Here_now_CR_mvN0;
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN0) or not(Here_now_C2R_mvN0);
	m.addClause(c);


	CNF::Var Here_now_CR_accS;
	// neccessarly
	c = not(Here_now_CR_accS)  or Here_now_C0R_accS      or Here_now_C1R_accS      or Here_now_C2R_accS;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accS) or Here_now_C1R_accS      or Here_now_C2R_accS      or Here_now_CR_accS;
	m.addClause(c);
	c = Here_now_C0R_accS      or not(Here_now_C1R_accS) or Here_now_C2R_accS      or Here_now_CR_accS;
	m.addClause(c);
	c = Here_now_C0R_accS      or Here_now_C1R_accS      or not(Here_now_C2R_accS) or Here_now_CR_accS;
	m.addClause(c);
	c = not(Here_now_C0R_accS) or not(Here_now_C1R_accS);
	m.addClause(c);
	c = not(Here_now_C0R_accS) or not(Here_now_C2R_accS);
	m.addClause(c);
	c = not(Here_now_C1R_accS) or not(Here_now_C2R_accS);
	m.addClause(c);
	CNF::Var Here_now_CR_mvS1;
	// neccessarly
	c = not(Here_now_CR_mvS1)  or Here_now_C0R_mvS1      or Here_now_C1R_mvS1      or Here_now_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvS1) or Here_now_C1R_mvS1      or Here_now_C2R_mvS1      or Here_now_CR_mvS1;
	m.addClause(c);
	c = Here_now_C0R_mvS1      or not(Here_now_C1R_mvS1) or Here_now_C2R_mvS1      or Here_now_CR_mvS1;
	m.addClause(c);
	c = Here_now_C0R_mvS1      or Here_now_C1R_mvS1      or not(Here_now_C2R_mvS1) or Here_now_CR_mvS1;
	m.addClause(c);
	c = not(Here_now_C0R_mvS1) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvS1) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvS1) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	CNF::Var Here_now_CR_mvS2;
	// neccessarly
	c = not(Here_now_CR_mvS2)  or Here_now_C0R_mvS2      or Here_now_C1R_mvS2      or Here_now_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvS2) or Here_now_C1R_mvS2      or Here_now_C2R_mvS2      or Here_now_CR_mvS2;
	m.addClause(c);
	c = Here_now_C0R_mvS2      or not(Here_now_C1R_mvS2) or Here_now_C2R_mvS2      or Here_now_CR_mvS2;
	m.addClause(c);
	c = Here_now_C0R_mvS2      or Here_now_C1R_mvS2      or not(Here_now_C2R_mvS2) or Here_now_CR_mvS2;
	m.addClause(c);
	c = not(Here_now_C0R_mvS2) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvS2) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvS2) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	CNF::Var Here_now_CR_mvS3;
	// neccessarly
	c = not(Here_now_CR_mvS3)  or Here_now_C0R_mvS3      or Here_now_C1R_mvS3      or Here_now_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvS3) or Here_now_C1R_mvS3      or Here_now_C2R_mvS3      or Here_now_CR_mvS3;
	m.addClause(c);
	c = Here_now_C0R_mvS3      or not(Here_now_C1R_mvS3) or Here_now_C2R_mvS3      or Here_now_CR_mvS3;
	m.addClause(c);
	c = Here_now_C0R_mvS3      or Here_now_C1R_mvS3      or not(Here_now_C2R_mvS3) or Here_now_CR_mvS3;
	m.addClause(c);
	c = not(Here_now_C0R_mvS3) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvS3) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvS3) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	CNF::Var Here_now_CR_mvS0;
	// neccessarly
	c = not(Here_now_CR_mvS0)  or Here_now_C0R_mvS0      or Here_now_C1R_mvS0      or Here_now_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvS0) or Here_now_C1R_mvS0      or Here_now_C2R_mvS0      or Here_now_CR_mvS0;
	m.addClause(c);
	c = Here_now_C0R_mvS0      or not(Here_now_C1R_mvS0) or Here_now_C2R_mvS0      or Here_now_CR_mvS0;
	m.addClause(c);
	c = Here_now_C0R_mvS0      or Here_now_C1R_mvS0      or not(Here_now_C2R_mvS0) or Here_now_CR_mvS0;
	m.addClause(c);
	c = not(Here_now_C0R_mvS0) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvS0) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvS0) or not(Here_now_C2R_mvS0);
	m.addClause(c);


	CNF::Var E_now_nobodyhome= var( G.east(v),    t,    NdStat::nobodyhome);
	CNF::Var E_now_R_ready   = var( G.east(v),    t,    NdStat::R_ready);
	CNF::Var E_now_R_accE    = var( G.east(v),    t,    R_Move::accE);
	CNF::Var E_now_R_mvE0    = var( G.east(v),    t,    R_Move::mvE0);
	CNF::Var E_now_R_accW    = var( G.east(v),    t,    R_Move::accW);
	CNF::Var E_now_R_mvW0    = var( G.east(v),    t,    R_Move::mvW0);
	CNF::Var E_now_C0R_ready = var( G.east(v),    t,    NdStat::C0R_ready);
	CNF::Var E_now_C0R_accE  = var( G.east(v),    t,    R_Move::w0_accE);
	CNF::Var E_now_C0R_mvE1  = var( G.east(v),    t,    R_Move::w0_mvE1);
	CNF::Var E_now_C0R_mvE0  = var( G.east(v),    t,    R_Move::w0_mvE0);
	CNF::Var E_now_C0R_accW  = var( G.east(v),    t,    R_Move::w0_accW);
	CNF::Var E_now_C0R_mvW1  = var( G.east(v),    t,    R_Move::w0_mvW1);
	CNF::Var E_now_C0R_mvW0  = var( G.east(v),    t,    R_Move::w0_mvW0);
	CNF::Var E_now_C1R_ready = var( G.east(v),    t,    NdStat::C1R_ready);
	CNF::Var E_now_C1R_accE  = var( G.east(v),    t,    R_Move::w1_accE);
	CNF::Var E_now_C1R_mvE1  = var( G.east(v),    t,    R_Move::w1_mvE1);
	CNF::Var E_now_C1R_mvE0  = var( G.east(v),    t,    R_Move::w1_mvE0);
	CNF::Var E_now_C1R_accW  = var( G.east(v),    t,    R_Move::w1_accW);
	CNF::Var E_now_C1R_mvW1  = var( G.east(v),    t,    R_Move::w1_mvW1);
	CNF::Var E_now_C1R_mvW0  = var( G.east(v),    t,    R_Move::w1_mvW0);
	CNF::Var E_now_C2R_ready = var( G.east(v),    t,    NdStat::C2R_ready);
	CNF::Var E_now_C2R_accE  = var( G.east(v),    t,    R_Move::w2_accE);
	CNF::Var E_now_C2R_mvE1  = var( G.east(v),    t,    R_Move::w2_mvE1);
	CNF::Var E_now_C2R_mvE0  = var( G.east(v),    t,    R_Move::w2_mvE0);
	CNF::Var E_now_C2R_accW  = var( G.east(v),    t,    R_Move::w2_accW);
	CNF::Var E_now_C2R_mvW1  = var( G.east(v),    t,    R_Move::w2_mvW1);
	CNF::Var E_now_C2R_mvW0  = var( G.east(v),    t,    R_Move::w2_mvW0);

        // Abbreviations
	CNF::Var E_now_CR_accE;
	// neccessarly
	c = not(E_now_CR_accE)  or E_now_C0R_accE      or E_now_C1R_accE      or E_now_C2R_accE;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_accE) or E_now_C1R_accE      or E_now_C2R_accE      or E_now_CR_accE;
	m.addClause(c);
	c = E_now_C0R_accE      or not(E_now_C1R_accE) or E_now_C2R_accE      or E_now_CR_accE;
	m.addClause(c);
	c = E_now_C0R_accE      or E_now_C1R_accE      or not(E_now_C2R_accE) or E_now_CR_accE;
	m.addClause(c);
	c = not(E_now_C0R_accE) or not(E_now_C1R_accE);
	m.addClause(c);
	c = not(E_now_C0R_accE) or not(E_now_C2R_accE);
	m.addClause(c);
	c = not(E_now_C1R_accE) or not(E_now_C2R_accE);
	m.addClause(c);
	CNF::Var E_now_CR_mvE1;
	// neccessarly
	c = not(E_now_CR_mvE1)  or E_now_C0R_mvE1      or E_now_C1R_mvE1      or E_now_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_mvE1) or E_now_C1R_mvE1      or E_now_C2R_mvE1      or E_now_CR_mvE1;
	m.addClause(c);
	c = E_now_C0R_mvE1      or not(E_now_C1R_mvE1) or E_now_C2R_mvE1      or E_now_CR_mvE1;
	m.addClause(c);
	c = E_now_C0R_mvE1      or E_now_C1R_mvE1      or not(E_now_C2R_mvE1) or E_now_CR_mvE1;
	m.addClause(c);
	c = not(E_now_C0R_mvE1) or not(E_now_C1R_mvE1);
	m.addClause(c);
	c = not(E_now_C0R_mvE1) or not(E_now_C2R_mvE1);
	m.addClause(c);
	c = not(E_now_C1R_mvE1) or not(E_now_C2R_mvE1);
	m.addClause(c);
	CNF::Var E_now_CR_mvE0;
	// neccessarly
	c = not(E_now_CR_mvE0)  or E_now_C0R_mvE0      or E_now_C1R_mvE0      or E_now_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_mvE0) or E_now_C1R_mvE0      or E_now_C2R_mvE0      or E_now_CR_mvE0;
	m.addClause(c);
	c = E_now_C0R_mvE0      or not(E_now_C1R_mvE0) or E_now_C2R_mvE0      or E_now_CR_mvE0;
	m.addClause(c);
	c = E_now_C0R_mvE0      or E_now_C1R_mvE0      or not(E_now_C2R_mvE0) or E_now_CR_mvE0;
	m.addClause(c);
	c = not(E_now_C0R_mvE0) or not(E_now_C1R_mvE0);
	m.addClause(c);
	c = not(E_now_C0R_mvE0) or not(E_now_C2R_mvE0);
	m.addClause(c);
	c = not(E_now_C1R_mvE0) or not(E_now_C2R_mvE0);
	m.addClause(c);

	CNF::Var E_now_CR_accW;
	// neccessarly
	c = not(E_now_CR_accW)  or E_now_C0R_accW      or E_now_C1R_accW      or E_now_C2R_accW;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_accW) or E_now_C1R_accW      or E_now_C2R_accW      or E_now_CR_accW;
	m.addClause(c);
	c = E_now_C0R_accW      or not(E_now_C1R_accW) or E_now_C2R_accW      or E_now_CR_accW;
	m.addClause(c);
	c = E_now_C0R_accW      or E_now_C1R_accW      or not(E_now_C2R_accW) or E_now_CR_accW;
	m.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C1R_accW);
	m.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C2R_accW);
	m.addClause(c);
	c = not(E_now_C1R_accW) or not(E_now_C2R_accW);
	m.addClause(c);
	CNF::Var E_now_CR_mvW1;
	// neccessarly
	c = not(E_now_CR_mvW1)  or E_now_C0R_mvW1      or E_now_C1R_mvW1      or E_now_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_mvW1) or E_now_C1R_mvW1      or E_now_C2R_mvW1      or E_now_CR_mvW1;
	m.addClause(c);
	c = E_now_C0R_mvW1      or not(E_now_C1R_mvW1) or E_now_C2R_mvW1      or E_now_CR_mvW1;
	m.addClause(c);
	c = E_now_C0R_mvW1      or E_now_C1R_mvW1      or not(E_now_C2R_mvW1) or E_now_CR_mvW1;
	m.addClause(c);
	c = not(E_now_C0R_mvW1) or not(E_now_C1R_mvW1);
	m.addClause(c);
	c = not(E_now_C0R_mvW1) or not(E_now_C2R_mvW1);
	m.addClause(c);
	c = not(E_now_C1R_mvW1) or not(E_now_C2R_mvW1);
	m.addClause(c);
	CNF::Var E_now_CR_mvW0;
	// neccessarly
	c = not(E_now_CR_mvW0)  or E_now_C0R_mvW0      or E_now_C1R_mvW0      or E_now_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_mvW0) or E_now_C1R_mvW0      or E_now_C2R_mvW0      or E_now_CR_mvW0;
	m.addClause(c);
	c = E_now_C0R_mvW0      or not(E_now_C1R_mvW0) or E_now_C2R_mvW0      or E_now_CR_mvW0;
	m.addClause(c);
	c = E_now_C0R_mvW0      or E_now_C1R_mvW0      or not(E_now_C2R_mvW0) or E_now_CR_mvW0;
	m.addClause(c);
	c = not(E_now_C0R_mvW0) or not(E_now_C1R_mvW0);
	m.addClause(c);
	c = not(E_now_C0R_mvW0) or not(E_now_C2R_mvW0);
	m.addClause(c);
	c = not(E_now_C1R_mvW0) or not(E_now_C2R_mvW0);
	m.addClause(c);


	CNF::Var N_now_nobodyhome= var( G.north(v),    t,    NdStat::nobodyhome);
	CNF::Var N_now_R_ready   = var( G.north(v),    t,    NdStat::R_ready);
	CNF::Var N_now_R_accN    = var( G.north(v),    t,    R_Move::accN);
	CNF::Var N_now_R_mvN1    = var( G.north(v),    t,    R_Move::mvN1);
	CNF::Var N_now_R_mvN0    = var( G.north(v),    t,    R_Move::mvN0);
	CNF::Var N_now_R_accS    = var( G.north(v),    t,    R_Move::accS);
	CNF::Var N_now_R_mvS1    = var( G.north(v),    t,    R_Move::mvS1);
	CNF::Var N_now_R_mvS0    = var( G.north(v),    t,    R_Move::mvS0);
	CNF::Var N_now_C0R_ready = var( G.north(v),    t,    NdStat::C0R_ready);
	CNF::Var N_now_C0R_accN  = var( G.north(v),    t,    R_Move::w0_accN);
	CNF::Var N_now_C0R_mvN1  = var( G.north(v),    t,    R_Move::w0_mvN1);
	CNF::Var N_now_C0R_mvN2  = var( G.north(v),    t,    R_Move::w0_mvN2);
	CNF::Var N_now_C0R_mvN3  = var( G.north(v),    t,    R_Move::w0_mvN3);
	CNF::Var N_now_C0R_mvN0  = var( G.north(v),    t,    R_Move::w0_mvN0);
	CNF::Var N_now_C0R_accS  = var( G.north(v),    t,    R_Move::w0_accS);
	CNF::Var N_now_C0R_mvS1  = var( G.north(v),    t,    R_Move::w0_mvS1);
	CNF::Var N_now_C0R_mvS2  = var( G.north(v),    t,    R_Move::w0_mvS2);
	CNF::Var N_now_C0R_mvS3  = var( G.north(v),    t,    R_Move::w0_mvS3);
	CNF::Var N_now_C0R_mvS0  = var( G.north(v),    t,    R_Move::w0_mvS0);
	CNF::Var N_now_C1R_ready = var( G.north(v),    t,    NdStat::C1R_ready);
	CNF::Var N_now_C1R_accN  = var( G.north(v),    t,    R_Move::w1_accN);
	CNF::Var N_now_C1R_mvN1  = var( G.north(v),    t,    R_Move::w1_mvN1);
	CNF::Var N_now_C1R_mvN2  = var( G.north(v),    t,    R_Move::w1_mvN2);
	CNF::Var N_now_C1R_mvN3  = var( G.north(v),    t,    R_Move::w1_mvN3);
	CNF::Var N_now_C1R_mvN0  = var( G.north(v),    t,    R_Move::w1_mvN0);
	CNF::Var N_now_C1R_accS  = var( G.north(v),    t,    R_Move::w1_accS);
	CNF::Var N_now_C1R_mvS1  = var( G.north(v),    t,    R_Move::w1_mvS1);
	CNF::Var N_now_C1R_mvS2  = var( G.north(v),    t,    R_Move::w1_mvS2);
	CNF::Var N_now_C1R_mvS3  = var( G.north(v),    t,    R_Move::w1_mvS3);
	CNF::Var N_now_C1R_mvS0  = var( G.north(v),    t,    R_Move::w1_mvS0);
	CNF::Var N_now_C2R_ready = var( G.north(v),    t,    NdStat::C2R_ready);
	CNF::Var N_now_C2R_accN  = var( G.north(v),    t,    R_Move::w2_accN);
	CNF::Var N_now_C2R_mvN1  = var( G.north(v),    t,    R_Move::w2_mvN1);
	CNF::Var N_now_C2R_mvN2  = var( G.north(v),    t,    R_Move::w2_mvN2);
	CNF::Var N_now_C2R_mvN3  = var( G.north(v),    t,    R_Move::w2_mvN3);
	CNF::Var N_now_C2R_mvN0  = var( G.north(v),    t,    R_Move::w2_mvN0);
	CNF::Var N_now_C2R_accS  = var( G.north(v),    t,    R_Move::w2_accS);
	CNF::Var N_now_C2R_mvS1  = var( G.north(v),    t,    R_Move::w2_mvS1);
	CNF::Var N_now_C2R_mvS2  = var( G.north(v),    t,    R_Move::w2_mvS2);
	CNF::Var N_now_C2R_mvS3  = var( G.north(v),    t,    R_Move::w2_mvS3);
	CNF::Var N_now_C2R_mvS0  = var( G.north(v),    t,    R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var N_now_CR_accN;
	// neccessarly
	c = not(N_now_CR_accN)  or N_now_C0R_accN      or N_now_C1R_accN      or N_now_C2R_accN;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_accN) or N_now_C1R_accN      or N_now_C2R_accN      or N_now_CR_accN;
	m.addClause(c);
	c = N_now_C0R_accN      or not(N_now_C1R_accN) or N_now_C2R_accN      or N_now_CR_accN;
	m.addClause(c);
	c = N_now_C0R_accN      or N_now_C1R_accN      or not(N_now_C2R_accN) or N_now_CR_accN;
	m.addClause(c);
	c = not(N_now_C0R_accN) or not(N_now_C1R_accN);
	m.addClause(c);
	c = not(N_now_C0R_accN) or not(N_now_C2R_accN);
	m.addClause(c);
	c = not(N_now_C1R_accN) or not(N_now_C2R_accN);
	m.addClause(c);
	CNF::Var N_now_CR_mvN1;
	// neccessarly
	c = not(N_now_CR_mvN1)  or N_now_C0R_mvN1      or N_now_C1R_mvN1      or N_now_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvN1) or N_now_C1R_mvN1      or N_now_C2R_mvN1      or N_now_CR_mvN1;
	m.addClause(c);
	c = N_now_C0R_mvN1      or not(N_now_C1R_mvN1) or N_now_C2R_mvN1      or N_now_CR_mvN1;
	m.addClause(c);
	c = N_now_C0R_mvN1      or N_now_C1R_mvN1      or not(N_now_C2R_mvN1) or N_now_CR_mvN1;
	m.addClause(c);
	c = not(N_now_C0R_mvN1) or not(N_now_C1R_mvN1);
	m.addClause(c);
	c = not(N_now_C0R_mvN1) or not(N_now_C2R_mvN1);
	m.addClause(c);
	c = not(N_now_C1R_mvN1) or not(N_now_C2R_mvN1);
	m.addClause(c);
	CNF::Var N_now_CR_mvN2;
	// neccessarly
	c = not(N_now_CR_mvN2)  or N_now_C0R_mvN2      or N_now_C1R_mvN2      or N_now_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvN2) or N_now_C1R_mvN2      or N_now_C2R_mvN2      or N_now_CR_mvN2;
	m.addClause(c);
	c = N_now_C0R_mvN2      or not(N_now_C1R_mvN2) or N_now_C2R_mvN2      or N_now_CR_mvN2;
	m.addClause(c);
	c = N_now_C0R_mvN2      or N_now_C1R_mvN2      or not(N_now_C2R_mvN2) or N_now_CR_mvN2;
	m.addClause(c);
	c = not(N_now_C0R_mvN2) or not(N_now_C1R_mvN2);
	m.addClause(c);
	c = not(N_now_C0R_mvN2) or not(N_now_C2R_mvN2);
	m.addClause(c);
	c = not(N_now_C1R_mvN2) or not(N_now_C2R_mvN2);
	m.addClause(c);
	CNF::Var N_now_CR_mvN3;
	// neccessarly
	c = not(N_now_CR_mvN3)  or N_now_C0R_mvN3      or N_now_C1R_mvN3      or N_now_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvN3) or N_now_C1R_mvN3      or N_now_C2R_mvN3      or N_now_CR_mvN3;
	m.addClause(c);
	c = N_now_C0R_mvN3      or not(N_now_C1R_mvN3) or N_now_C2R_mvN3      or N_now_CR_mvN3;
	m.addClause(c);
	c = N_now_C0R_mvN3      or N_now_C1R_mvN3      or not(N_now_C2R_mvN3) or N_now_CR_mvN3;
	m.addClause(c);
	c = not(N_now_C0R_mvN3) or not(N_now_C1R_mvN3);
	m.addClause(c);
	c = not(N_now_C0R_mvN3) or not(N_now_C2R_mvN3);
	m.addClause(c);
	c = not(N_now_C1R_mvN3) or not(N_now_C2R_mvN3);
	m.addClause(c);
	CNF::Var N_now_CR_mvN0;
	// neccessarly
	c = not(N_now_CR_mvN0)  or N_now_C0R_mvN0      or N_now_C1R_mvN0      or N_now_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvN0) or N_now_C1R_mvN0      or N_now_C2R_mvN0      or N_now_CR_mvN0;
	m.addClause(c);
	c = N_now_C0R_mvN0      or not(N_now_C1R_mvN0) or N_now_C2R_mvN0      or N_now_CR_mvN0;
	m.addClause(c);
	c = N_now_C0R_mvN0      or N_now_C1R_mvN0      or not(N_now_C2R_mvN0) or N_now_CR_mvN0;
	m.addClause(c);
	c = not(N_now_C0R_mvN0) or not(N_now_C1R_mvN0);
	m.addClause(c);
	c = not(N_now_C0R_mvN0) or not(N_now_C2R_mvN0);
	m.addClause(c);
	c = not(N_now_C1R_mvN0) or not(N_now_C2R_mvN0);
	m.addClause(c);

	CNF::Var N_now_CR_accS;
	// neccessarly
	c = not(N_now_CR_accS)  or N_now_C0R_accS      or N_now_C1R_accS      or N_now_C2R_accS;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_accS) or N_now_C1R_accS      or N_now_C2R_accS      or N_now_CR_accS;
	m.addClause(c);
	c = N_now_C0R_accS      or not(N_now_C1R_accS) or N_now_C2R_accS      or N_now_CR_accS;
	m.addClause(c);
	c = N_now_C0R_accS      or N_now_C1R_accS      or not(N_now_C2R_accS) or N_now_CR_accS;
	m.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C1R_accS);
	m.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C2R_accS);
	m.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C2R_accS);
	m.addClause(c);
	CNF::Var N_now_CR_mvS1;
	// neccessarly
	c = not(N_now_CR_mvS1)  or N_now_C0R_mvS1      or N_now_C1R_mvS1      or N_now_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvS1) or N_now_C1R_mvS1      or N_now_C2R_mvS1      or N_now_CR_mvS1;
	m.addClause(c);
	c = N_now_C0R_mvS1      or not(N_now_C1R_mvS1) or N_now_C2R_mvS1      or N_now_CR_mvS1;
	m.addClause(c);
	c = N_now_C0R_mvS1      or N_now_C1R_mvS1      or not(N_now_C2R_mvS1) or N_now_CR_mvS1;
	m.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C1R_mvS1);
	m.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C2R_mvS1);
	m.addClause(c);
	c = not(N_now_C1R_mvS1) or not(N_now_C2R_mvS1);
	m.addClause(c);
	CNF::Var N_now_CR_mvS2;
	// neccessarly
	c = not(N_now_CR_mvS2)  or N_now_C0R_mvS2      or N_now_C1R_mvS2      or N_now_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvS2) or N_now_C1R_mvS2      or N_now_C2R_mvS2      or N_now_CR_mvS2;
	m.addClause(c);
	c = N_now_C0R_mvS2      or not(N_now_C1R_mvS2) or N_now_C2R_mvS2      or N_now_CR_mvS2;
	m.addClause(c);
	c = N_now_C0R_mvS2      or N_now_C1R_mvS2      or not(N_now_C2R_mvS2) or N_now_CR_mvS2;
	m.addClause(c);
	c = not(N_now_C0R_mvS2) or not(N_now_C1R_mvS2);
	m.addClause(c);
	c = not(N_now_C0R_mvS2) or not(N_now_C2R_mvS2);
	m.addClause(c);
	c = not(N_now_C1R_mvS2) or not(N_now_C2R_mvS2);
	m.addClause(c);
	CNF::Var N_now_CR_mvS3;
	// neccessarly
	c = not(N_now_CR_mvS3)  or N_now_C0R_mvS3      or N_now_C1R_mvS3      or N_now_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvS3) or N_now_C1R_mvS3      or N_now_C2R_mvS3      or N_now_CR_mvS3;
	m.addClause(c);
	c = N_now_C0R_mvS3      or not(N_now_C1R_mvS3) or N_now_C2R_mvS3      or N_now_CR_mvS3;
	m.addClause(c);
	c = N_now_C0R_mvS3      or N_now_C1R_mvS3      or not(N_now_C2R_mvS3) or N_now_CR_mvS3;
	m.addClause(c);
	c = not(N_now_C0R_mvS3) or not(N_now_C1R_mvS3);
	m.addClause(c);
	c = not(N_now_C0R_mvS3) or not(N_now_C2R_mvS3);
	m.addClause(c);
	c = not(N_now_C1R_mvS3) or not(N_now_C2R_mvS3);
	m.addClause(c);
	CNF::Var N_now_CR_mvS0;
	// neccessarly
	c = not(N_now_CR_mvS0)  or N_now_C0R_mvS0      or N_now_C1R_mvS0      or N_now_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvS0) or N_now_C1R_mvS0      or N_now_C2R_mvS0      or N_now_CR_mvS0;
	m.addClause(c);
	c = N_now_C0R_mvS0      or not(N_now_C1R_mvS0) or N_now_C2R_mvS0      or N_now_CR_mvS0;
	m.addClause(c);
	c = N_now_C0R_mvS0      or N_now_C1R_mvS0      or not(N_now_C2R_mvS0) or N_now_CR_mvS0;
	m.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C1R_mvS0);
	m.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C2R_mvS0);
	m.addClause(c);
	c = not(N_now_C1R_mvS0) or not(N_now_C2R_mvS0);
	m.addClause(c);


	CNF::Var W_now_nobodyhome= var( G.west(v),    t,    NdStat::nobodyhome);
	CNF::Var W_now_R_ready   = var( G.west(v),    t,    NdStat::R_ready);
	CNF::Var W_now_R_accE    = var( G.west(v),    t,    R_Move::accE);
	CNF::Var W_now_R_mvE0    = var( G.west(v),    t,    R_Move::mvE0);
	CNF::Var W_now_R_accW    = var( G.west(v),    t,    R_Move::accW);
	CNF::Var W_now_R_mvW0    = var( G.west(v),    t,    R_Move::mvW0);
	CNF::Var W_now_C0R_ready = var( G.west(v),    t,    NdStat::C0R_ready);
	CNF::Var W_now_C0R_accE  = var( G.west(v),    t,    R_Move::w0_accE);
	CNF::Var W_now_C0R_mvE1  = var( G.west(v),    t,    R_Move::w0_mvE1);
	CNF::Var W_now_C0R_mvE0  = var( G.west(v),    t,    R_Move::w0_mvE0);
	CNF::Var W_now_C0R_accW  = var( G.west(v),    t,    R_Move::w0_accW);
	CNF::Var W_now_C0R_mvW1  = var( G.west(v),    t,    R_Move::w0_mvW1);
	CNF::Var W_now_C0R_mvW0  = var( G.west(v),    t,    R_Move::w0_mvW0);
	CNF::Var W_now_C1R_ready = var( G.west(v),    t,    NdStat::C1R_ready);
	CNF::Var W_now_C1R_accE  = var( G.west(v),    t,    R_Move::w1_accE);
	CNF::Var W_now_C1R_mvE1  = var( G.west(v),    t,    R_Move::w1_mvE1);
	CNF::Var W_now_C1R_mvE0  = var( G.west(v),    t,    R_Move::w1_mvE0);
	CNF::Var W_now_C1R_accW  = var( G.west(v),    t,    R_Move::w1_accW);
	CNF::Var W_now_C1R_mvW1  = var( G.west(v),    t,    R_Move::w1_mvW1);
	CNF::Var W_now_C1R_mvW0  = var( G.west(v),    t,    R_Move::w1_mvW0);
	CNF::Var W_now_C2R_ready = var( G.west(v),    t,    NdStat::C2R_ready);
	CNF::Var W_now_C2R_accE  = var( G.west(v),    t,    R_Move::w2_accE);
	CNF::Var W_now_C2R_mvE1  = var( G.west(v),    t,    R_Move::w2_mvE1);
	CNF::Var W_now_C2R_mvE0  = var( G.west(v),    t,    R_Move::w2_mvE0);
	CNF::Var W_now_C2R_accW  = var( G.west(v),    t,    R_Move::w2_accW);
	CNF::Var W_now_C2R_mvW1  = var( G.west(v),    t,    R_Move::w2_mvW1);
	CNF::Var W_now_C2R_mvW0  = var( G.west(v),    t,    R_Move::w2_mvW0);

        // Abbreviations
	CNF::Var W_now_CR_accE;
	// neccessarly
	c = not(W_now_CR_accE)  or W_now_C0R_accE      or W_now_C1R_accE      or W_now_C2R_accE;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_accE) or W_now_C1R_accE      or W_now_C2R_accE      or W_now_CR_accE;
	m.addClause(c);
	c = W_now_C0R_accE      or not(W_now_C1R_accE) or W_now_C2R_accE      or W_now_CR_accE;
	m.addClause(c);
	c = W_now_C0R_accE      or W_now_C1R_accE      or not(W_now_C2R_accE) or W_now_CR_accE;
	m.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C1R_accE);
	m.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C2R_accE);
	m.addClause(c);
	c = not(W_now_C1R_accE) or not(W_now_C2R_accE);
	m.addClause(c);
	CNF::Var W_now_CR_mvE1;
	// neccessarly
	c = not(W_now_CR_mvE1)  or W_now_C0R_mvE1      or W_now_C1R_mvE1      or W_now_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_mvE1) or W_now_C1R_mvE1      or W_now_C2R_mvE1      or W_now_CR_mvE1;
	m.addClause(c);
	c = W_now_C0R_mvE1      or not(W_now_C1R_mvE1) or W_now_C2R_mvE1      or W_now_CR_mvE1;
	m.addClause(c);
	c = W_now_C0R_mvE1      or W_now_C1R_mvE1      or not(W_now_C2R_mvE1) or W_now_CR_mvE1;
	m.addClause(c);
	c = not(W_now_C0R_mvE1) or not(W_now_C1R_mvE1);
	m.addClause(c);
	c = not(W_now_C0R_mvE1) or not(W_now_C2R_mvE1);
	m.addClause(c);
	c = not(W_now_C1R_mvE1) or not(W_now_C2R_mvE1);
	m.addClause(c);
	CNF::Var W_now_CR_mvE0;
	// neccessarly
	c = not(W_now_CR_mvE0)  or W_now_C0R_mvE0      or W_now_C1R_mvE0      or W_now_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_mvE0) or W_now_C1R_mvE0      or W_now_C2R_mvE0      or W_now_CR_mvE0;
	m.addClause(c);
	c = W_now_C0R_mvE0      or not(W_now_C1R_mvE0) or W_now_C2R_mvE0      or W_now_CR_mvE0;
	m.addClause(c);
	c = W_now_C0R_mvE0      or W_now_C1R_mvE0      or not(W_now_C2R_mvE0) or W_now_CR_mvE0;
	m.addClause(c);
	c = not(W_now_C0R_mvE0) or not(W_now_C1R_mvE0);
	m.addClause(c);
	c = not(W_now_C0R_mvE0) or not(W_now_C2R_mvE0);
	m.addClause(c);
	c = not(W_now_C1R_mvE0) or not(W_now_C2R_mvE0);
	m.addClause(c);

	CNF::Var W_now_CR_accW;
	// neccessarly
	c = not(W_now_CR_accW)  or W_now_C0R_accW      or W_now_C1R_accW      or W_now_C2R_accW;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_accW) or W_now_C1R_accW      or W_now_C2R_accW      or W_now_CR_accW;
	m.addClause(c);
	c = W_now_C0R_accW      or not(W_now_C1R_accW) or W_now_C2R_accW      or W_now_CR_accW;
	m.addClause(c);
	c = W_now_C0R_accW      or W_now_C1R_accW      or not(W_now_C2R_accW) or W_now_CR_accW;
	m.addClause(c);
	c = not(W_now_C0R_accW) or not(W_now_C1R_accW);
	m.addClause(c);
	c = not(W_now_C0R_accW) or not(W_now_C2R_accW);
	m.addClause(c);
	c = not(W_now_C1R_accW) or not(W_now_C2R_accW);
	m.addClause(c);
	CNF::Var W_now_CR_mvW1;
	// neccessarly
	c = not(W_now_CR_mvW1)  or W_now_C0R_mvW1      or W_now_C1R_mvW1      or W_now_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_mvW1) or W_now_C1R_mvW1      or W_now_C2R_mvW1      or W_now_CR_mvW1;
	m.addClause(c);
	c = W_now_C0R_mvW1      or not(W_now_C1R_mvW1) or W_now_C2R_mvW1      or W_now_CR_mvW1;
	m.addClause(c);
	c = W_now_C0R_mvW1      or W_now_C1R_mvW1      or not(W_now_C2R_mvW1) or W_now_CR_mvW1;
	m.addClause(c);
	c = not(W_now_C0R_mvW1) or not(W_now_C1R_mvW1);
	m.addClause(c);
	c = not(W_now_C0R_mvW1) or not(W_now_C2R_mvW1);
	m.addClause(c);
	c = not(W_now_C1R_mvW1) or not(W_now_C2R_mvW1);
	m.addClause(c);
	CNF::Var W_now_CR_mvW0;
	// neccessarly
	c = not(W_now_CR_mvW0)  or W_now_C0R_mvW0      or W_now_C1R_mvW0      or W_now_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_mvW0) or W_now_C1R_mvW0      or W_now_C2R_mvW0      or W_now_CR_mvW0;
	m.addClause(c);
	c = W_now_C0R_mvW0      or not(W_now_C1R_mvW0) or W_now_C2R_mvW0      or W_now_CR_mvW0;
	m.addClause(c);
	c = W_now_C0R_mvW0      or W_now_C1R_mvW0      or not(W_now_C2R_mvW0) or W_now_CR_mvW0;
	m.addClause(c);
	c = not(W_now_C0R_mvW0) or not(W_now_C1R_mvW0);
	m.addClause(c);
	c = not(W_now_C0R_mvW0) or not(W_now_C2R_mvW0);
	m.addClause(c);
	c = not(W_now_C1R_mvW0) or not(W_now_C2R_mvW0);
	m.addClause(c);


	CNF::Var S_now_nobodyhome= var( G.south(v),    t,    NdStat::nobodyhome);
	CNF::Var S_now_R_ready   = var( G.south(v),    t,    NdStat::R_ready);
	CNF::Var S_now_R_accN    = var( G.south(v),    t,    R_Move::accN);
	CNF::Var S_now_R_mvN1    = var( G.south(v),    t,    R_Move::mvN1);
	CNF::Var S_now_R_mvN0    = var( G.south(v),    t,    R_Move::mvN0);
	CNF::Var S_now_R_accS    = var( G.south(v),    t,    R_Move::accS);
	CNF::Var S_now_R_mvS1    = var( G.south(v),    t,    R_Move::mvS1);
	CNF::Var S_now_R_mvS0    = var( G.south(v),    t,    R_Move::mvS0);
	CNF::Var S_now_C0R_ready = var( G.south(v),    t,    NdStat::C0R_ready);
	CNF::Var S_now_C0R_accN  = var( G.south(v),    t,    R_Move::w0_accN);
	CNF::Var S_now_C0R_mvN1  = var( G.south(v),    t,    R_Move::w0_mvN1);
	CNF::Var S_now_C0R_mvN2  = var( G.south(v),    t,    R_Move::w0_mvN2);
	CNF::Var S_now_C0R_mvN3  = var( G.south(v),    t,    R_Move::w0_mvN3);
	CNF::Var S_now_C0R_mvN0  = var( G.south(v),    t,    R_Move::w0_mvN0);
	CNF::Var S_now_C0R_accS  = var( G.south(v),    t,    R_Move::w0_accS);
	CNF::Var S_now_C0R_mvS1  = var( G.south(v),    t,    R_Move::w0_mvS1);
	CNF::Var S_now_C0R_mvS2  = var( G.south(v),    t,    R_Move::w0_mvS2);
	CNF::Var S_now_C0R_mvS3  = var( G.south(v),    t,    R_Move::w0_mvS3);
	CNF::Var S_now_C0R_mvS0  = var( G.south(v),    t,    R_Move::w0_mvS0);
	CNF::Var S_now_C1R_ready = var( G.south(v),    t,    NdStat::C1R_ready);
	CNF::Var S_now_C1R_accN  = var( G.south(v),    t,    R_Move::w1_accN);
	CNF::Var S_now_C1R_mvN1  = var( G.south(v),    t,    R_Move::w1_mvN1);
	CNF::Var S_now_C1R_mvN2  = var( G.south(v),    t,    R_Move::w1_mvN2);
	CNF::Var S_now_C1R_mvN3  = var( G.south(v),    t,    R_Move::w1_mvN3);
	CNF::Var S_now_C1R_mvN0  = var( G.south(v),    t,    R_Move::w1_mvN0);
	CNF::Var S_now_C1R_accS  = var( G.south(v),    t,    R_Move::w1_accS);
	CNF::Var S_now_C1R_mvS1  = var( G.south(v),    t,    R_Move::w1_mvS1);
	CNF::Var S_now_C1R_mvS2  = var( G.south(v),    t,    R_Move::w1_mvS2);
	CNF::Var S_now_C1R_mvS3  = var( G.south(v),    t,    R_Move::w1_mvS3);
	CNF::Var S_now_C1R_mvS0  = var( G.south(v),    t,    R_Move::w1_mvS0);
	CNF::Var S_now_C2R_ready = var( G.south(v),    t,    NdStat::C2R_ready);
	CNF::Var S_now_C2R_accN  = var( G.south(v),    t,    R_Move::w2_accN);
	CNF::Var S_now_C2R_mvN1  = var( G.south(v),    t,    R_Move::w2_mvN1);
	CNF::Var S_now_C2R_mvN2  = var( G.south(v),    t,    R_Move::w2_mvN2);
	CNF::Var S_now_C2R_mvN3  = var( G.south(v),    t,    R_Move::w2_mvN3);
	CNF::Var S_now_C2R_mvN0  = var( G.south(v),    t,    R_Move::w2_mvN0);
	CNF::Var S_now_C2R_accS  = var( G.south(v),    t,    R_Move::w2_accS);
	CNF::Var S_now_C2R_mvS1  = var( G.south(v),    t,    R_Move::w2_mvS1);
	CNF::Var S_now_C2R_mvS2  = var( G.south(v),    t,    R_Move::w2_mvS2);
	CNF::Var S_now_C2R_mvS3  = var( G.south(v),    t,    R_Move::w2_mvS3);
	CNF::Var S_now_C2R_mvS0  = var( G.south(v),    t,    R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var S_now_CR_accN;
	// neccessarly
	c = not(S_now_CR_accN)  or S_now_C0R_accN      or S_now_C1R_accN      or S_now_C2R_accN;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_accN) or S_now_C1R_accN      or S_now_C2R_accN      or S_now_CR_accN;
	m.addClause(c);
	c = S_now_C0R_accN      or not(S_now_C1R_accN) or S_now_C2R_accN      or S_now_CR_accN;
	m.addClause(c);
	c = S_now_C0R_accN      or S_now_C1R_accN      or not(S_now_C2R_accN) or S_now_CR_accN;
	m.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C1R_accN);
	m.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C2R_accN);
	m.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C2R_accN);
	m.addClause(c);
	CNF::Var S_now_CR_mvN1;
	// neccessarly
	c = not(S_now_CR_mvN1)  or S_now_C0R_mvN1      or S_now_C1R_mvN1      or S_now_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvN1) or S_now_C1R_mvN1      or S_now_C2R_mvN1      or S_now_CR_mvN1;
	m.addClause(c);
	c = S_now_C0R_mvN1      or not(S_now_C1R_mvN1) or S_now_C2R_mvN1      or S_now_CR_mvN1;
	m.addClause(c);
	c = S_now_C0R_mvN1      or S_now_C1R_mvN1      or not(S_now_C2R_mvN1) or S_now_CR_mvN1;
	m.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C1R_mvN1);
	m.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C2R_mvN1);
	m.addClause(c);
	c = not(S_now_C1R_mvN1) or not(S_now_C2R_mvN1);
	m.addClause(c);
	CNF::Var S_now_CR_mvN2;
	// neccessarly
	c = not(S_now_CR_mvN2)  or S_now_C0R_mvN2      or S_now_C1R_mvN2      or S_now_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvN2) or S_now_C1R_mvN2      or S_now_C2R_mvN2      or S_now_CR_mvN2;
	m.addClause(c);
	c = S_now_C0R_mvN2      or not(S_now_C1R_mvN2) or S_now_C2R_mvN2      or S_now_CR_mvN2;
	m.addClause(c);
	c = S_now_C0R_mvN2      or S_now_C1R_mvN2      or not(S_now_C2R_mvN2) or S_now_CR_mvN2;
	m.addClause(c);
	c = not(S_now_C0R_mvN2) or not(S_now_C1R_mvN2);
	m.addClause(c);
	c = not(S_now_C0R_mvN2) or not(S_now_C2R_mvN2);
	m.addClause(c);
	c = not(S_now_C1R_mvN2) or not(S_now_C2R_mvN2);
	m.addClause(c);
	CNF::Var S_now_CR_mvN3;
	// neccessarly
	c = not(S_now_CR_mvN3)  or S_now_C0R_mvN3      or S_now_C1R_mvN3      or S_now_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvN3) or S_now_C1R_mvN3      or S_now_C2R_mvN3      or S_now_CR_mvN3;
	m.addClause(c);
	c = S_now_C0R_mvN3      or not(S_now_C1R_mvN3) or S_now_C2R_mvN3      or S_now_CR_mvN3;
	m.addClause(c);
	c = S_now_C0R_mvN3      or S_now_C1R_mvN3      or not(S_now_C2R_mvN3) or S_now_CR_mvN3;
	m.addClause(c);
	c = not(S_now_C0R_mvN3) or not(S_now_C1R_mvN3);
	m.addClause(c);
	c = not(S_now_C0R_mvN3) or not(S_now_C2R_mvN3);
	m.addClause(c);
	c = not(S_now_C1R_mvN3) or not(S_now_C2R_mvN3);
	m.addClause(c);
	CNF::Var S_now_CR_mvN0;
	// neccessarly
	c = not(S_now_CR_mvN0)  or S_now_C0R_mvN0      or S_now_C1R_mvN0      or S_now_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvN0) or S_now_C1R_mvN0      or S_now_C2R_mvN0      or S_now_CR_mvN0;
	m.addClause(c);
	c = S_now_C0R_mvN0      or not(S_now_C1R_mvN0) or S_now_C2R_mvN0      or S_now_CR_mvN0;
	m.addClause(c);
	c = S_now_C0R_mvN0      or S_now_C1R_mvN0      or not(S_now_C2R_mvN0) or S_now_CR_mvN0;
	m.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C1R_mvN0);
	m.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C2R_mvN0);
	m.addClause(c);
	c = not(S_now_C1R_mvN0) or not(S_now_C2R_mvN0);
	m.addClause(c);

	CNF::Var S_now_CR_accS;
	// neccessarly
	c = not(S_now_CR_accS)  or S_now_C0R_accS      or S_now_C1R_accS      or S_now_C2R_accS;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_accS) or S_now_C1R_accS      or S_now_C2R_accS      or S_now_CR_accS;
	m.addClause(c);
	c = S_now_C0R_accS      or not(S_now_C1R_accS) or S_now_C2R_accS      or S_now_CR_accS;
	m.addClause(c);
	c = S_now_C0R_accS      or S_now_C1R_accS      or not(S_now_C2R_accS) or S_now_CR_accS;
	m.addClause(c);
	c = not(S_now_C0R_accS) or not(S_now_C1R_accS);
	m.addClause(c);
	c = not(S_now_C0R_accS) or not(S_now_C2R_accS);
	m.addClause(c);
	c = not(S_now_C1R_accS) or not(S_now_C2R_accS);
	m.addClause(c);
	CNF::Var S_now_CR_mvS1;
	// neccessarly
	c = not(S_now_CR_mvS1)  or S_now_C0R_mvS1      or S_now_C1R_mvS1      or S_now_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvS1) or S_now_C1R_mvS1      or S_now_C2R_mvS1      or S_now_CR_mvS1;
	m.addClause(c);
	c = S_now_C0R_mvS1      or not(S_now_C1R_mvS1) or S_now_C2R_mvS1      or S_now_CR_mvS1;
	m.addClause(c);
	c = S_now_C0R_mvS1      or S_now_C1R_mvS1      or not(S_now_C2R_mvS1) or S_now_CR_mvS1;
	m.addClause(c);
	c = not(S_now_C0R_mvS1) or not(S_now_C1R_mvS1);
	m.addClause(c);
	c = not(S_now_C0R_mvS1) or not(S_now_C2R_mvS1);
	m.addClause(c);
	c = not(S_now_C1R_mvS1) or not(S_now_C2R_mvS1);
	m.addClause(c);
	CNF::Var S_now_CR_mvS2;
	// neccessarly
	c = not(S_now_CR_mvS2)  or S_now_C0R_mvS2      or S_now_C1R_mvS2      or S_now_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvS2) or S_now_C1R_mvS2      or S_now_C2R_mvS2      or S_now_CR_mvS2;
	m.addClause(c);
	c = S_now_C0R_mvS2      or not(S_now_C1R_mvS2) or S_now_C2R_mvS2      or S_now_CR_mvS2;
	m.addClause(c);
	c = S_now_C0R_mvS2      or S_now_C1R_mvS2      or not(S_now_C2R_mvS2) or S_now_CR_mvS2;
	m.addClause(c);
	c = not(S_now_C0R_mvS2) or not(S_now_C1R_mvS2);
	m.addClause(c);
	c = not(S_now_C0R_mvS2) or not(S_now_C2R_mvS2);
	m.addClause(c);
	c = not(S_now_C1R_mvS2) or not(S_now_C2R_mvS2);
	m.addClause(c);
	CNF::Var S_now_CR_mvS3;
	// neccessarly
	c = not(S_now_CR_mvS3)  or S_now_C0R_mvS3      or S_now_C1R_mvS3      or S_now_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvS3) or S_now_C1R_mvS3      or S_now_C2R_mvS3      or S_now_CR_mvS3;
	m.addClause(c);
	c = S_now_C0R_mvS3      or not(S_now_C1R_mvS3) or S_now_C2R_mvS3      or S_now_CR_mvS3;
	m.addClause(c);
	c = S_now_C0R_mvS3      or S_now_C1R_mvS3      or not(S_now_C2R_mvS3) or S_now_CR_mvS3;
	m.addClause(c);
	c = not(S_now_C0R_mvS3) or not(S_now_C1R_mvS3);
	m.addClause(c);
	c = not(S_now_C0R_mvS3) or not(S_now_C2R_mvS3);
	m.addClause(c);
	c = not(S_now_C1R_mvS3) or not(S_now_C2R_mvS3);
	m.addClause(c);
	CNF::Var S_now_CR_mvS0;
	// neccessarly
	c = not(S_now_CR_mvS0)  or S_now_C0R_mvS0      or S_now_C1R_mvS0      or S_now_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvS0) or S_now_C1R_mvS0      or S_now_C2R_mvS0      or S_now_CR_mvS0;
	m.addClause(c);
	c = S_now_C0R_mvS0      or not(S_now_C1R_mvS0) or S_now_C2R_mvS0      or S_now_CR_mvS0;
	m.addClause(c);
	c = S_now_C0R_mvS0      or S_now_C1R_mvS0      or not(S_now_C2R_mvS0) or S_now_CR_mvS0;
	m.addClause(c);
	c = not(S_now_C0R_mvS0) or not(S_now_C1R_mvS0);
	m.addClause(c);
	c = not(S_now_C0R_mvS0) or not(S_now_C2R_mvS0);
	m.addClause(c);
	c = not(S_now_C1R_mvS0) or not(S_now_C2R_mvS0);
	m.addClause(c);



        // At most one robot moving towards this node:
	c = E_now_R_accW  or E_now_R_mvW0
	or  W_now_R_accE  or W_now_R_mvE0
	or  N_now_R_accS  or N_now_R_mvS0  or N_now_R_mvS1
	or  S_now_R_accN  or S_now_R_mvN0  or S_now_R_mvN1
        or  E_now_CR_accW or E_now_CR_mvW0 or E_now_CR_mvW1
	or  W_now_CR_accE or W_now_CR_mvE0 or W_now_CR_mvE1
	or  N_now_CR_accS or N_now_CR_mvS0 or N_now_CR_mvS1 or N_now_CR_mvS2 or N_now_CR_mvS3
	or  S_now_CR_accN or S_now_CR_mvN0 or S_now_CR_mvN1 or S_now_CR_mvN2 or S_now_CR_mvN3;
	m.addClause(c);
	// 26 choose 2
	c = not(E_now_R_accW) or not(E_now_R_mvW0);
	m.addClause(c);
	c = not(E_now_R_accW) or not(W_now_R_accE);
	m.addClause(c);
	c = not(E_now_R_accW) or not(W_now_R_mvE0);
	m.addClause(c);
	c = not(E_now_R_accW) or not(N_now_R_accS);
	m.addClause(c);
	c = not(E_now_R_accW) or not(N_now_R_mvS0);
	m.addClause(c);
	c = not(E_now_R_accW) or not(N_now_R_mvS1);
	m.addClause(c);
	c = not(E_now_R_accW) or not(S_now_R_accN);
	m.addClause(c);
	c = not(E_now_R_accW) or not(S_now_R_mvN0);
	m.addClause(c);
	c = not(E_now_R_accW) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(E_now_R_accW) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(E_now_R_accW) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(E_now_R_accW) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(E_now_R_accW) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(E_now_R_accW) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(E_now_R_accW) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(E_now_R_accW) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(E_now_R_accW) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(E_now_R_accW) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(E_now_R_accW) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(E_now_R_accW) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(E_now_R_accW) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(E_now_R_accW) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(E_now_R_accW) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(E_now_R_accW) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(E_now_R_accW) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_R_accE);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_R_mvE0);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_R_accS);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_R_mvS0);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_R_mvS1);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_R_accN);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_R_mvN0);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(E_now_R_mvW0) or not(S_now_CR_mvN3);
	m.addClause(c);

	c = not(W_now_R_accE) or not(W_now_R_mvE0);
	m.addClause(c);
	c = not(W_now_R_accE) or not(N_now_R_accS);
	m.addClause(c);
	c = not(W_now_R_accE) or not(N_now_R_mvS0);
	m.addClause(c);
	c = not(W_now_R_accE) or not(N_now_R_mvS1);
	m.addClause(c);
	c = not(W_now_R_accE) or not(S_now_R_accN);
	m.addClause(c);
	c = not(W_now_R_accE) or not(S_now_R_mvN0);
	m.addClause(c);
	c = not(W_now_R_accE) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(W_now_R_accE) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(W_now_R_accE) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(W_now_R_accE) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(W_now_R_accE) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(W_now_R_accE) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(W_now_R_accE) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(W_now_R_accE) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(W_now_R_accE) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(W_now_R_accE) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(W_now_R_accE) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(W_now_R_accE) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(W_now_R_accE) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(W_now_R_accE) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(W_now_R_accE) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(W_now_R_accE) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(W_now_R_accE) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_R_accS);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_R_mvS0);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_R_mvS1);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_R_accN);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_R_mvN0);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(W_now_R_mvE0) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(N_now_R_accS) or not(N_now_R_mvS0);
	m.addClause(c);
	c = not(N_now_R_accS) or not(N_now_R_mvS1);
	m.addClause(c);
	c = not(N_now_R_accS) or not(S_now_R_accN);
	m.addClause(c);
	c = not(N_now_R_accS) or not(S_now_R_mvN0);
	m.addClause(c);
	c = not(N_now_R_accS) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(N_now_R_accS) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(N_now_R_accS) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(N_now_R_accS) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(N_now_R_accS) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(N_now_R_accS) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(N_now_R_accS) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(N_now_R_accS) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(N_now_R_accS) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(N_now_R_accS) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(N_now_R_accS) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(N_now_R_accS) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(N_now_R_accS) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(N_now_R_accS) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(N_now_R_accS) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(N_now_R_accS) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(N_now_R_accS) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_R_mvS1);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_R_accN);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_R_mvN0);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(N_now_R_mvS0) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_R_accN);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_R_mvN0);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(N_now_R_mvS1) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(S_now_R_accN) or not(S_now_R_mvN0);
	m.addClause(c);
	c = not(S_now_R_accN) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(S_now_R_accN) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(S_now_R_accN) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(S_now_R_accN) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(S_now_R_accN) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(S_now_R_accN) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(S_now_R_accN) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(S_now_R_accN) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(S_now_R_accN) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(S_now_R_accN) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(S_now_R_accN) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(S_now_R_accN) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(S_now_R_accN) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(S_now_R_accN) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(S_now_R_accN) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(S_now_R_accN) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(S_now_R_accN) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_R_mvN1);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(S_now_R_mvN0) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_CR_accW);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(S_now_R_mvN1) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(E_now_CR_mvW0);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(E_now_CR_accW) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(E_now_CR_mvW1);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(E_now_CR_mvW0) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(W_now_CR_accE);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(E_now_CR_mvW1) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(W_now_CR_mvE0);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(W_now_CR_accE) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(W_now_CR_mvE1);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(W_now_CR_mvE0) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(N_now_CR_accS);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(W_now_CR_mvE1) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(N_now_CR_mvS0);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(N_now_CR_accS) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(N_now_CR_mvS0) or not(N_now_CR_mvS1);
	m.addClause(c);
	c = not(N_now_CR_mvS0) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(N_now_CR_mvS0) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(N_now_CR_mvS0) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(N_now_CR_mvS0) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(N_now_CR_mvS0) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(N_now_CR_mvS0) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(N_now_CR_mvS0) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(N_now_CR_mvS1) or not(N_now_CR_mvS2);
	m.addClause(c);
	c = not(N_now_CR_mvS1) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(N_now_CR_mvS1) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(N_now_CR_mvS1) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(N_now_CR_mvS1) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(N_now_CR_mvS1) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(N_now_CR_mvS1) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(N_now_CR_mvS2) or not(N_now_CR_mvS3);
	m.addClause(c);
	c = not(N_now_CR_mvS2) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(N_now_CR_mvS2) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(N_now_CR_mvS2) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(N_now_CR_mvS2) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(N_now_CR_mvS2) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(N_now_CR_mvS3) or not(S_now_CR_accN);
	m.addClause(c);
	c = not(N_now_CR_mvS3) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(N_now_CR_mvS3) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(N_now_CR_mvS3) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(N_now_CR_mvS3) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(S_now_CR_accN) or not(S_now_CR_mvN0);
	m.addClause(c);
	c = not(S_now_CR_accN) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(S_now_CR_accN) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(S_now_CR_accN) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(S_now_CR_mvN0) or not(S_now_CR_mvN1);
	m.addClause(c);
	c = not(S_now_CR_mvN0) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(S_now_CR_mvN0) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(S_now_CR_mvN1) or not(S_now_CR_mvN2);
	m.addClause(c);
	c = not(S_now_CR_mvN1) or not(S_now_CR_mvN3);
	m.addClause(c);
	c = not(S_now_CR_mvN2) or not(S_now_CR_mvN3);
	m.addClause(c);

	c = not(Here_now_R_mvN0)  or N_now_nobodyhome or N_now_R_mvN0  or N_now_R_accN  or N_now_R_mvN1                 	                                                              or N_now_CR_mvN2 or N_now_CR_mvN3;
	m.addClause(c);
	c = not(N_now_R_accN)     or N_now_nobodyhome                  or N_now_R_accN  or N_now_R_mvN1                  	                                                              or N_now_CR_mvN2 or N_now_CR_mvN3;
	m.addClause(c);
	c = not(Here_now_R_mvN1)  or N_now_nobodyhome                                   or N_now_R_mvN1                 	                                                                               or N_now_CR_mvN3;
	m.addClause(c);
	c = not(Here_now_R_mvN0)  or S_now_nobodyhome or S_now_R_mvN0                                   ;
	m.addClause(c);
	c = not(Here_now_R_mvS0)  or S_now_nobodyhome or S_now_R_mvS0  or S_now_R_accS  or S_now_R_mvS1                 	                                                              or S_now_CR_mvS2 or S_now_CR_mvS3;
	m.addClause(c);
	c = not(Here_now_R_accS)  or S_now_nobodyhome                  or S_now_R_accS  or S_now_R_mvS1                 	                                                              or S_now_CR_mvS2 or S_now_CR_mvS3;
	m.addClause(c);
	c = not(Here_now_R_mvS1)  or S_now_nobodyhome                                   or S_now_R_mvS1                 	                                                                               or S_now_CR_mvS3;
	m.addClause(c);
	c = not(Here_now_R_mvS0)  or S_now_nobodyhome or N_now_R_mvS0                                   ;
	m.addClause(c);
	c = not(Here_now_R_mvE0)  or E_now_nobodyhome or E_now_R_mvE0  or E_now_R_accE                                  	                                                                               or E_now_CR_mvE1;
	m.addClause(c);
	c = not(Here_now_R_accE)  or E_now_nobodyhome                  or E_now_R_accE                                    	                                                                               or E_now_CR_mvE1;
	m.addClause(C);
	c = not(Here_now_R_mvE0)  or W_now_nobodyhome or W_now_R_mvE0                                   ;
	m.addClause(c); 
	c = not(Here_now_R_mvW0)  or W_now_nobodyhome or W_now_R_mvW0  or W_now_R_accW                                  	                                                                               or W_now_CR_mvW1;
	m.addClause(c);
        c = not(Here_now_R_accW)  or W_now_nobodyhome                  or W_now_R_accW                                  	                                                                               or W_now_CR_mvW1;
	m.addClause(c);
	c = not(Here_now_R_mvW0)  or W_now_nobodyhome or E_now_R_mvW0                                   ; 
	m.addClause(c);


	c = not(Here_now_CR_mvN0) or N_now_nobodyhome or N_now_CR_mvN0 or N_now_CR_accN or N_now_CR_mvN1                           or N_now_CR_mvN2      or N_now_CR_mvN3                     or N_now_R_accN  or N_now_R_mvN1 ;
	m.addClause(c);
	c = not(Here_now_CR_accN) or N_now_nobodyhome                  or N_now_CR_accN or N_now_CR_mvN1                           or N_now_CR_mvN2      or N_now_CR_mvN3                     or N_now_R_accN  or N_now_R_mvN1 ;
	m.addClause(c);
	c = not(Here_now_CR_mvN1) or N_now_nobodyhome                                   or N_now_CR_mvN1                           or N_now_CR_mvN2      or N_now_CR_mvN3                                      or N_now_R_mvN1 ;
	m.addClause(c);
	c = not(Here_now_CR_mvN2) or N_now_nobodyhome                                                                              or N_now_CR_mvN2      or N_now_CR_mvN3                                                      ;
	m.addClause(c);
	c = not(Here_now_CR_mvN3) or N_now_nobodyhome                                                                                                    or N_now_CR_mvN3                                                      ;
	m.addClause(c);
	c = not(Here_now_CR_mvN0) or S_now_nobodyhome or S_now_CR_mvN0                                  ;
	m.addClause(c);

	c = not(Here_now_CR_mvS0) or S_now_nobodyhome or S_now_CR_mvS0 or S_now_CR_accS or S_now_CR_mvS1                           or S_now_CR_mvS2      or S_now_CR_mvS3                     or S_now_R_accS  or S_now_R_mvS1 ;
	m.addClause(c);
	c = not(Here_now_CR_accS) or S_now_nobodyhome                  or S_now_CR_accS or S_now_CR_mvS1                           or S_now_CR_mvS2      or S_now_CR_mvS3                     or S_now_R_accS  or S_now_R_mvS1 ;
	m.addClause(c);
	c = not(Here_now_CR_mvS1) or S_now_nobodyhome                  or S_now_CR_mvS1                                            or S_now_CR_mvS2      or S_now_CR_mvS3                                      or S_now_R_mvS1 ;
	m.addClause(c);
	c = not(Here_now_CR_mvS2) or S_now_nobodyhome                                                                              or S_now_CR_mvS2      or S_now_CR_mvS3                                                      ;
	m.addClause(c);
	c = not(Here_now_CR_mvS3) or S_now_nobodyhome                                                                                                    or S_now_CR_mvS3                                                      ;
	m.addClause(c);
	c = not(Here_now_CR_mvS0) or S_now_nobodyhome or N_now_CR_mvS0                                  ;
	m.addClause(c);    

	c = not(Here_now_CR_mvE0) or E_now_nobodyhome or E_now_CR_mvE0 or E_now_CR_accE or E_now_CR_mvE1                           or E_now_R_accE                                                                             ;
	m.addClause(c);
	c = not(Here_now_CR_accE) or E_now_nobodyhome                  or E_now_CR_accE or E_now_CR_mvE1                           or E_now_R_accE                                                                             ;
	m.addClause(c);
	c = not(Here_now_CR_mvE1) or E_now_nobodyhome                                   or E_now_CR_mvE1;
	m.addClause(c);
	c = not(Here_now_CR_mvE0) or W_now_nobodyhome or W_now_CR_mvE0                                  ;
	m.addClause(c);
	c = not(Here_now_CR_mvW0) or W_now_nobodyhome or W_now_CR_mvW0 or W_now_CR_accW or W_now_CR_mvW1                           or W_now_R_accW                                                                             ;
	m.addClause(c);
	c = not(Here_now_CR_accW) or W_now_nobodyhome                  or W_now_CR_accW or W_now_CR_mvW1                           or W_now_R_accW                                                                             ;
	m.addClause(c);
	c = not(Here_now_CR_mvW1) or W_now_nobodyhome                                   or W_now_CR_mvW1;
	m.addClause(c);
	c = not(Here_now_CR_mvW0) or E_now_nobodyhome or E_now_CR_mvW0                                  ;

    }


    // L I F T I N G + D R O P P I N G
    {
        CNF::Var Here_now_empty       = var(v,       t,    On_Node::empty);

	CNF::Var Here_now_R_ready     = var(v,       t,    NdStat::R_ready);
	CNF::Var Here_now_R_lift      = var(v,       t,    R_Vertical::lift);
	CNF::Var Here_now_R_lifting1  = var(v,       t,    R_Vertical::l1);
	CNF::Var Here_now_R_lifting2  = var(v,       t,    R_Vertical::l2);
	CNF::Var Here_now_R_lifting3  = var(v,       t,    R_Vertical::l3);
	CNF::Var Here_now_R_lifting4  = var(v,       t,    R_Vertical::l4);
	CNF::Var Here_now_R_drop      = var(v,       t,    R_Vertical::drop);

	c = not(Here_now_R_lift)     or  not(Here_now_empty);
	m.addClause(c);
	c = not(Here_now_R_lifting1) or  not(Here_now_empty);
	m.addClause(c);
	c = not(Here_now_R_lifting2) or  not(Here_now_empty);
	m.addClause(c);
	c = not(Here_now_R_lifting3) or  not(Here_now_empty);
	m.addClause(c);
	c = not(Here_now_R_lifting4) or  not(Here_now_empty);
	m.addClause(c);
	c = not(Here_now_R_drop)     or  not(Here_now_empty);
	m.addClause(c);
    }

} // atom_constraints()


//********************************************************************************************************************************************************************************************************
//       T I M E  L I N K     C O N S T R A I N T S
//********************************************************************************************************************************************************************************************************

void GridSpace::Grid_Gurobi::time_link_constraints(const XY v, const unsigned t)
{
    if (t>=t_max) throw std::range_error("Grid_Gurobi::time_link_constraints(): It's too late!");

    // B A S I C S
    {
        CNF::Var Here_now_nobodyhome= var(v,    t,     NdStat::nobodyhome);
	CNF::Var Here_now_R_ready   = var(v,    t,     NdStat::R_ready);
	CNF::Var Here_now_R_accE    = var(v,    t,     R_Move::accE);
	CNF::Var Here_now_R_mvE0    = var(v,    t,     R_Move::mvE0);
	CNF::Var Here_now_R_accW    = var(v,    t,     R_Move::accW);
	CNF::Var Here_now_R_mvW0    = var(v,    t,     R_Move::mvW0);
	CNF::Var Here_now_R_accN    = var(v,    t,     R_Move::accN);
	CNF::Var Here_now_R_mvN1    = var(v,    t,     R_Move::mvN1);
	CNF::Var Here_now_R_mvN0    = var(v,    t,     R_Move::mvN0);
	CNF::Var Here_now_R_accS    = var(v,    t,     R_Move::accS);
	CNF::Var Here_now_R_mvS1    = var(v,    t,     R_Move::mvS1);
	CNF::Var Here_now_R_mvS0    = var(v,    t,     R_Move::mvS0);
	CNF::Var Here_now_R_lift    = var(v,    t,     R_Vertical::lift);
	CNF::Var Here_now_R_lifting1= var(v,    t,     R_Vertical::l1);
	CNF::Var Here_now_R_lifting2= var(v,    t,     R_Vertical::l2);
	CNF::Var Here_now_R_lifting3= var(v,    t,     R_Vertical::l3);
	CNF::Var Here_now_R_lifting4= var(v,    t,     R_Vertical::l4);
	CNF::Var Here_now_R_drop    = var(v,    t,     R_Vertical::drop);
	CNF::Var Here_now_C0R_ready = var(v,    t,     NdStat::C0R_ready);
	CNF::Var Here_now_C0R_accE  = var(v,    t,     R_Move::w0_accE);
	CNF::Var Here_now_C0R_mvE1  = var(v,    t,     R_Move::w0_mvE1);
	CNF::Var Here_now_C0R_mvE0  = var(v,    t,     R_Move::w0_mvE0);
	CNF::Var Here_now_C0R_accW  = var(v,    t,     R_Move::w0_accW);
	CNF::Var Here_now_C0R_mvW1  = var(v,    t,     R_Move::w0_mvW1);
	CNF::Var Here_now_C0R_mvW0  = var(v,    t,     R_Move::w0_mvW0);
	CNF::Var Here_now_C0R_accN  = var(v,    t,     R_Move::w0_accN);
	CNF::Var Here_now_C0R_mvN1  = var(v,    t,     R_Move::w0_mvN1);
	CNF::Var Here_now_C0R_mvN2  = var(v,    t,     R_Move::w0_mvN2);
	CNF::Var Here_now_C0R_mvN3  = var(v,    t,     R_Move::w0_mvN3);
	CNF::Var Here_now_C0R_mvN0  = var(v,    t,     R_Move::w0_mvN0);
	CNF::Var Here_now_C0R_accS  = var(v,    t,     R_Move::w0_accS);
	CNF::Var Here_now_C0R_mvS1  = var(v,    t,     R_Move::w0_mvS1);
	CNF::Var Here_now_C0R_mvS2  = var(v,    t,     R_Move::w0_mvS2);
	CNF::Var Here_now_C0R_mvS3  = var(v,    t,     R_Move::w0_mvS3);
	CNF::Var Here_now_C0R_mvS0  = var(v,    t,     R_Move::w0_mvS0);
	CNF::Var Here_now_C1R_ready = var(v,    t,     NdStat::C1R_ready);
	CNF::Var Here_now_C1R_accE  = var(v,    t,     R_Move::w1_accE);
	CNF::Var Here_now_C1R_mvE1  = var(v,    t,     R_Move::w1_mvE1);
	CNF::Var Here_now_C1R_mvE0  = var(v,    t,     R_Move::w1_mvE0);
	CNF::Var Here_now_C1R_accW  = var(v,    t,     R_Move::w1_accW);
	CNF::Var Here_now_C1R_mvW1  = var(v,    t,     R_Move::w1_mvW1);
	CNF::Var Here_now_C1R_mvW0  = var(v,    t,     R_Move::w1_mvW0);
	CNF::Var Here_now_C1R_accN  = var(v,    t,     R_Move::w1_accN);
	CNF::Var Here_now_C1R_mvN1  = var(v,    t,     R_Move::w1_mvN1);
	CNF::Var Here_now_C1R_mvN2  = var(v,    t,     R_Move::w1_mvN2);
	CNF::Var Here_now_C1R_mvN3  = var(v,    t,     R_Move::w1_mvN3);
	CNF::Var Here_now_C1R_mvN0  = var(v,    t,     R_Move::w1_mvN0);
	CNF::Var Here_now_C1R_accS  = var(v,    t,     R_Move::w1_accS);
	CNF::Var Here_now_C1R_mvS1  = var(v,    t,     R_Move::w1_mvS1);
	CNF::Var Here_now_C1R_mvS2  = var(v,    t,     R_Move::w1_mvS2);
	CNF::Var Here_now_C1R_mvS3  = var(v,    t,     R_Move::w1_mvS3);
	CNF::Var Here_now_C1R_mvS0  = var(v,    t,     R_Move::w1_mvS0);
	CNF::Var Here_now_C2R_ready = var(v,    t,     NdStat::C2R_ready);
	CNF::Var Here_now_C2R_accE  = var(v,    t,     R_Move::w2_accE);
	CNF::Var Here_now_C2R_mvE1  = var(v,    t,     R_Move::w2_mvE1);
	CNF::Var Here_now_C2R_mvE0  = var(v,    t,     R_Move::w2_mvE0);
	CNF::Var Here_now_C2R_accW  = var(v,    t,     R_Move::w2_accW);
	CNF::Var Here_now_C2R_mvW1  = var(v,    t,     R_Move::w2_mvW1);
	CNF::Var Here_now_C2R_mvW0  = var(v,    t,     R_Move::w2_mvW0);
	CNF::Var Here_now_C2R_accN  = var(v,    t,     R_Move::w2_accN);
	CNF::Var Here_now_C2R_mvN1  = var(v,    t,     R_Move::w2_mvN1);
	CNF::Var Here_now_C2R_mvN2  = var(v,    t,     R_Move::w2_mvN2);
	CNF::Var Here_now_C2R_mvN3  = var(v,    t,     R_Move::w2_mvN3);
	CNF::Var Here_now_C2R_mvN0  = var(v,    t,     R_Move::w2_mvN0);
	CNF::Var Here_now_C2R_accS  = var(v,    t,     R_Move::w2_accS);
	CNF::Var Here_now_C2R_mvS1  = var(v,    t,     R_Move::w2_mvS1);
	CNF::Var Here_now_C2R_mvS2  = var(v,    t,     R_Move::w2_mvS2);
	CNF::Var Here_now_C2R_mvS3  = var(v,    t,     R_Move::w2_mvS3);
	CNF::Var Here_now_C2R_mvS0  = var(v,    t,     R_Move::w2_mvS0);


	CNF::Var Here_will_nobodyhome= var(v,    t+1,   NdStat::nobodyhome);
	CNF::Var Here_will_R_ready   = var(v,    t+1,   NdStat::R_ready);
	CNF::Var Here_will_R_accE    = var(v,    t+1,   R_Move::accE);
	CNF::Var Here_will_R_mvE0    = var(v,    t+1,   R_Move::mvE0);
	CNF::Var Here_will_R_accW    = var(v,    t+1,   R_Move::accW);
	CNF::Var Here_will_R_mvW0    = var(v,    t+1,   R_Move::mvW0);
	CNF::Var Here_will_R_accN    = var(v,    t+1,   R_Move::accN);
	CNF::Var Here_will_R_mvN1    = var(v,    t+1,   R_Move::mvN1);
	CNF::Var Here_will_R_mvN0    = var(v,    t+1,   R_Move::mvN0);
	CNF::Var Here_will_R_accS    = var(v,    t+1,   R_Move::accS);
	CNF::Var Here_will_R_mvS1    = var(v,    t+1,   R_Move::mvS1);
	CNF::Var Here_will_R_mvS0    = var(v,    t+1,   R_Move::mvS0);
	CNF::Var Here_will_R_lift    = var(v,    t+1,   R_Vertical::lift);
	CNF::Var Here_will_R_lifting1= var(v,    t+1,   R_Vertical::l1);
	CNF::Var Here_will_R_lifting2= var(v,    t+1,   R_Vertical::l2);
	CNF::Var Here_will_R_lifting3= var(v,    t+1,   R_Vertical::l3);
	CNF::Var Here_will_R_lifting4= var(v,    t+1,   R_Vertical::l4);
	CNF::Var Here_will_R_drop    = var(v,    t+1,   R_Vertical::drop);
	CNF::Var Here_will_C0R_ready = var(v,    t+1,   NdStat::C0R_ready);
	CNF::Var Here_will_C0R_accE  = var(v,    t+1,   R_Move::w0_accE);
	CNF::Var Here_will_C0R_mvE1  = var(v,    t+1,   R_Move::w0_mvE1);
	CNF::Var Here_will_C0R_mvE0  = var(v,    t+1,   R_Move::w0_mvE0);
	CNF::Var Here_will_C0R_accW  = var(v,    t+1,   R_Move::w0_accW);
	CNF::Var Here_will_C0R_mvW1  = var(v,    t+1,   R_Move::w0_mvW1);
	CNF::Var Here_will_C0R_mvW0  = var(v,    t+1,   R_Move::w0_mvW0);
	CNF::Var Here_will_C0R_accN  = var(v,    t+1,   R_Move::w0_accN);
	CNF::Var Here_will_C0R_mvN1  = var(v,    t+1,   R_Move::w0_mvN1);
	CNF::Var Here_will_C0R_mvN2  = var(v,    t+1,   R_Move::w0_mvN2);
	CNF::Var Here_will_C0R_mvN3  = var(v,    t+1,   R_Move::w0_mvN3);
	CNF::Var Here_will_C0R_mvN0  = var(v,    t+1,   R_Move::w0_mvN0);
	CNF::Var Here_will_C0R_accS  = var(v,    t+1,   R_Move::w0_accS);
	CNF::Var Here_will_C0R_mvS1  = var(v,    t+1,   R_Move::w0_mvS1);
	CNF::Var Here_will_C0R_mvS2  = var(v,    t+1,   R_Move::w0_mvS2);
	CNF::Var Here_will_C0R_mvS3  = var(v,    t+1,   R_Move::w0_mvS3);
	CNF::Var Here_will_C0R_mvS0  = var(v,    t+1,   R_Move::w0_mvS0);
	CNF::Var Here_will_C1R_ready = var(v,    t+1,   NdStat::C1R_ready);
	CNF::Var Here_will_C1R_accE  = var(v,    t+1,   R_Move::w1_accE);
	CNF::Var Here_will_C1R_mvE1  = var(v,    t+1,   R_Move::w1_mvE1);
	CNF::Var Here_will_C1R_mvE0  = var(v,    t+1,   R_Move::w1_mvE0);
	CNF::Var Here_will_C1R_accW  = var(v,    t+1,   R_Move::w1_accW);
	CNF::Var Here_will_C1R_mvW1  = var(v,    t+1,   R_Move::w1_mvW1);
	CNF::Var Here_will_C1R_mvW0  = var(v,    t+1,   R_Move::w1_mvW0);
	CNF::Var Here_will_C1R_accN  = var(v,    t+1,   R_Move::w1_accN);
	CNF::Var Here_will_C1R_mvN1  = var(v,    t+1,   R_Move::w1_mvN1);
	CNF::Var Here_will_C1R_mvN2  = var(v,    t+1,   R_Move::w1_mvN2);
	CNF::Var Here_will_C1R_mvN3  = var(v,    t+1,   R_Move::w1_mvN3);
	CNF::Var Here_will_C1R_mvN0  = var(v,    t+1,   R_Move::w1_mvN0);
	CNF::Var Here_will_C1R_accS  = var(v,    t+1,   R_Move::w1_accS);
	CNF::Var Here_will_C1R_mvS1  = var(v,    t+1,   R_Move::w1_mvS1);
	CNF::Var Here_will_C1R_mvS2  = var(v,    t+1,   R_Move::w1_mvS2);
	CNF::Var Here_will_C1R_mvS3  = var(v,    t+1,   R_Move::w1_mvS3);
	CNF::Var Here_will_C1R_mvS0  = var(v,    t+1,   R_Move::w1_mvS0);
	CNF::Var Here_will_C2R_ready = var(v,    t+1,   NdStat::C2R_ready);
	CNF::Var Here_will_C2R_accE  = var(v,    t+1,   R_Move::w2_accE);
	CNF::Var Here_will_C2R_mvE1  = var(v,    t+1,   R_Move::w2_mvE1);
	CNF::Var Here_will_C2R_mvE0  = var(v,    t+1,   R_Move::w2_mvE0);
	CNF::Var Here_will_C2R_accW  = var(v,    t+1,   R_Move::w2_accW);
	CNF::Var Here_will_C2R_mvW1  = var(v,    t+1,   R_Move::w2_mvW1);
	CNF::Var Here_will_C2R_mvW0  = var(v,    t+1,   R_Move::w2_mvW0);
	CNF::Var Here_will_C2R_accN  = var(v,    t+1,   R_Move::w2_accN);
	CNF::Var Here_will_C2R_mvN1  = var(v,    t+1,   R_Move::w2_mvN1);
	CNF::Var Here_will_C2R_mvN2  = var(v,    t+1,   R_Move::w2_mvN2);
	CNF::Var Here_will_C2R_mvN3  = var(v,    t+1,   R_Move::w2_mvN3);
	CNF::Var Here_will_C2R_mvN0  = var(v,    t+1,   R_Move::w2_mvN0);
	CNF::Var Here_will_C2R_accS  = var(v,    t+1,   R_Move::w2_accS);
	CNF::Var Here_will_C2R_mvS1  = var(v,    t+1,   R_Move::w2_mvS1);
	CNF::Var Here_will_C2R_mvS2  = var(v,    t+1,   R_Move::w2_mvS2);
	CNF::Var Here_will_C2R_mvS3  = var(v,    t+1,   R_Move::w2_mvS3);
	CNF::Var Here_will_C2R_mvS0  = var(v,    t+1,   R_Move::w2_mvS0);



	CNF::Var W_now_R_accE    = var( G.west(v),    t,    R_Move::accE);
	CNF::Var W_now_R_mvE0    = var( G.west(v),    t,    R_Move::mvE0);
	CNF::Var W_now_C0R_mvE1  = var( G.west(v),    t,    R_Move::w0_mvE1);
	CNF::Var W_now_C1R_mvE1  = var( G.west(v),    t,    R_Move::w1_mvE1);
	CNF::Var W_now_C2R_mvE1  = var( G.west(v),    t,    R_Move::w2_mvE1);

	CNF::Var E_now_R_accW    = var( G.east(v),    t,    R_Move::accW);
	CNF::Var E_now_R_mvW0    = var( G.east(v),    t,    R_Move::mvW0);
	CNF::Var E_now_C0R_mvW1  = var( G.east(v),    t,    R_Move::w0_mvW1);
	CNF::Var E_now_C1R_mvW1  = var( G.east(v),    t,    R_Move::w1_mvW1);
	CNF::Var E_now_C2R_mvW1  = var( G.east(v),    t,    R_Move::w2_mvW1);

	CNF::Var S_now_R_mvN1    = var( G.south(v),    t,    R_Move::mvN1);
	CNF::Var S_now_C0R_mvN3  = var( G.south(v),    t,    R_Move::w0_mvN3);
	CNF::Var S_now_C1R_mvN3  = var( G.south(v),    t,    R_Move::w1_mvN3);
	CNF::Var S_now_C2R_mvN3  = var( G.south(v),    t,    R_Move::w2_mvN3);

	CNF::Var N_now_R_mvS1    = var( G.north(v),    t,    R_Move::mvS1);
	CNF::Var N_now_C0R_mvS3  = var( G.north(v),    t,    R_Move::w0_mvS3);
	CNF::Var N_now_C1R_mvS3  = var( G.north(v),    t,    R_Move::w1_mvS3);
	CNF::Var N_now_C2R_mvS3  = var( G.north(v),    t,    R_Move::w2_mvS3);

	c = not(Here_now_R_ready)    or Here_will_R_ready                                                                                                   or Here_will_R_accE    or Here_will_R_accN   or Here_will_R_accW   or Here_will_R_accS                                 or Here_will_R_lift;
	m.addClause(c);
	c = not(Here_will_R_ready)   or Here_now_R_ready                                                                                                    or W_now_R_mvE0        or W_now_R_accE                                                                                 or E_now_R_mvW0        or E_now_R_accW                                                                                 or S_now_R_mvN1                                                                                                        or N_now_R_mvS1                                                                                                        or Here_now_R_drop;
	m.addClause(c);
	c = not(Here_now_C0R_ready)  or Here_will_C0R_ready                                                                                                 or Here_will_C0R_accE  or Here_will_C0R_accN or Here_will_C0R_accW or Here_will_C0R_accS                                                                                                                                                      or Here_will_R_drop;
	m.addClause(c);
	c = not(Here_will_C0R_ready) or Here_now_C0R_ready                                                                                                  or W_now_C0R_mvE1                                                                                                      or E_now_C0R_mvW1                                                                                                      or S_now_C0R_mvN3                                                                                                      or N_now_C0R_mvS3                                                                                                      or Here_now_R_lifting4;
	m.addClause(c);
	c = not(Here_now_C1R_ready)  or Here_will_C1R_ready                                                                                                 or Here_will_C1R_accE  or Here_will_C1R_accN or Here_will_C1R_accW or Here_will_C1R_accS                                                                                                                                                      or Here_will_R_drop;
	m.addClause(c);
	c = not(Here_will_C1R_ready) or Here_now_C1R_ready                                                                                                  or W_now_C1R_mvE1                                                                                                      or E_now_C1R_mvW1                                                                                                      or S_now_C1R_mvN3                                                                                                      or N_now_C1R_mvS3                                                                                                      or Here_now_R_lifting4;
	m.addClause(c);
	c = not(Here_now_C2R_ready)  or Here_will_C2R_ready                                                                                                 or Here_will_C2R_accE or Here_will_C2R_accN or Here_will_C2R_accW or Here_will_C2R_accS                                                                                                                                                       or Here_will_R_drop;
	m.addClause(c);
	c = not(Here_will_C2R_ready) or Here_now_C2R_ready                                                                                                  or W_now_C2R_mvE1                                                                                                      or E_now_C2R_mvW1                                                                                                      or S_now_C2R_mvN3                                                                                                      or N_now_C2R_mvS3                                                                                                      or Here_now_R_lifting4;
	m.addClause(c);
    }

    // M O V E M E N T
    {
        CNF::Var Here_now_nobodyhome= var(v,    t,     NdStat::nobodyhome);
	CNF::Var Here_now_R_ready   = var(v,    t,     NdStat::R_ready);
	CNF::Var Here_now_R_accE    = var(v,    t,     R_Move::accE);
	CNF::Var Here_now_R_mvE0    = var(v,    t,     R_Move::mvE0);
	CNF::Var Here_now_R_accW    = var(v,    t,     R_Move::accW);
	CNF::Var Here_now_R_mvW0    = var(v,    t,     R_Move::mvW0);
	CNF::Var Here_now_R_accN    = var(v,    t,     R_Move::accN);
	CNF::Var Here_now_R_mvN1    = var(v,    t,     R_Move::mvN1);
	CNF::Var Here_now_R_mvN0    = var(v,    t,     R_Move::mvN0);
	CNF::Var Here_now_R_accS    = var(v,    t,     R_Move::accS);
	CNF::Var Here_now_R_mvS1    = var(v,    t,     R_Move::mvS1);
	CNF::Var Here_now_R_mvS0    = var(v,    t,     R_Move::mvS0);
	CNF::Var Here_now_R_lift    = var(v,    t,     R_Vertical::lift);
	CNF::Var Here_now_R_lifting1= var(v,    t,     R_Vertical::l1);
	CNF::Var Here_now_R_lifting2= var(v,    t,     R_Vertical::l2);
	CNF::Var Here_now_R_lifting3= var(v,    t,     R_Vertical::l3);
	CNF::Var Here_now_R_lifting4= var(v,    t,     R_Vertical::l4);
	CNF::Var Here_now_R_drop    = var(v,    t,     R_Vertical::drop);
	CNF::Var Here_now_C0R_ready = var(v,    t,     NdStat::C0R_ready);
	CNF::Var Here_now_C0R_accE  = var(v,    t,     R_Move::w0_accE);
	CNF::Var Here_now_C0R_mvE1  = var(v,    t,     R_Move::w0_mvE1);
	CNF::Var Here_now_C0R_mvE0  = var(v,    t,     R_Move::w0_mvE0);
	CNF::Var Here_now_C0R_accW  = var(v,    t,     R_Move::w0_accW);
	CNF::Var Here_now_C0R_mvW1  = var(v,    t,     R_Move::w0_mvW1);
	CNF::Var Here_now_C0R_mvW0  = var(v,    t,     R_Move::w0_mvW0);
	CNF::Var Here_now_C0R_accN  = var(v,    t,     R_Move::w0_accN);
	CNF::Var Here_now_C0R_mvN1  = var(v,    t,     R_Move::w0_mvN1);
	CNF::Var Here_now_C0R_mvN2  = var(v,    t,     R_Move::w0_mvN2);
	CNF::Var Here_now_C0R_mvN3  = var(v,    t,     R_Move::w0_mvN3);
	CNF::Var Here_now_C0R_mvN0  = var(v,    t,     R_Move::w0_mvN0);
	CNF::Var Here_now_C0R_accS  = var(v,    t,     R_Move::w0_accS);
	CNF::Var Here_now_C0R_mvS1  = var(v,    t,     R_Move::w0_mvS1);
	CNF::Var Here_now_C0R_mvS2  = var(v,    t,     R_Move::w0_mvS2);
	CNF::Var Here_now_C0R_mvS3  = var(v,    t,     R_Move::w0_mvS3);
	CNF::Var Here_now_C0R_mvS0  = var(v,    t,     R_Move::w0_mvS0);
	CNF::Var Here_now_C1R_ready = var(v,    t,     NdStat::C1R_ready);
	CNF::Var Here_now_C1R_accE  = var(v,    t,     R_Move::w1_accE);
	CNF::Var Here_now_C1R_mvE1  = var(v,    t,     R_Move::w1_mvE1);
	CNF::Var Here_now_C1R_mvE0  = var(v,    t,     R_Move::w1_mvE0);
	CNF::Var Here_now_C1R_accW  = var(v,    t,     R_Move::w1_accW);
	CNF::Var Here_now_C1R_mvW1  = var(v,    t,     R_Move::w1_mvW1);
	CNF::Var Here_now_C1R_mvW0  = var(v,    t,     R_Move::w1_mvW0);
	CNF::Var Here_now_C1R_accN  = var(v,    t,     R_Move::w1_accN);
	CNF::Var Here_now_C1R_mvN1  = var(v,    t,     R_Move::w1_mvN1);
	CNF::Var Here_now_C1R_mvN2  = var(v,    t,     R_Move::w1_mvN2);
	CNF::Var Here_now_C1R_mvN3  = var(v,    t,     R_Move::w1_mvN3);
	CNF::Var Here_now_C1R_mvN0  = var(v,    t,     R_Move::w1_mvN0);
	CNF::Var Here_now_C1R_accS  = var(v,    t,     R_Move::w1_accS);
	CNF::Var Here_now_C1R_mvS1  = var(v,    t,     R_Move::w1_mvS1);
	CNF::Var Here_now_C1R_mvS2  = var(v,    t,     R_Move::w1_mvS2);
	CNF::Var Here_now_C1R_mvS3  = var(v,    t,     R_Move::w1_mvS3);
	CNF::Var Here_now_C1R_mvS0  = var(v,    t,     R_Move::w1_mvS0);
	CNF::Var Here_now_C2R_ready = var(v,    t,     NdStat::C2R_ready);
	CNF::Var Here_now_C2R_accE  = var(v,    t,     R_Move::w2_accE);
	CNF::Var Here_now_C2R_mvE1  = var(v,    t,     R_Move::w2_mvE1);
	CNF::Var Here_now_C2R_mvE0  = var(v,    t,     R_Move::w2_mvE0);
	CNF::Var Here_now_C2R_accW  = var(v,    t,     R_Move::w2_accW);
	CNF::Var Here_now_C2R_mvW1  = var(v,    t,     R_Move::w2_mvW1);
	CNF::Var Here_now_C2R_mvW0  = var(v,    t,     R_Move::w2_mvW0);
	CNF::Var Here_now_C2R_accN  = var(v,    t,     R_Move::w2_accN);
	CNF::Var Here_now_C2R_mvN1  = var(v,    t,     R_Move::w2_mvN1);
	CNF::Var Here_now_C2R_mvN2  = var(v,    t,     R_Move::w2_mvN2);
	CNF::Var Here_now_C2R_mvN3  = var(v,    t,     R_Move::w2_mvN3);
	CNF::Var Here_now_C2R_mvN0  = var(v,    t,     R_Move::w2_mvN0);
	CNF::Var Here_now_C2R_accS  = var(v,    t,     R_Move::w2_accS);
	CNF::Var Here_now_C2R_mvS1  = var(v,    t,     R_Move::w2_mvS1);
	CNF::Var Here_now_C2R_mvS2  = var(v,    t,     R_Move::w2_mvS2);
	CNF::Var Here_now_C2R_mvS3  = var(v,    t,     R_Move::w2_mvS3);
	CNF::Var Here_now_C2R_mvS0  = var(v,    t,     R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var Here_now_CR_mvE1;
	// necessarly
	c = not(Here_now_CR_mvE1)  or Here_now_C0R_mvE1      or Here_now_C1R_mvE1      or Here_now_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvE1) or Here_now_C1R_mvE1      or Here_now_C2R_mvE1      or Here_now_CR_mvE1;
	m.addClause(c);
	c = Here_now_C0R_mvE1      or not(Here_now_C1R_mvE1) or Here_now_C2R_mvE1      or Here_now_CR_mvE1;
	m.addClause(c);
	c = Here_now_C0R_mvE1      or Here_now_C1R_mvE1      or not(Here_now_C2R_mvE1) or Here_now_CR_mvE1;
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C1R_mvE1);
	m.addClause(c);
	c = not(Here_now_C0R_mvE1) or not(Here_now_C2R_mvE1);
	m.addClause(c);
	c = not(Here_now_C1R_mvE1) or not(Here_now_C2R_mvE1);
	m.addClause(c);
	CNF::Var Here_now_CR_mvE0;
	// necessalry
	c = not(Here_now_CR_mvE0)  or Here_now_C0R_mvE0      or Here_now_C1R_mvE0      or Here_now_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvE0) or Here_now_C1R_mvE0      or Here_now_C2R_mvE0      or Here_now_CR_mvE0;
	m.addClause(c);
	c = Here_now_C0R_mvE0      or not(Here_now_C1R_mvE0) or Here_now_C2R_mvE0      or Here_now_CR_mvE0;
	m.addClause(c);
	c = Here_now_C0R_mvE0      or Here_now_C1R_mvE0      or not(Here_now_C2R_mvE0) or Here_now_CR_mvE0;
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C1R_mvE0);
	m.addClause(c);
	c = not(Here_now_C0R_mvE0) or not(Here_now_C2R_mvE0);
	m.addClause(c);
	c = not(Here_now_C1R_mvE0) or not(Here_now_C2R_mvE0);
	m.addClause(c);

	CNF::Var Here_now_CR_mvW1;
	// necessarly
	c = not(Here_now_CR_mvW1)  or Here_now_C0R_mvW1      or Here_now_C1R_mvW1      or Here_now_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvW1) or Here_now_C1R_mvW1      or Here_now_C2R_mvW1      or Here_now_CR_mvW1;
	m.addClause(c);
	c = Here_now_C0R_mvW1      or not(Here_now_C1R_mvW1) or Here_now_C2R_mvW1      or Here_now_CR_mvW1;
	m.addClause(c);
	c = Here_now_C0R_mvW1      or Here_now_C1R_mvW1      or not(Here_now_C2R_mvW1) or Here_now_CR_mvW1;
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C1R_mvW1);
	m.addClause(c);
	c = not(Here_now_C0R_mvW1) or not(Here_now_C2R_mvW1);
	m.addClause(c);
	c = not(Here_now_C1R_mvW1) or not(Here_now_C2R_mvW1);
	m.addClause(c);
	CNF::Var Here_now_CR_mvW0;
	// necessarly
	c = not(Here_now_CR_mvW0)  or Here_now_C0R_mvW0      or Here_now_C1R_mvW0      or Here_now_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvW0) or Here_now_C1R_mvW0      or Here_now_C2R_mvW0      or Here_now_CR_mvW0;
	m.addClause(c);
	c = Here_now_C0R_mvW0      or not(Here_now_C1R_mvW0) or Here_now_C2R_mvW0      or Here_now_CR_mvW0;
	m.addClause(c);
	c = Here_now_C0R_mvW0      or Here_now_C1R_mvW0      or not(Here_now_C2R_mvW0) or Here_now_CR_mvW0;
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C1R_mvW0);
	m.addClause(c);
	c = not(Here_now_C0R_mvW0) or not(Here_now_C2R_mvW0);
	m.addClause(c);
	c = not(Here_now_C1R_mvW0) or not(Here_now_C2R_mvW0);
	m.addClause(c);

	CNF::Var Here_now_CR_mvN1;
	// neccessarly
	c = not(Here_now_CR_mvN1)  or Here_now_C0R_mvN1      or Here_now_C1R_mvN1      or Here_now_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvN1) or Here_now_C1R_mvN1      or Here_now_C2R_mvN1      or Here_now_CR_mvN1;
	m.addClause(c);
	c = Here_now_C0R_mvN1      or not(Here_now_C1R_mvN1) or Here_now_C2R_mvN1      or Here_now_CR_mvN1;
	m.addClause(c);
	c = Here_now_C0R_mvN1      or Here_now_C1R_mvN1      or not(Here_now_C2R_mvN1) or Here_now_CR_mvN1;
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C1R_mvN1);
	m.addClause(c);
	c = not(Here_now_C0R_mvN1) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	c = not(Here_now_C1R_mvN1) or not(Here_now_C2R_mvN1);
	m.addClause(c);
	CNF::Var Here_now_CR_mvN2;
	// neccessarly
	c = not(Here_now_CR_mvN2)  or Here_now_C0R_mvN2      or Here_now_C1R_mvN2       or Here_now_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvN2) or Here_now_C1R_mvN2      or Here_now_C2R_mvN2       or Here_now_CR_mvN2;
	m.addClause(c);
	c = Here_now_C0R_mvN2      or not(Here_now_C1R_mvN2) or Here_now_C2R_mvN2       or Here_now_CR_mvN2;
	m.addClause(c);
	c = Here_now_C0R_mvN2      or Here_now_C1R_mvN2      or not(Here_now_C2R_mvN2)  or Here_now_CR_mvN2;
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C1R_mvN2);
	m.addClause(c);
	c = not(Here_now_C0R_mvN2) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	c = not(Here_now_C1R_mvN2) or not(Here_now_C2R_mvN2);
	m.addClause(c);
	CNF::Var Here_now_CR_mvN3;
	// neccessarly
	c = not(Here_now_CR_mvN3)  or Here_now_C0R_mvN3      or Here_now_C1R_mvN3      or Here_now_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvN3) or Here_now_C1R_mvN3      or Here_now_C2R_mvN3      or Here_now_CR_mvN3;
	m.addClause(c);
	c = Here_now_C0R_mvN3      or not(Here_now_C1R_mvN3) or Here_now_C2R_mvN3      or Here_now_CR_mvN3;
	m.addClause(c);
	c = Here_now_C0R_mvN3      or Here_now_C1R_mvN3      or not(Here_now_C2R_mvN3) or Here_now_CR_mvN3;
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C1R_mvN3);
	m.addClause(c);
	c = not(Here_now_C0R_mvN3) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	c = not(Here_now_C1R_mvN3) or not(Here_now_C2R_mvN3);
	m.addClause(c);
	CNF::Var Here_now_CR_mvN0;
	// neccessarly
	c = not(Here_now_CR_mvN0)  or Here_now_C0R_mvN0      or Here_now_C1R_mvN0      or Here_now_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvN0) or Here_now_C1R_mvN0      or Here_now_C2R_mvN0      or Here_now_CR_mvN0;
	m.addClause(c);
	c = Here_now_C0R_mvN0      or not(Here_now_C1R_mvN0) or Here_now_C2R_mvN0      or Here_now_CR_mvN0;
	m.addClause(c);
	c = Here_now_C0R_mvN0      or Here_now_C1R_mvN0      or not(Here_now_C2R_mvN0) or Here_now_CR_mvN0;
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C1R_mvN0);
	m.addClause(c);
	c = not(Here_now_C0R_mvN0) or not(Here_now_C2R_mvN0);
	m.addClause(c);
	c = not(Here_now_C1R_mvN0) or not(Here_now_C2R_mvN0);
	m.addClause(c);

	CNF::Var Here_now_CR_mvS1;
	// neccessarly
	c = not(Here_now_CR_mvS1)  or Here_now_C0R_mvS1      or Here_now_C1R_mvS1      or Here_now_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvS1) or Here_now_C1R_mvS1      or Here_now_C2R_mvS1      or Here_now_CR_mvS1;
	m.addClause(c);
	c = Here_now_C0R_mvS1      or not(Here_now_C1R_mvS1) or Here_now_C2R_mvS1      or Here_now_CR_mvS1;
	m.addClause(c);
	c = Here_now_C0R_mvS1      or Here_now_C1R_mvS1      or not(Here_now_C2R_mvS1) or Here_now_CR_mvS1;
	m.addClause(c);
	c = not(Here_now_C0R_mvS1) or not(Here_now_C1R_mvS1);
	m.addClause(c);
	c = not(Here_now_C0R_mvS1) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	c = not(Here_now_C1R_mvS1) or not(Here_now_C2R_mvS1);
	m.addClause(c);
	CNF::Var Here_now_CR_mvS2;
	// neccessarly
	c = not(Here_now_CR_mvS2)  or Here_now_C0R_mvS2      or Here_now_C1R_mvS2      or Here_now_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvS2) or Here_now_C1R_mvS2      or Here_now_C2R_mvS2      or Here_now_CR_mvS2;
	m.addClause(c);
	c = Here_now_C0R_mvS2      or not(Here_now_C1R_mvS2) or Here_now_C2R_mvS2      or Here_now_CR_mvS2;
	m.addClause(c);
	c = Here_now_C0R_mvS2      or Here_now_C1R_mvS2      or not(Here_now_C2R_mvS2) or Here_now_CR_mvS2;
	m.addClause(c);
	c = not(Here_now_C0R_mvS2) or not(Here_now_C1R_mvS2);
	m.addClause(c);
	c = not(Here_now_C0R_mvS2) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	c = not(Here_now_C1R_mvS2) or not(Here_now_C2R_mvS2);
	m.addClause(c);
	CNF::Var Here_now_CR_mvS3;
	// neccessarly
	c = not(Here_now_CR_mvS3)  or Here_now_C0R_mvS3      or Here_now_C1R_mvS3      or Here_now_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvS3) or Here_now_C1R_mvS3      or Here_now_C2R_mvS3      or Here_now_CR_mvS3;
	m.addClause(c);
	c = Here_now_C0R_mvS3      or not(Here_now_C1R_mvS3) or Here_now_C2R_mvS3      or Here_now_CR_mvS3;
	m.addClause(c);
	c = Here_now_C0R_mvS3      or Here_now_C1R_mvS3      or not(Here_now_C2R_mvS3) or Here_now_CR_mvS3;
	m.addClause(c);
	c = not(Here_now_C0R_mvS3) or not(Here_now_C1R_mvS3);
	m.addClause(c);
	c = not(Here_now_C0R_mvS3) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	c = not(Here_now_C1R_mvS3) or not(Here_now_C2R_mvS3);
	m.addClause(c);
	CNF::Var Here_now_CR_mvS0;
	// neccessarly
	c = not(Here_now_CR_mvS0)  or Here_now_C0R_mvS0      or Here_now_C1R_mvS0      or Here_now_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_mvS0) or Here_now_C1R_mvS0      or Here_now_C2R_mvS0      or Here_now_CR_mvS0;
	m.addClause(c);
	c = Here_now_C0R_mvS0      or not(Here_now_C1R_mvS0) or Here_now_C2R_mvS0      or Here_now_CR_mvS0;
	m.addClause(c);
	c = Here_now_C0R_mvS0      or Here_now_C1R_mvS0      or not(Here_now_C2R_mvS0) or Here_now_CR_mvS0;
	m.addClause(c);
	c = not(Here_now_C0R_mvS0) or not(Here_now_C1R_mvS0);
	m.addClause(c);
	c = not(Here_now_C0R_mvS0) or not(Here_now_C2R_mvS0);
	m.addClause(c);
	c = not(Here_now_C1R_mvS0) or not(Here_now_C2R_mvS0);
	m.addClause(c);


	CNF::Var E_now_nobodyhome= var( G.east(v),    t,    NdStat::nobodyhome);
	CNF::Var E_now_R_ready   = var( G.east(v),    t,    NdStat::R_ready);
	CNF::Var E_now_R_accE    = var( G.east(v),    t,    R_Move::accE);
	CNF::Var E_now_R_mvE0    = var( G.east(v),    t,    R_Move::mvE0);
	CNF::Var E_now_R_accW    = var( G.east(v),    t,    R_Move::accW);
	CNF::Var E_now_R_mvW0    = var( G.east(v),    t,    R_Move::mvW0);
	CNF::Var E_now_C0R_ready = var( G.east(v),    t,    NdStat::C0R_ready);
	CNF::Var E_now_C0R_accE  = var( G.east(v),    t,    R_Move::w0_accE);
	CNF::Var E_now_C0R_mvE1  = var( G.east(v),    t,    R_Move::w0_mvE1);
	CNF::Var E_now_C0R_mvE0  = var( G.east(v),    t,    R_Move::w0_mvE0);
	CNF::Var E_now_C0R_accW  = var( G.east(v),    t,    R_Move::w0_accW);
	CNF::Var E_now_C0R_mvW1  = var( G.east(v),    t,    R_Move::w0_mvW1);
	CNF::Var E_now_C0R_mvW0  = var( G.east(v),    t,    R_Move::w0_mvW0);
	CNF::Var E_now_C1R_ready = var( G.east(v),    t,    NdStat::C1R_ready);
	CNF::Var E_now_C1R_accE  = var( G.east(v),    t,    R_Move::w1_accE);
	CNF::Var E_now_C1R_mvE1  = var( G.east(v),    t,    R_Move::w1_mvE1);
	CNF::Var E_now_C1R_mvE0  = var( G.east(v),    t,    R_Move::w1_mvE0);
	CNF::Var E_now_C1R_accW  = var( G.east(v),    t,    R_Move::w1_accW);
	CNF::Var E_now_C1R_mvW1  = var( G.east(v),    t,    R_Move::w1_mvW1);
	CNF::Var E_now_C1R_mvW0  = var( G.east(v),    t,    R_Move::w1_mvW0);
	CNF::Var E_now_C2R_ready = var( G.east(v),    t,    NdStat::C2R_ready);
	CNF::Var E_now_C2R_accE  = var( G.east(v),    t,    R_Move::w2_accE);
	CNF::Var E_now_C2R_mvE1  = var( G.east(v),    t,    R_Move::w2_mvE1);
	CNF::Var E_now_C2R_mvE0  = var( G.east(v),    t,    R_Move::w2_mvE0);
	CNF::Var E_now_C2R_accW  = var( G.east(v),    t,    R_Move::w2_accW);
	CNF::Var E_now_C2R_mvW1  = var( G.east(v),    t,    R_Move::w2_mvW1);
	CNF::Var E_now_C2R_mvW0  = var( G.east(v),    t,    R_Move::w2_mvW0);

        // Abbreviations
	CNF::Var E_now_CR_mvE1;
	// neccessarly
	c = not(E_now_CR_mvE1)  or E_now_C0R_mvE1      or E_now_C1R_mvE1      or E_now_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_mvE1) or E_now_C1R_mvE1      or E_now_C2R_mvE1      or E_now_CR_mvE1;
	m.addClause(c);
	c = E_now_C0R_mvE1      or not(E_now_C1R_mvE1) or E_now_C2R_mvE1      or E_now_CR_mvE1;
	m.addClause(c);
	c = E_now_C0R_mvE1      or E_now_C1R_mvE1      or not(E_now_C2R_mvE1) or E_now_CR_mvE1;
	m.addClause(c);
	c = not(E_now_C0R_mvE1) or not(E_now_C1R_mvE1);
	m.addClause(c);
	c = not(E_now_C0R_mvE1) or not(E_now_C2R_mvE1);
	m.addClause(c);
	c = not(E_now_C1R_mvE1) or not(E_now_C2R_mvE1);
	m.addClause(c);
	CNF::Var E_now_CR_mvE0;
	// neccessarly
	c = not(E_now_CR_mvE0)  or E_now_C0R_mvE0      or E_now_C1R_mvE0      or E_now_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_mvE0) or E_now_C1R_mvE0      or E_now_C2R_mvE0      or E_now_CR_mvE0;
	m.addClause(c);
	c = E_now_C0R_mvE0      or not(E_now_C1R_mvE0) or E_now_C2R_mvE0      or E_now_CR_mvE0;
	m.addClause(c);
	c = E_now_C0R_mvE0      or E_now_C1R_mvE0      or not(E_now_C2R_mvE0) or E_now_CR_mvE0;
	m.addClause(c);
	c = not(E_now_C0R_mvE0) or not(E_now_C1R_mvE0);
	m.addClause(c);
	c = not(E_now_C0R_mvE0) or not(E_now_C2R_mvE0);
	m.addClause(c);
	c = not(E_now_C1R_mvE0) or not(E_now_C2R_mvE0);
	m.addClause(c);

	CNF::Var E_now_CR_accW;
	// neccessarly
	c = not(E_now_CR_accW)  or E_now_C0R_accW      or E_now_C1R_accW      or E_now_C2R_accW;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_accW) or E_now_C1R_accW      or E_now_C2R_accW      or E_now_CR_accW;
	m.addClause(c);
	c = E_now_C0R_accW      or not(E_now_C1R_accW) or E_now_C2R_accW      or E_now_CR_accW;
	m.addClause(c);
	c = E_now_C0R_accW      or E_now_C1R_accW      or not(E_now_C2R_accW) or E_now_CR_accW;
	m.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C1R_accW);
	m.addClause(c);
	c = not(E_now_C0R_accW) or not(E_now_C2R_accW);
	m.addClause(c);
	c = not(E_now_C1R_accW) or not(E_now_C2R_accW);
	m.addClause(c);
	CNF::Var E_now_CR_mvW1;
	// neccessarly
	c = not(E_now_CR_mvW1)  or E_now_C0R_mvW1      or E_now_C1R_mvW1      or E_now_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_mvW1) or E_now_C1R_mvW1      or E_now_C2R_mvW1      or E_now_CR_mvW1;
	m.addClause(c);
	c = E_now_C0R_mvW1      or not(E_now_C1R_mvW1) or E_now_C2R_mvW1      or E_now_CR_mvW1;
	m.addClause(c);
	c = E_now_C0R_mvW1      or E_now_C1R_mvW1      or not(E_now_C2R_mvW1) or E_now_CR_mvW1;
	m.addClause(c);
	c = not(E_now_C0R_mvW1) or not(E_now_C1R_mvW1);
	m.addClause(c);
	c = not(E_now_C0R_mvW1) or not(E_now_C2R_mvW1);
	m.addClause(c);
	c = not(E_now_C1R_mvW1) or not(E_now_C2R_mvW1);
	m.addClause(c);
	CNF::Var E_now_CR_mvW0;
	// neccessarly
	c = not(E_now_CR_mvW0)  or E_now_C0R_mvW0      or E_now_C1R_mvW0      or E_now_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(E_now_C0R_mvW0) or E_now_C1R_mvW0      or E_now_C2R_mvW0      or E_now_CR_mvW0;
	m.addClause(c);
	c = E_now_C0R_mvW0      or not(E_now_C1R_mvW0) or E_now_C2R_mvW0      or E_now_CR_mvW0;
	m.addClause(c);
	c = E_now_C0R_mvW0      or E_now_C1R_mvW0      or not(E_now_C2R_mvW0) or E_now_CR_mvW0;
	m.addClause(c);
	c = not(E_now_C0R_mvW0) or not(E_now_C1R_mvW0);
	m.addClause(c);
	c = not(E_now_C0R_mvW0) or not(E_now_C2R_mvW0);
	m.addClause(c);
	c = not(E_now_C1R_mvW0) or not(E_now_C2R_mvW0);
	m.addClause(c);


	CNF::Var N_now_nobodyhome= var( G.north(v),    t,    NdStat::nobodyhome);
	CNF::Var N_now_R_ready   = var( G.north(v),    t,    NdStat::R_ready);
	CNF::Var N_now_R_accN    = var( G.north(v),    t,    R_Move::accN);
	CNF::Var N_now_R_mvN1    = var( G.north(v),    t,    R_Move::mvN1);
	CNF::Var N_now_R_mvN0    = var( G.north(v),    t,    R_Move::mvN0);
	CNF::Var N_now_R_accS    = var( G.north(v),    t,    R_Move::accS);
	CNF::Var N_now_R_mvS1    = var( G.north(v),    t,    R_Move::mvS1);
	CNF::Var N_now_R_mvS0    = var( G.north(v),    t,    R_Move::mvS0);
	CNF::Var N_now_C0R_ready = var( G.north(v),    t,    NdStat::C0R_ready);
	CNF::Var N_now_C0R_accN  = var( G.north(v),    t,    R_Move::w0_accN);
	CNF::Var N_now_C0R_mvN1  = var( G.north(v),    t,    R_Move::w0_mvN1);
	CNF::Var N_now_C0R_mvN2  = var( G.north(v),    t,    R_Move::w0_mvN2);
	CNF::Var N_now_C0R_mvN3  = var( G.north(v),    t,    R_Move::w0_mvN3);
	CNF::Var N_now_C0R_mvN0  = var( G.north(v),    t,    R_Move::w0_mvN0);
	CNF::Var N_now_C0R_accS  = var( G.north(v),    t,    R_Move::w0_accS);
	CNF::Var N_now_C0R_mvS1  = var( G.north(v),    t,    R_Move::w0_mvS1);
	CNF::Var N_now_C0R_mvS2  = var( G.north(v),    t,    R_Move::w0_mvS2);
	CNF::Var N_now_C0R_mvS3  = var( G.north(v),    t,    R_Move::w0_mvS3);
	CNF::Var N_now_C0R_mvS0  = var( G.north(v),    t,    R_Move::w0_mvS0);
	CNF::Var N_now_C1R_ready = var( G.north(v),    t,    NdStat::C1R_ready);
	CNF::Var N_now_C1R_accN  = var( G.north(v),    t,    R_Move::w1_accN);
	CNF::Var N_now_C1R_mvN1  = var( G.north(v),    t,    R_Move::w1_mvN1);
	CNF::Var N_now_C1R_mvN2  = var( G.north(v),    t,    R_Move::w1_mvN2);
	CNF::Var N_now_C1R_mvN3  = var( G.north(v),    t,    R_Move::w1_mvN3);
	CNF::Var N_now_C1R_mvN0  = var( G.north(v),    t,    R_Move::w1_mvN0);
	CNF::Var N_now_C1R_accS  = var( G.north(v),    t,    R_Move::w1_accS);
	CNF::Var N_now_C1R_mvS1  = var( G.north(v),    t,    R_Move::w1_mvS1);
	CNF::Var N_now_C1R_mvS2  = var( G.north(v),    t,    R_Move::w1_mvS2);
	CNF::Var N_now_C1R_mvS3  = var( G.north(v),    t,    R_Move::w1_mvS3);
	CNF::Var N_now_C1R_mvS0  = var( G.north(v),    t,    R_Move::w1_mvS0);
	CNF::Var N_now_C2R_ready = var( G.north(v),    t,    NdStat::C2R_ready);
	CNF::Var N_now_C2R_accN  = var( G.north(v),    t,    R_Move::w2_accN);
	CNF::Var N_now_C2R_mvN1  = var( G.north(v),    t,    R_Move::w2_mvN1);
	CNF::Var N_now_C2R_mvN2  = var( G.north(v),    t,    R_Move::w2_mvN2);
	CNF::Var N_now_C2R_mvN3  = var( G.north(v),    t,    R_Move::w2_mvN3);
	CNF::Var N_now_C2R_mvN0  = var( G.north(v),    t,    R_Move::w2_mvN0);
	CNF::Var N_now_C2R_accS  = var( G.north(v),    t,    R_Move::w2_accS);
	CNF::Var N_now_C2R_mvS1  = var( G.north(v),    t,    R_Move::w2_mvS1);
	CNF::Var N_now_C2R_mvS2  = var( G.north(v),    t,    R_Move::w2_mvS2);
	CNF::Var N_now_C2R_mvS3  = var( G.north(v),    t,    R_Move::w2_mvS3);
	CNF::Var N_now_C2R_mvS0  = var( G.north(v),    t,    R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var N_now_CR_accN;
	// neccessarly
	c = not(N_now_CR_accN)  or N_now_C0R_accN      or N_now_C1R_accN      or N_now_C2R_accN;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_accN) or N_now_C1R_accN      or N_now_C2R_accN      or N_now_CR_accN;
	m.addClause(c);
	c = N_now_C0R_accN      or not(N_now_C1R_accN) or N_now_C2R_accN      or N_now_CR_accN;
	m.addClause(c);
	c = N_now_C0R_accN      or N_now_C1R_accN      or not(N_now_C2R_accN) or N_now_CR_accN;
	m.addClause(c);
	c = not(N_now_C0R_accN) or not(N_now_C1R_accN);
	m.addClause(c);
	c = not(N_now_C0R_accN) or not(N_now_C2R_accN);
	m.addClause(c);
	c = not(N_now_C1R_accN) or not(N_now_C2R_accN);
	m.addClause(c);
	CNF::Var N_now_CR_mvN1;
	// neccessarly
	c = not(N_now_CR_mvN1)  or N_now_C0R_mvN1      or N_now_C1R_mvN1      or N_now_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvN1) or N_now_C1R_mvN1      or N_now_C2R_mvN1      or N_now_CR_mvN1;
	m.addClause(c);
	c = N_now_C0R_mvN1      or not(N_now_C1R_mvN1) or N_now_C2R_mvN1      or N_now_CR_mvN1;
	m.addClause(c);
	c = N_now_C0R_mvN1      or N_now_C1R_mvN1      or not(N_now_C2R_mvN1) or N_now_CR_mvN1;
	m.addClause(c);
	c = not(N_now_C0R_mvN1) or not(N_now_C1R_mvN1);
	m.addClause(c);
	c = not(N_now_C0R_mvN1) or not(N_now_C2R_mvN1);
	m.addClause(c);
	c = not(N_now_C1R_mvN1) or not(N_now_C2R_mvN1);
	m.addClause(c);
	CNF::Var N_now_CR_mvN2;
	// neccessarly
	c = not(N_now_CR_mvN2)  or N_now_C0R_mvN2      or N_now_C1R_mvN2      or N_now_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvN2) or N_now_C1R_mvN2      or N_now_C2R_mvN2      or N_now_CR_mvN2;
	m.addClause(c);
	c = N_now_C0R_mvN2      or not(N_now_C1R_mvN2) or N_now_C2R_mvN2      or N_now_CR_mvN2;
	m.addClause(c);
	c = N_now_C0R_mvN2      or N_now_C1R_mvN2      or not(N_now_C2R_mvN2) or N_now_CR_mvN2;
	m.addClause(c);
	c = not(N_now_C0R_mvN2) or not(N_now_C1R_mvN2);
	m.addClause(c);
	c = not(N_now_C0R_mvN2) or not(N_now_C2R_mvN2);
	m.addClause(c);
	c = not(N_now_C1R_mvN2) or not(N_now_C2R_mvN2);
	m.addClause(c);
	CNF::Var N_now_CR_mvN3;
	// neccessarly
	c = not(N_now_CR_mvN3)  or N_now_C0R_mvN3      or N_now_C1R_mvN3      or N_now_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvN3) or N_now_C1R_mvN3      or N_now_C2R_mvN3      or N_now_CR_mvN3;
	m.addClause(c);
	c = N_now_C0R_mvN3      or not(N_now_C1R_mvN3) or N_now_C2R_mvN3      or N_now_CR_mvN3;
	m.addClause(c);
	c = N_now_C0R_mvN3      or N_now_C1R_mvN3      or not(N_now_C2R_mvN3) or N_now_CR_mvN3;
	m.addClause(c);
	c = not(N_now_C0R_mvN3) or not(N_now_C1R_mvN3);
	m.addClause(c);
	c = not(N_now_C0R_mvN3) or not(N_now_C2R_mvN3);
	m.addClause(c);
	c = not(N_now_C1R_mvN3) or not(N_now_C2R_mvN3);
	m.addClause(c);
	CNF::Var N_now_CR_mvN0;
	// neccessarly
	c = not(N_now_CR_mvN0)  or N_now_C0R_mvN0      or N_now_C1R_mvN0      or N_now_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvN0) or N_now_C1R_mvN0      or N_now_C2R_mvN0      or N_now_CR_mvN0;
	m.addClause(c);
	c = N_now_C0R_mvN0      or not(N_now_C1R_mvN0) or N_now_C2R_mvN0      or N_now_CR_mvN0;
	m.addClause(c);
	c = N_now_C0R_mvN0      or N_now_C1R_mvN0      or not(N_now_C2R_mvN0) or N_now_CR_mvN0;
	m.addClause(c);
	c = not(N_now_C0R_mvN0) or not(N_now_C1R_mvN0);
	m.addClause(c);
	c = not(N_now_C0R_mvN0) or not(N_now_C2R_mvN0);
	m.addClause(c);
	c = not(N_now_C1R_mvN0) or not(N_now_C2R_mvN0);
	m.addClause(c);

	CNF::Var N_now_CR_accS;
	// neccessarly
	c = not(N_now_CR_accS)  or N_now_C0R_accS      or N_now_C1R_accS      or N_now_C2R_accS;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_accS) or N_now_C1R_accS      or N_now_C2R_accS      or N_now_CR_accS;
	m.addClause(c);
	c = N_now_C0R_accS      or not(N_now_C1R_accS) or N_now_C2R_accS      or N_now_CR_accS;
	m.addClause(c);
	c = N_now_C0R_accS      or N_now_C1R_accS      or not(N_now_C2R_accS) or N_now_CR_accS;
	m.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C1R_accS);
	m.addClause(c);
	c = not(N_now_C0R_accS) or not(N_now_C2R_accS);
	m.addClause(c);
	c = not(N_now_C1R_accS) or not(N_now_C2R_accS);
	m.addClause(c);
	CNF::Var N_now_CR_mvS1;
	// neccessarly
	c = not(N_now_CR_mvS1)  or N_now_C0R_mvS1      or N_now_C1R_mvS1      or N_now_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvS1) or N_now_C1R_mvS1      or N_now_C2R_mvS1      or N_now_CR_mvS1;
	m.addClause(c);
	c = N_now_C0R_mvS1      or not(N_now_C1R_mvS1) or N_now_C2R_mvS1      or N_now_CR_mvS1;
	m.addClause(c);
	c = N_now_C0R_mvS1      or N_now_C1R_mvS1      or not(N_now_C2R_mvS1) or N_now_CR_mvS1;
	m.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C1R_mvS1);
	m.addClause(c);
	c = not(N_now_C0R_mvS1) or not(N_now_C2R_mvS1);
	m.addClause(c);
	c = not(N_now_C1R_mvS1) or not(N_now_C2R_mvS1);
	m.addClause(c);
	CNF::Var N_now_CR_mvS2;
	// neccessarly
	c = not(N_now_CR_mvS2)  or N_now_C0R_mvS2      or N_now_C1R_mvS2      or N_now_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvS2) or N_now_C1R_mvS2      or N_now_C2R_mvS2      or N_now_CR_mvS2;
	m.addClause(c);
	c = N_now_C0R_mvS2      or not(N_now_C1R_mvS2) or N_now_C2R_mvS2      or N_now_CR_mvS2;
	m.addClause(c);
	c = N_now_C0R_mvS2      or N_now_C1R_mvS2      or not(N_now_C2R_mvS2) or N_now_CR_mvS2;
	m.addClause(c);
	c = not(N_now_C0R_mvS2) or not(N_now_C1R_mvS2);
	m.addClause(c);
	c = not(N_now_C0R_mvS2) or not(N_now_C2R_mvS2);
	m.addClause(c);
	c = not(N_now_C1R_mvS2) or not(N_now_C2R_mvS2);
	m.addClause(c);
	CNF::Var N_now_CR_mvS3;
	// neccessarly
	c = not(N_now_CR_mvS3)  or N_now_C0R_mvS3      or N_now_C1R_mvS3      or N_now_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvS3) or N_now_C1R_mvS3      or N_now_C2R_mvS3      or N_now_CR_mvS3;
	m.addClause(c);
	c = N_now_C0R_mvS3      or not(N_now_C1R_mvS3) or N_now_C2R_mvS3      or N_now_CR_mvS3;
	m.addClause(c);
	c = N_now_C0R_mvS3      or N_now_C1R_mvS3      or not(N_now_C2R_mvS3) or N_now_CR_mvS3;
	m.addClause(c);
	c = not(N_now_C0R_mvS3) or not(N_now_C1R_mvS3);
	m.addClause(c);
	c = not(N_now_C0R_mvS3) or not(N_now_C2R_mvS3);
	m.addClause(c);
	c = not(N_now_C1R_mvS3) or not(N_now_C2R_mvS3);
	m.addClause(c);
	CNF::Var N_now_CR_mvS0;
	// neccessarly
	c = not(N_now_CR_mvS0)  or N_now_C0R_mvS0      or N_now_C1R_mvS0      or N_now_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(N_now_C0R_mvS0) or N_now_C1R_mvS0      or N_now_C2R_mvS0      or N_now_CR_mvS0;
	m.addClause(c);
	c = N_now_C0R_mvS0      or not(N_now_C1R_mvS0) or N_now_C2R_mvS0      or N_now_CR_mvS0;
	m.addClause(c);
	c = N_now_C0R_mvS0      or N_now_C1R_mvS0      or not(N_now_C2R_mvS0) or N_now_CR_mvS0;
	m.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C1R_mvS0);
	m.addClause(c);
	c = not(N_now_C0R_mvS0) or not(N_now_C2R_mvS0);
	m.addClause(c);
	c = not(N_now_C1R_mvS0) or not(N_now_C2R_mvS0);
	m.addClause(c);


	CNF::Var W_now_nobodyhome= var( G.west(v),    t,    NdStat::nobodyhome);
	CNF::Var W_now_R_ready   = var( G.west(v),    t,    NdStat::R_ready);
	CNF::Var W_now_R_accE    = var( G.west(v),    t,    R_Move::accE);
	CNF::Var W_now_R_mvE0    = var( G.west(v),    t,    R_Move::mvE0);
	CNF::Var W_now_R_accW    = var( G.west(v),    t,    R_Move::accW);
	CNF::Var W_now_R_mvW0    = var( G.west(v),    t,    R_Move::mvW0);
	CNF::Var W_now_C0R_ready = var( G.west(v),    t,    NdStat::C0R_ready);
	CNF::Var W_now_C0R_accE  = var( G.west(v),    t,    R_Move::w0_accE);
	CNF::Var W_now_C0R_mvE1  = var( G.west(v),    t,    R_Move::w0_mvE1);
	CNF::Var W_now_C0R_mvE0  = var( G.west(v),    t,    R_Move::w0_mvE0);
	CNF::Var W_now_C0R_accW  = var( G.west(v),    t,    R_Move::w0_accW);
	CNF::Var W_now_C0R_mvW1  = var( G.west(v),    t,    R_Move::w0_mvW1);
	CNF::Var W_now_C0R_mvW0  = var( G.west(v),    t,    R_Move::w0_mvW0);
	CNF::Var W_now_C1R_ready = var( G.west(v),    t,    NdStat::C1R_ready);
	CNF::Var W_now_C1R_accE  = var( G.west(v),    t,    R_Move::w1_accE);
	CNF::Var W_now_C1R_mvE1  = var( G.west(v),    t,    R_Move::w1_mvE1);
	CNF::Var W_now_C1R_mvE0  = var( G.west(v),    t,    R_Move::w1_mvE0);
	CNF::Var W_now_C1R_accW  = var( G.west(v),    t,    R_Move::w1_accW);
	CNF::Var W_now_C1R_mvW1  = var( G.west(v),    t,    R_Move::w1_mvW1);
	CNF::Var W_now_C1R_mvW0  = var( G.west(v),    t,    R_Move::w1_mvW0);
	CNF::Var W_now_C2R_ready = var( G.west(v),    t,    NdStat::C2R_ready);
	CNF::Var W_now_C2R_accE  = var( G.west(v),    t,    R_Move::w2_accE);
	CNF::Var W_now_C2R_mvE1  = var( G.west(v),    t,    R_Move::w2_mvE1);
	CNF::Var W_now_C2R_mvE0  = var( G.west(v),    t,    R_Move::w2_mvE0);
	CNF::Var W_now_C2R_accW  = var( G.west(v),    t,    R_Move::w2_accW);
	CNF::Var W_now_C2R_mvW1  = var( G.west(v),    t,    R_Move::w2_mvW1);
	CNF::Var W_now_C2R_mvW0  = var( G.west(v),    t,    R_Move::w2_mvW0);

        // Abbreviations
	CNF::Var W_now_CR_accE;
	// neccessarly
	c = not(W_now_CR_accE)  or W_now_C0R_accE      or W_now_C1R_accE      or W_now_C2R_accE;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_accE) or W_now_C1R_accE      or W_now_C2R_accE      or W_now_CR_accE;
	m.addClause(c);
	c = W_now_C0R_accE      or not(W_now_C1R_accE) or W_now_C2R_accE      or W_now_CR_accE;
	m.addClause(c);
	c = W_now_C0R_accE      or W_now_C1R_accE      or not(W_now_C2R_accE) or W_now_CR_accE;
	m.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C1R_accE);
	m.addClause(c);
	c = not(W_now_C0R_accE) or not(W_now_C2R_accE);
	m.addClause(c);
	c = not(W_now_C1R_accE) or not(W_now_C2R_accE);
	m.addClause(c);
	CNF::Var W_now_CR_mvE1;
	// neccessarly
	c = not(W_now_CR_mvE1)  or W_now_C0R_mvE1      or W_now_C1R_mvE1      or W_now_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_mvE1) or W_now_C1R_mvE1      or W_now_C2R_mvE1      or W_now_CR_mvE1;
	m.addClause(c);
	c = W_now_C0R_mvE1      or not(W_now_C1R_mvE1) or W_now_C2R_mvE1      or W_now_CR_mvE1;
	m.addClause(c);
	c = W_now_C0R_mvE1      or W_now_C1R_mvE1      or not(W_now_C2R_mvE1) or W_now_CR_mvE1;
	m.addClause(c);
	c = not(W_now_C0R_mvE1) or not(W_now_C1R_mvE1);
	m.addClause(c);
	c = not(W_now_C0R_mvE1) or not(W_now_C2R_mvE1);
	m.addClause(c);
	c = not(W_now_C1R_mvE1) or not(W_now_C2R_mvE1);
	m.addClause(c);
	CNF::Var W_now_CR_mvE0;
	// neccessarly
	c = not(W_now_CR_mvE0)  or W_now_C0R_mvE0      or W_now_C1R_mvE0      or W_now_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_mvE0) or W_now_C1R_mvE0      or W_now_C2R_mvE0      or W_now_CR_mvE0;
	m.addClause(c);
	c = W_now_C0R_mvE0      or not(W_now_C1R_mvE0) or W_now_C2R_mvE0      or W_now_CR_mvE0;
	m.addClause(c);
	c = W_now_C0R_mvE0      or W_now_C1R_mvE0      or not(W_now_C2R_mvE0) or W_now_CR_mvE0;
	m.addClause(c);
	c = not(W_now_C0R_mvE0) or not(W_now_C1R_mvE0);
	m.addClause(c);
	c = not(W_now_C0R_mvE0) or not(W_now_C2R_mvE0);
	m.addClause(c);
	c = not(W_now_C1R_mvE0) or not(W_now_C2R_mvE0);
	m.addClause(c);

	CNF::Var W_now_CR_mvW1;
	// neccessarly
	c = not(W_now_CR_mvW1)  or W_now_C0R_mvW1      or W_now_C1R_mvW1      or W_now_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_mvW1) or W_now_C1R_mvW1      or W_now_C2R_mvW1      or W_now_CR_mvW1;
	m.addClause(c);
	c = W_now_C0R_mvW1      or not(W_now_C1R_mvW1) or W_now_C2R_mvW1      or W_now_CR_mvW1;
	m.addClause(c);
	c = W_now_C0R_mvW1      or W_now_C1R_mvW1      or not(W_now_C2R_mvW1) or W_now_CR_mvW1;
	m.addClause(c);
	c = not(W_now_C0R_mvW1) or not(W_now_C1R_mvW1);
	m.addClause(c);
	c = not(W_now_C0R_mvW1) or not(W_now_C2R_mvW1);
	m.addClause(c);
	c = not(W_now_C1R_mvW1) or not(W_now_C2R_mvW1);
	m.addClause(c);
	CNF::Var W_now_CR_mvW0;
	// neccessarly
	c = not(W_now_CR_mvW0)  or W_now_C0R_mvW0      or W_now_C1R_mvW0      or W_now_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(W_now_C0R_mvW0) or W_now_C1R_mvW0      or W_now_C2R_mvW0      or W_now_CR_mvW0;
	m.addClause(c);
	c = W_now_C0R_mvW0      or not(W_now_C1R_mvW0) or W_now_C2R_mvW0      or W_now_CR_mvW0;
	m.addClause(c);
	c = W_now_C0R_mvW0      or W_now_C1R_mvW0      or not(W_now_C2R_mvW0) or W_now_CR_mvW0;
	m.addClause(c);
	c = not(W_now_C0R_mvW0) or not(W_now_C1R_mvW0);
	m.addClause(c);
	c = not(W_now_C0R_mvW0) or not(W_now_C2R_mvW0);
	m.addClause(c);
	c = not(W_now_C1R_mvW0) or not(W_now_C2R_mvW0);
	m.addClause(c);


	CNF::Var S_now_nobodyhome= var( G.south(v),    t,    NdStat::nobodyhome);
	CNF::Var S_now_R_ready   = var( G.south(v),    t,    NdStat::R_ready);
	CNF::Var S_now_R_accN    = var( G.south(v),    t,    R_Move::accN);
	CNF::Var S_now_R_mvN1    = var( G.south(v),    t,    R_Move::mvN1);
	CNF::Var S_now_R_mvN0    = var( G.south(v),    t,    R_Move::mvN0);
	CNF::Var S_now_R_accS    = var( G.south(v),    t,    R_Move::accS);
	CNF::Var S_now_R_mvS1    = var( G.south(v),    t,    R_Move::mvS1);
	CNF::Var S_now_R_mvS0    = var( G.south(v),    t,    R_Move::mvS0);
	CNF::Var S_now_C0R_ready = var( G.south(v),    t,    NdStat::C0R_ready);
	CNF::Var S_now_C0R_accN  = var( G.south(v),    t,    R_Move::w0_accN);
	CNF::Var S_now_C0R_mvN1  = var( G.south(v),    t,    R_Move::w0_mvN1);
	CNF::Var S_now_C0R_mvN2  = var( G.south(v),    t,    R_Move::w0_mvN2);
	CNF::Var S_now_C0R_mvN3  = var( G.south(v),    t,    R_Move::w0_mvN3);
	CNF::Var S_now_C0R_mvN0  = var( G.south(v),    t,    R_Move::w0_mvN0);
	CNF::Var S_now_C0R_accS  = var( G.south(v),    t,    R_Move::w0_accS);
	CNF::Var S_now_C0R_mvS1  = var( G.south(v),    t,    R_Move::w0_mvS1);
	CNF::Var S_now_C0R_mvS2  = var( G.south(v),    t,    R_Move::w0_mvS2);
	CNF::Var S_now_C0R_mvS3  = var( G.south(v),    t,    R_Move::w0_mvS3);
	CNF::Var S_now_C0R_mvS0  = var( G.south(v),    t,    R_Move::w0_mvS0);
	CNF::Var S_now_C1R_ready = var( G.south(v),    t,    NdStat::C1R_ready);
	CNF::Var S_now_C1R_accN  = var( G.south(v),    t,    R_Move::w1_accN);
	CNF::Var S_now_C1R_mvN1  = var( G.south(v),    t,    R_Move::w1_mvN1);
	CNF::Var S_now_C1R_mvN2  = var( G.south(v),    t,    R_Move::w1_mvN2);
	CNF::Var S_now_C1R_mvN3  = var( G.south(v),    t,    R_Move::w1_mvN3);
	CNF::Var S_now_C1R_mvN0  = var( G.south(v),    t,    R_Move::w1_mvN0);
	CNF::Var S_now_C1R_accS  = var( G.south(v),    t,    R_Move::w1_accS);
	CNF::Var S_now_C1R_mvS1  = var( G.south(v),    t,    R_Move::w1_mvS1);
	CNF::Var S_now_C1R_mvS2  = var( G.south(v),    t,    R_Move::w1_mvS2);
	CNF::Var S_now_C1R_mvS3  = var( G.south(v),    t,    R_Move::w1_mvS3);
	CNF::Var S_now_C1R_mvS0  = var( G.south(v),    t,    R_Move::w1_mvS0);
	CNF::Var S_now_C2R_ready = var( G.south(v),    t,    NdStat::C2R_ready);
	CNF::Var S_now_C2R_accN  = var( G.south(v),    t,    R_Move::w2_accN);
	CNF::Var S_now_C2R_mvN1  = var( G.south(v),    t,    R_Move::w2_mvN1);
	CNF::Var S_now_C2R_mvN2  = var( G.south(v),    t,    R_Move::w2_mvN2);
	CNF::Var S_now_C2R_mvN3  = var( G.south(v),    t,    R_Move::w2_mvN3);
	CNF::Var S_now_C2R_mvN0  = var( G.south(v),    t,    R_Move::w2_mvN0);
	CNF::Var S_now_C2R_accS  = var( G.south(v),    t,    R_Move::w2_accS);
	CNF::Var S_now_C2R_mvS1  = var( G.south(v),    t,    R_Move::w2_mvS1);
	CNF::Var S_now_C2R_mvS2  = var( G.south(v),    t,    R_Move::w2_mvS2);
	CNF::Var S_now_C2R_mvS3  = var( G.south(v),    t,    R_Move::w2_mvS3);
	CNF::Var S_now_C2R_mvS0  = var( G.south(v),    t,    R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var S_now_CR_accN;
	// neccessarly
	c = not(S_now_CR_accN)  or S_now_C0R_accN      or S_now_C1R_accN      or S_now_C2R_accN;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_accN) or S_now_C1R_accN      or S_now_C2R_accN      or S_now_CR_accN;
	m.addClause(c);
	c = S_now_C0R_accN      or not(S_now_C1R_accN) or S_now_C2R_accN      or S_now_CR_accN;
	m.addClause(c);
	c = S_now_C0R_accN      or S_now_C1R_accN      or not(S_now_C2R_accN) or S_now_CR_accN;
	m.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C1R_accN);
	m.addClause(c);
	c = not(S_now_C0R_accN) or not(S_now_C2R_accN);
	m.addClause(c);
	c = not(S_now_C1R_accN) or not(S_now_C2R_accN);
	m.addClause(c);
	CNF::Var S_now_CR_mvN1;
	// neccessarly
	c = not(S_now_CR_mvN1)  or S_now_C0R_mvN1      or S_now_C1R_mvN1      or S_now_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvN1) or S_now_C1R_mvN1      or S_now_C2R_mvN1      or S_now_CR_mvN1;
	m.addClause(c);
	c = S_now_C0R_mvN1      or not(S_now_C1R_mvN1) or S_now_C2R_mvN1      or S_now_CR_mvN1;
	m.addClause(c);
	c = S_now_C0R_mvN1      or S_now_C1R_mvN1      or not(S_now_C2R_mvN1) or S_now_CR_mvN1;
	m.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C1R_mvN1);
	m.addClause(c);
	c = not(S_now_C0R_mvN1) or not(S_now_C2R_mvN1);
	m.addClause(c);
	c = not(S_now_C1R_mvN1) or not(S_now_C2R_mvN1);
	m.addClause(c);
	CNF::Var S_now_CR_mvN2;
	// neccessarly
	c = not(S_now_CR_mvN2)  or S_now_C0R_mvN2      or S_now_C1R_mvN2      or S_now_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvN2) or S_now_C1R_mvN2      or S_now_C2R_mvN2      or S_now_CR_mvN2;
	m.addClause(c);
	c = S_now_C0R_mvN2      or not(S_now_C1R_mvN2) or S_now_C2R_mvN2      or S_now_CR_mvN2;
	m.addClause(c);
	c = S_now_C0R_mvN2      or S_now_C1R_mvN2      or not(S_now_C2R_mvN2) or S_now_CR_mvN2;
	m.addClause(c);
	c = not(S_now_C0R_mvN2) or not(S_now_C1R_mvN2);
	m.addClause(c);
	c = not(S_now_C0R_mvN2) or not(S_now_C2R_mvN2);
	m.addClause(c);
	c = not(S_now_C1R_mvN2) or not(S_now_C2R_mvN2);
	m.addClause(c);
	CNF::Var S_now_CR_mvN3;
	// neccessarly
	c = not(S_now_CR_mvN3)  or S_now_C0R_mvN3      or S_now_C1R_mvN3      or S_now_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvN3) or S_now_C1R_mvN3      or S_now_C2R_mvN3      or S_now_CR_mvN3;
	m.addClause(c);
	c = S_now_C0R_mvN3      or not(S_now_C1R_mvN3) or S_now_C2R_mvN3      or S_now_CR_mvN3;
	m.addClause(c);
	c = S_now_C0R_mvN3      or S_now_C1R_mvN3      or not(S_now_C2R_mvN3) or S_now_CR_mvN3;
	m.addClause(c);
	c = not(S_now_C0R_mvN3) or not(S_now_C1R_mvN3);
	m.addClause(c);
	c = not(S_now_C0R_mvN3) or not(S_now_C2R_mvN3);
	m.addClause(c);
	c = not(S_now_C1R_mvN3) or not(S_now_C2R_mvN3);
	m.addClause(c);
	CNF::Var S_now_CR_mvN0;
	// neccessarly
	c = not(S_now_CR_mvN0)  or S_now_C0R_mvN0      or S_now_C1R_mvN0      or S_now_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvN0) or S_now_C1R_mvN0      or S_now_C2R_mvN0      or S_now_CR_mvN0;
	m.addClause(c);
	c = S_now_C0R_mvN0      or not(S_now_C1R_mvN0) or S_now_C2R_mvN0      or S_now_CR_mvN0;
	m.addClause(c);
	c = S_now_C0R_mvN0      or S_now_C1R_mvN0      or not(S_now_C2R_mvN0) or S_now_CR_mvN0;
	m.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C1R_mvN0);
	m.addClause(c);
	c = not(S_now_C0R_mvN0) or not(S_now_C2R_mvN0);
	m.addClause(c);
	c = not(S_now_C1R_mvN0) or not(S_now_C2R_mvN0);
	m.addClause(c);

	CNF::Var S_now_CR_mvS1;
	// neccessarly
	c = not(S_now_CR_mvS1)  or S_now_C0R_mvS1      or S_now_C1R_mvS1      or S_now_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvS1) or S_now_C1R_mvS1      or S_now_C2R_mvS1      or S_now_CR_mvS1;
	m.addClause(c);
	c = S_now_C0R_mvS1      or not(S_now_C1R_mvS1) or S_now_C2R_mvS1      or S_now_CR_mvS1;
	m.addClause(c);
	c = S_now_C0R_mvS1      or S_now_C1R_mvS1      or not(S_now_C2R_mvS1) or S_now_CR_mvS1;
	m.addClause(c);
	c = not(S_now_C0R_mvS1) or not(S_now_C1R_mvS1);
	m.addClause(c);
	c = not(S_now_C0R_mvS1) or not(S_now_C2R_mvS1);
	m.addClause(c);
	c = not(S_now_C1R_mvS1) or not(S_now_C2R_mvS1);
	m.addClause(c);
	CNF::Var S_now_CR_mvS2;
	// neccessarly
	c = not(S_now_CR_mvS2)  or S_now_C0R_mvS2      or S_now_C1R_mvS2      or S_now_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvS2) or S_now_C1R_mvS2      or S_now_C2R_mvS2      or S_now_CR_mvS2;
	m.addClause(c);
	c = S_now_C0R_mvS2      or not(S_now_C1R_mvS2) or S_now_C2R_mvS2      or S_now_CR_mvS2;
	m.addClause(c);
	c = S_now_C0R_mvS2      or S_now_C1R_mvS2      or not(S_now_C2R_mvS2) or S_now_CR_mvS2;
	m.addClause(c);
	c = not(S_now_C0R_mvS2) or not(S_now_C1R_mvS2);
	m.addClause(c);
	c = not(S_now_C0R_mvS2) or not(S_now_C2R_mvS2);
	m.addClause(c);
	c = not(S_now_C1R_mvS2) or not(S_now_C2R_mvS2);
	m.addClause(c);
	CNF::Var S_now_CR_mvS3;
	// neccessarly
	c = not(S_now_CR_mvS3)  or S_now_C0R_mvS3      or S_now_C1R_mvS3      or S_now_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvS3) or S_now_C1R_mvS3      or S_now_C2R_mvS3      or S_now_CR_mvS3;
	m.addClause(c);
	c = S_now_C0R_mvS3      or not(S_now_C1R_mvS3) or S_now_C2R_mvS3      or S_now_CR_mvS3;
	m.addClause(c);
	c = S_now_C0R_mvS3      or S_now_C1R_mvS3      or not(S_now_C2R_mvS3) or S_now_CR_mvS3;
	m.addClause(c);
	c = not(S_now_C0R_mvS3) or not(S_now_C1R_mvS3);
	m.addClause(c);
	c = not(S_now_C0R_mvS3) or not(S_now_C2R_mvS3);
	m.addClause(c);
	c = not(S_now_C1R_mvS3) or not(S_now_C2R_mvS3);
	m.addClause(c);
	CNF::Var S_now_CR_mvS0;
	// neccessarly
	c = not(S_now_CR_mvS0)  or S_now_C0R_mvS0      or S_now_C1R_mvS0      or S_now_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(S_now_C0R_mvS0) or S_now_C1R_mvS0      or S_now_C2R_mvS0      or S_now_CR_mvS0;
	m.addClause(c);
	c = S_now_C0R_mvS0      or not(S_now_C1R_mvS0) or S_now_C2R_mvS0      or S_now_CR_mvS0;
	m.addClause(c);
	c = S_now_C0R_mvS0      or S_now_C1R_mvS0      or not(S_now_C2R_mvS0) or S_now_CR_mvS0;
	m.addClause(c);
	c = not(S_now_C0R_mvS0) or not(S_now_C1R_mvS0);
	m.addClause(c);
	c = not(S_now_C0R_mvS0) or not(S_now_C2R_mvS0);
	m.addClause(c);
	c = not(S_now_C1R_mvS0) or not(S_now_C2R_mvS0);
	m.addClause(c);
	
        
        
        // Future
	CNF::Var Here_will_nobodyhome= var(v,    t+1,     NdStat::nobodyhome);
	CNF::Var Here_will_R_ready   = var(v,    t+1,     NdStat::R_ready);
	CNF::Var Here_will_R_accE    = var(v,    t+1,     R_Move::accE);
	CNF::Var Here_will_R_mvE0    = var(v,    t+1,     R_Move::mvE0);
	CNF::Var Here_will_R_accW    = var(v,    t+1,     R_Move::accW);
	CNF::Var Here_will_R_mvW0    = var(v,    t+1,     R_Move::mvW0);
	CNF::Var Here_will_R_accN    = var(v,    t+1,     R_Move::accN);
	CNF::Var Here_will_R_mvN1    = var(v,    t+1,     R_Move::mvN1);
	CNF::Var Here_will_R_mvN0    = var(v,    t+1,     R_Move::mvN0);
	CNF::Var Here_will_R_accS    = var(v,    t+1,     R_Move::accS);
	CNF::Var Here_will_R_mvS1    = var(v,    t+1,     R_Move::mvS1);
	CNF::Var Here_will_R_mvS0    = var(v,    t+1,     R_Move::mvS0);
	CNF::Var Here_will_R_lift    = var(v,    t+1,     R_Vertical::lift);
	CNF::Var Here_will_R_lifting1= var(v,    t+1,     R_Vertical::l1);
	CNF::Var Here_will_R_lifting2= var(v,    t+1,     R_Vertical::l2);
	CNF::Var Here_will_R_lifting3= var(v,    t+1,     R_Vertical::l3);
	CNF::Var Here_will_R_lifting4= var(v,    t+1,     R_Vertical::l4);
	CNF::Var Here_will_R_drop    = var(v,    t+1,     R_Vertical::drop);
	CNF::Var Here_will_C0R_ready = var(v,    t+1,     NdStat::C0R_ready);
	CNF::Var Here_will_C0R_accE  = var(v,    t+1,     R_Move::w0_accE);
	CNF::Var Here_will_C0R_mvE1  = var(v,    t+1,     R_Move::w0_mvE1);
	CNF::Var Here_will_C0R_mvE0  = var(v,    t+1,     R_Move::w0_mvE0);
	CNF::Var Here_will_C0R_accW  = var(v,    t+1,     R_Move::w0_accW);
	CNF::Var Here_will_C0R_mvW1  = var(v,    t+1,     R_Move::w0_mvW1);
	CNF::Var Here_will_C0R_mvW0  = var(v,    t+1,     R_Move::w0_mvW0);
	CNF::Var Here_will_C0R_accN  = var(v,    t+1,     R_Move::w0_accN);
	CNF::Var Here_will_C0R_mvN1  = var(v,    t+1,     R_Move::w0_mvN1);
	CNF::Var Here_will_C0R_mvN2  = var(v,    t+1,     R_Move::w0_mvN2);
	CNF::Var Here_will_C0R_mvN3  = var(v,    t+1,     R_Move::w0_mvN3);
	CNF::Var Here_will_C0R_mvN0  = var(v,    t+1,     R_Move::w0_mvN0);
	CNF::Var Here_will_C0R_accS  = var(v,    t+1,     R_Move::w0_accS);
	CNF::Var Here_will_C0R_mvS1  = var(v,    t+1,     R_Move::w0_mvS1);
	CNF::Var Here_will_C0R_mvS2  = var(v,    t+1,     R_Move::w0_mvS2);
	CNF::Var Here_will_C0R_mvS3  = var(v,    t+1,     R_Move::w0_mvS3);
	CNF::Var Here_will_C0R_mvS0  = var(v,    t+1,     R_Move::w0_mvS0);
	CNF::Var Here_will_C1R_ready = var(v,    t+1,     NdStat::C1R_ready);
	CNF::Var Here_will_C1R_accE  = var(v,    t+1,     R_Move::w1_accE);
	CNF::Var Here_will_C1R_mvE1  = var(v,    t+1,     R_Move::w1_mvE1);
	CNF::Var Here_will_C1R_mvE0  = var(v,    t+1,     R_Move::w1_mvE0);
	CNF::Var Here_will_C1R_accW  = var(v,    t+1,     R_Move::w1_accW);
	CNF::Var Here_will_C1R_mvW1  = var(v,    t+1,     R_Move::w1_mvW1);
	CNF::Var Here_will_C1R_mvW0  = var(v,    t+1,     R_Move::w1_mvW0);
	CNF::Var Here_will_C1R_accN  = var(v,    t+1,     R_Move::w1_accN);
	CNF::Var Here_will_C1R_mvN1  = var(v,    t+1,     R_Move::w1_mvN1);
	CNF::Var Here_will_C1R_mvN2  = var(v,    t+1,     R_Move::w1_mvN2);
	CNF::Var Here_will_C1R_mvN3  = var(v,    t+1,     R_Move::w1_mvN3);
	CNF::Var Here_will_C1R_mvN0  = var(v,    t+1,     R_Move::w1_mvN0);
	CNF::Var Here_will_C1R_accS  = var(v,    t+1,     R_Move::w1_accS);
	CNF::Var Here_will_C1R_mvS1  = var(v,    t+1,     R_Move::w1_mvS1);
	CNF::Var Here_will_C1R_mvS2  = var(v,    t+1,     R_Move::w1_mvS2);
	CNF::Var Here_will_C1R_mvS3  = var(v,    t+1,     R_Move::w1_mvS3);
	CNF::Var Here_will_C1R_mvS0  = var(v,    t+1,     R_Move::w1_mvS0);
	CNF::Var Here_will_C2R_ready = var(v,    t+1,     NdStat::C2R_ready);
	CNF::Var Here_will_C2R_accE  = var(v,    t+1,     R_Move::w2_accE);
	CNF::Var Here_will_C2R_mvE1  = var(v,    t+1,     R_Move::w2_mvE1);
	CNF::Var Here_will_C2R_mvE0  = var(v,    t+1,     R_Move::w2_mvE0);
	CNF::Var Here_will_C2R_accW  = var(v,    t+1,     R_Move::w2_accW);
	CNF::Var Here_will_C2R_mvW1  = var(v,    t+1,     R_Move::w2_mvW1);
	CNF::Var Here_will_C2R_mvW0  = var(v,    t+1,     R_Move::w2_mvW0);
	CNF::Var Here_will_C2R_accN  = var(v,    t+1,     R_Move::w2_accN);
	CNF::Var Here_will_C2R_mvN1  = var(v,    t+1,     R_Move::w2_mvN1);
	CNF::Var Here_will_C2R_mvN2  = var(v,    t+1,     R_Move::w2_mvN2);
	CNF::Var Here_will_C2R_mvN3  = var(v,    t+1,     R_Move::w2_mvN3);
	CNF::Var Here_will_C2R_mvN0  = var(v,    t+1,     R_Move::w2_mvN0);
	CNF::Var Here_will_C2R_accS  = var(v,    t+1,     R_Move::w2_accS);
	CNF::Var Here_will_C2R_mvS1  = var(v,    t+1,     R_Move::w2_mvS1);
	CNF::Var Here_will_C2R_mvS2  = var(v,    t+1,     R_Move::w2_mvS2);
	CNF::Var Here_will_C2R_mvS3  = var(v,    t+1,     R_Move::w2_mvS3);
	CNF::Var Here_will_C2R_mvS0  = var(v,    t+1,     R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var Here_will_CR_mvE1;
	// necessarly
	c = not(Here_will_CR_mvE1)  or Here_will_C0R_mvE1      or Here_will_C1R_mvE1      or Here_will_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvE1) or Here_will_C1R_mvE1      or Here_will_C2R_mvE1      or Here_will_CR_mvE1;
	m.addClause(c);
	c = Here_will_C0R_mvE1      or not(Here_will_C1R_mvE1) or Here_will_C2R_mvE1      or Here_will_CR_mvE1;
	m.addClause(c);
	c = Here_will_C0R_mvE1      or Here_will_C1R_mvE1      or not(Here_will_C2R_mvE1) or Here_will_CR_mvE1;
	m.addClause(c);
	c = not(Here_will_C0R_mvE1) or not(Here_will_C1R_mvE1);
	m.addClause(c);
	c = not(Here_will_C0R_mvE1) or not(Here_will_C2R_mvE1);
	m.addClause(c);
	c = not(Here_will_C1R_mvE1) or not(Here_will_C2R_mvE1);
	m.addClause(c);
	CNF::Var Here_will_CR_mvE0;
	// necessalry
	c = not(Here_will_CR_mvE0)  or Here_will_C0R_mvE0      or Here_will_C1R_mvE0      or Here_will_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvE0) or Here_will_C1R_mvE0      or Here_will_C2R_mvE0      or Here_will_CR_mvE0;
	m.addClause(c);
	c = Here_will_C0R_mvE0      or not(Here_will_C1R_mvE0) or Here_will_C2R_mvE0      or Here_will_CR_mvE0;
	m.addClause(c);
	c = Here_will_C0R_mvE0      or Here_will_C1R_mvE0      or not(Here_will_C2R_mvE0) or Here_will_CR_mvE0;
	m.addClause(c);
	c = not(Here_will_C0R_mvE0) or not(Here_will_C1R_mvE0);
	m.addClause(c);
	c = not(Here_will_C0R_mvE0) or not(Here_will_C2R_mvE0);
	m.addClause(c);
	c = not(Here_will_C1R_mvE0) or not(Here_will_C2R_mvE0);
	m.addClause(c);

	CNF::Var Here_will_CR_mvW1;
	// necessarly
	c = not(Here_will_CR_mvW1)  or Here_will_C0R_mvW1      or Here_will_C1R_mvW1      or Here_will_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvW1) or Here_will_C1R_mvW1      or Here_will_C2R_mvW1      or Here_will_CR_mvW1;
	m.addClause(c);
	c = Here_will_C0R_mvW1      or not(Here_will_C1R_mvW1) or Here_will_C2R_mvW1      or Here_will_CR_mvW1;
	m.addClause(c);
	c = Here_will_C0R_mvW1      or Here_will_C1R_mvW1      or not(Here_will_C2R_mvW1) or Here_will_CR_mvW1;
	m.addClause(c);
	c = not(Here_will_C0R_mvW1) or not(Here_will_C1R_mvW1);
	m.addClause(c);
	c = not(Here_will_C0R_mvW1) or not(Here_will_C2R_mvW1);
	m.addClause(c);
	c = not(Here_will_C1R_mvW1) or not(Here_will_C2R_mvW1);
	m.addClause(c);
	CNF::Var Here_will_CR_mvW0;
	// necessarly
	c = not(Here_will_CR_mvW0)  or Here_will_C0R_mvW0      or Here_will_C1R_mvW0      or Here_will_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvW0) or Here_will_C1R_mvW0      or Here_will_C2R_mvW0      or Here_will_CR_mvW0;
	m.addClause(c);
	c = Here_will_C0R_mvW0      or not(Here_will_C1R_mvW0) or Here_will_C2R_mvW0      or Here_will_CR_mvW0;
	m.addClause(c);
	c = Here_will_C0R_mvW0      or Here_will_C1R_mvW0      or not(Here_will_C2R_mvW0) or Here_will_CR_mvW0;
	m.addClause(c);
	c = not(Here_will_C0R_mvW0) or not(Here_will_C1R_mvW0);
	m.addClause(c);
	c = not(Here_will_C0R_mvW0) or not(Here_will_C2R_mvW0);
	m.addClause(c);
	c = not(Here_will_C1R_mvW0) or not(Here_will_C2R_mvW0);
	m.addClause(c);

	CNF::Var Here_will_CR_mvN1;
	// neccessarly
	c = not(Here_will_CR_mvN1)  or Here_will_C0R_mvN1      or Here_will_C1R_mvN1      or Here_will_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvN1) or Here_will_C1R_mvN1      or Here_will_C2R_mvN1      or Here_will_CR_mvN1;
	m.addClause(c);
	c = Here_will_C0R_mvN1      or not(Here_will_C1R_mvN1) or Here_will_C2R_mvN1      or Here_will_CR_mvN1;
	m.addClause(c);
	c = Here_will_C0R_mvN1      or Here_will_C1R_mvN1      or not(Here_will_C2R_mvN1) or Here_will_CR_mvN1;
	m.addClause(c);
	c = not(Here_will_C0R_mvN1) or not(Here_will_C1R_mvN1);
	m.addClause(c);
	c = not(Here_will_C0R_mvN1) or not(Here_will_C2R_mvN1);
	m.addClause(c);
	c = not(Here_will_C1R_mvN1) or not(Here_will_C2R_mvN1);
	m.addClause(c);
	CNF::Var Here_will_CR_mvN2;
	// neccessarly
	c = not(Here_will_CR_mvN2)  or Here_will_C0R_mvN2      or Here_will_C1R_mvN2       or Here_will_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvN2) or Here_will_C1R_mvN2      or Here_will_C2R_mvN2       or Here_will_CR_mvN2;
	m.addClause(c);
	c = Here_will_C0R_mvN2      or not(Here_will_C1R_mvN2) or Here_will_C2R_mvN2       or Here_will_CR_mvN2;
	m.addClause(c);
	c = Here_will_C0R_mvN2      or Here_will_C1R_mvN2      or not(Here_will_C2R_mvN2)  or Here_will_CR_mvN2;
	m.addClause(c);
	c = not(Here_will_C0R_mvN2) or not(Here_will_C1R_mvN2);
	m.addClause(c);
	c = not(Here_will_C0R_mvN2) or not(Here_will_C2R_mvN2);
	m.addClause(c);
	c = not(Here_will_C1R_mvN2) or not(Here_will_C2R_mvN2);
	m.addClause(c);
	CNF::Var Here_will_CR_mvN3;
	// neccessarly
	c = not(Here_will_CR_mvN3)  or Here_will_C0R_mvN3      or Here_will_C1R_mvN3      or Here_will_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvN3) or Here_will_C1R_mvN3      or Here_will_C2R_mvN3      or Here_will_CR_mvN3;
	m.addClause(c);
	c = Here_will_C0R_mvN3      or not(Here_will_C1R_mvN3) or Here_will_C2R_mvN3      or Here_will_CR_mvN3;
	m.addClause(c);
	c = Here_will_C0R_mvN3      or Here_will_C1R_mvN3      or not(Here_will_C2R_mvN3) or Here_will_CR_mvN3;
	m.addClause(c);
	c = not(Here_will_C0R_mvN3) or not(Here_will_C1R_mvN3);
	m.addClause(c);
	c = not(Here_will_C0R_mvN3) or not(Here_will_C2R_mvN3);
	m.addClause(c);
	c = not(Here_will_C1R_mvN3) or not(Here_will_C2R_mvN3);
	m.addClause(c);
	CNF::Var Here_will_CR_mvN0;
	// neccessarly
	c = not(Here_will_CR_mvN0)  or Here_will_C0R_mvN0      or Here_will_C1R_mvN0      or Here_will_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvN0) or Here_will_C1R_mvN0      or Here_will_C2R_mvN0      or Here_will_CR_mvN0;
	m.addClause(c);
	c = Here_will_C0R_mvN0      or not(Here_will_C1R_mvN0) or Here_will_C2R_mvN0      or Here_will_CR_mvN0;
	m.addClause(c);
	c = Here_will_C0R_mvN0      or Here_will_C1R_mvN0      or not(Here_will_C2R_mvN0) or Here_will_CR_mvN0;
	m.addClause(c);
	c = not(Here_will_C0R_mvN0) or not(Here_will_C1R_mvN0);
	m.addClause(c);
	c = not(Here_will_C0R_mvN0) or not(Here_will_C2R_mvN0);
	m.addClause(c);
	c = not(Here_will_C1R_mvN0) or not(Here_will_C2R_mvN0);
	m.addClause(c);


	CNF::Var Here_will_CR_mvS1;
	// neccessarly
	c = not(Here_will_CR_mvS1)  or Here_will_C0R_mvS1      or Here_will_C1R_mvS1      or Here_will_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvS1) or Here_will_C1R_mvS1      or Here_will_C2R_mvS1      or Here_will_CR_mvS1;
	m.addClause(c);
	c = Here_will_C0R_mvS1      or not(Here_will_C1R_mvS1) or Here_will_C2R_mvS1      or Here_will_CR_mvS1;
	m.addClause(c);
	c = Here_will_C0R_mvS1      or Here_will_C1R_mvS1      or not(Here_will_C2R_mvS1) or Here_will_CR_mvS1;
	m.addClause(c);
	c = not(Here_will_C0R_mvS1) or not(Here_will_C1R_mvS1);
	m.addClause(c);
	c = not(Here_will_C0R_mvS1) or not(Here_will_C2R_mvS1);
	m.addClause(c);
	c = not(Here_will_C1R_mvS1) or not(Here_will_C2R_mvS1);
	m.addClause(c);
	CNF::Var Here_will_CR_mvS2;
	// neccessarly
	c = not(Here_will_CR_mvS2)  or Here_will_C0R_mvS2      or Here_will_C1R_mvS2      or Here_will_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvS2) or Here_will_C1R_mvS2      or Here_will_C2R_mvS2      or Here_will_CR_mvS2;
	m.addClause(c);
	c = Here_will_C0R_mvS2      or not(Here_will_C1R_mvS2) or Here_will_C2R_mvS2      or Here_will_CR_mvS2;
	m.addClause(c);
	c = Here_will_C0R_mvS2      or Here_will_C1R_mvS2      or not(Here_will_C2R_mvS2) or Here_will_CR_mvS2;
	m.addClause(c);
	c = not(Here_will_C0R_mvS2) or not(Here_will_C1R_mvS2);
	m.addClause(c);
	c = not(Here_will_C0R_mvS2) or not(Here_will_C2R_mvS2);
	m.addClause(c);
	c = not(Here_will_C1R_mvS2) or not(Here_will_C2R_mvS2);
	m.addClause(c);
	CNF::Var Here_will_CR_mvS3;
	// neccessarly
	c = not(Here_will_CR_mvS3)  or Here_will_C0R_mvS3      or Here_will_C1R_mvS3      or Here_will_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvS3) or Here_will_C1R_mvS3      or Here_will_C2R_mvS3      or Here_will_CR_mvS3;
	m.addClause(c);
	c = Here_will_C0R_mvS3      or not(Here_will_C1R_mvS3) or Here_will_C2R_mvS3      or Here_will_CR_mvS3;
	m.addClause(c);
	c = Here_will_C0R_mvS3      or Here_will_C1R_mvS3      or not(Here_will_C2R_mvS3) or Here_will_CR_mvS3;
	m.addClause(c);
	c = not(Here_will_C0R_mvS3) or not(Here_will_C1R_mvS3);
	m.addClause(c);
	c = not(Here_will_C0R_mvS3) or not(Here_will_C2R_mvS3);
	m.addClause(c);
	c = not(Here_will_C1R_mvS3) or not(Here_will_C2R_mvS3);
	m.addClause(c);
	CNF::Var Here_will_CR_mvS0;
	// neccessarly
	c = not(Here_will_CR_mvS0)  or Here_will_C0R_mvS0      or Here_will_C1R_mvS0      or Here_will_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_mvS0) or Here_will_C1R_mvS0      or Here_will_C2R_mvS0      or Here_will_CR_mvS0;
	m.addClause(c);
	c = Here_will_C0R_mvS0      or not(Here_will_C1R_mvS0) or Here_will_C2R_mvS0      or Here_will_CR_mvS0;
	m.addClause(c);
	c = Here_will_C0R_mvS0      or Here_will_C1R_mvS0      or not(Here_will_C2R_mvS0) or Here_will_CR_mvS0;
	m.addClause(c);
	c = not(Here_will_C0R_mvS0) or not(Here_will_C1R_mvS0);
	m.addClause(c);
	c = not(Here_will_C0R_mvS0) or not(Here_will_C2R_mvS0);
	m.addClause(c);
	c = not(Here_will_C1R_mvS0) or not(Here_will_C2R_mvS0);
	m.addClause(c);


	CNF::Var E_will_nobodyhome= var( G.east(v),    t+1,    NdStat::nobodyhome);
	CNF::Var E_will_R_ready   = var( G.east(v),    t+1,    NdStat::R_ready);
	CNF::Var E_will_R_accE    = var( G.east(v),    t+1,    R_Move::accE);
	CNF::Var E_will_R_mvE0    = var( G.east(v),    t+1,    R_Move::mvE0);
	CNF::Var E_will_R_accW    = var( G.east(v),    t+1,    R_Move::accW);
	CNF::Var E_will_R_mvW0    = var( G.east(v),    t+1,    R_Move::mvW0);
	CNF::Var E_will_C0R_ready = var( G.east(v),    t+1,    NdStat::C0R_ready);
	CNF::Var E_will_C0R_accE  = var( G.east(v),    t+1,    R_Move::w0_accE);
	CNF::Var E_will_C0R_mvE1  = var( G.east(v),    t+1,    R_Move::w0_mvE1);
	CNF::Var E_will_C0R_mvE0  = var( G.east(v),    t+1,    R_Move::w0_mvE0);
	CNF::Var E_will_C0R_accW  = var( G.east(v),    t+1,    R_Move::w0_accW);
	CNF::Var E_will_C0R_mvW1  = var( G.east(v),    t+1,    R_Move::w0_mvW1);
	CNF::Var E_will_C0R_mvW0  = var( G.east(v),    t+1,    R_Move::w0_mvW0);
	CNF::Var E_will_C1R_ready = var( G.east(v),    t+1,    NdStat::C1R_ready);
	CNF::Var E_will_C1R_accE  = var( G.east(v),    t+1,    R_Move::w1_accE);
	CNF::Var E_will_C1R_mvE1  = var( G.east(v),    t+1,    R_Move::w1_mvE1);
	CNF::Var E_will_C1R_mvE0  = var( G.east(v),    t+1,    R_Move::w1_mvE0);
	CNF::Var E_will_C1R_accW  = var( G.east(v),    t+1,    R_Move::w1_accW);
	CNF::Var E_will_C1R_mvW1  = var( G.east(v),    t+1,    R_Move::w1_mvW1);
	CNF::Var E_will_C1R_mvW0  = var( G.east(v),    t+1,    R_Move::w1_mvW0);
	CNF::Var E_will_C2R_ready = var( G.east(v),    t+1,    NdStat::C2R_ready);
	CNF::Var E_will_C2R_accE  = var( G.east(v),    t+1,    R_Move::w2_accE);
	CNF::Var E_will_C2R_mvE1  = var( G.east(v),    t+1,    R_Move::w2_mvE1);
	CNF::Var E_will_C2R_mvE0  = var( G.east(v),    t+1,    R_Move::w2_mvE0);
	CNF::Var E_will_C2R_accW  = var( G.east(v),    t+1,    R_Move::w2_accW);
	CNF::Var E_will_C2R_mvW1  = var( G.east(v),    t+1,    R_Move::w2_mvW1);
	CNF::Var E_will_C2R_mvW0  = var( G.east(v),    t+1,    R_Move::w2_mvW0);

        // Abbreviations
	CNF::Var E_will_CR_mvE1;
	// neccessarly
	c = not(E_will_CR_mvE1)  or E_will_C0R_mvE1      or E_will_C1R_mvE1      or E_will_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(E_will_C0R_mvE1) or E_will_C1R_mvE1      or E_will_C2R_mvE1      or E_will_CR_mvE1;
	m.addClause(c);
	c = E_will_C0R_mvE1      or not(E_will_C1R_mvE1) or E_will_C2R_mvE1      or E_will_CR_mvE1;
	m.addClause(c);
	c = E_will_C0R_mvE1      or E_will_C1R_mvE1      or not(E_will_C2R_mvE1) or E_will_CR_mvE1;
	m.addClause(c);
	c = not(E_will_C0R_mvE1) or not(E_will_C1R_mvE1);
	m.addClause(c);
	c = not(E_will_C0R_mvE1) or not(E_will_C2R_mvE1);
	m.addClause(c);
	c = not(E_will_C1R_mvE1) or not(E_will_C2R_mvE1);
	m.addClause(c);
	CNF::Var E_will_CR_mvE0;
	// neccessarly
	c = not(E_will_CR_mvE0)  or E_will_C0R_mvE0      or E_will_C1R_mvE0      or E_will_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(E_will_C0R_mvE0) or E_will_C1R_mvE0      or E_will_C2R_mvE0      or E_will_CR_mvE0;
	m.addClause(c);
	c = E_will_C0R_mvE0      or not(E_will_C1R_mvE0) or E_will_C2R_mvE0      or E_will_CR_mvE0;
	m.addClause(c);
	c = E_will_C0R_mvE0      or E_will_C1R_mvE0      or not(E_will_C2R_mvE0) or E_will_CR_mvE0;
	m.addClause(c);
	c = not(E_will_C0R_mvE0) or not(E_will_C1R_mvE0);
	m.addClause(c);
	c = not(E_will_C0R_mvE0) or not(E_will_C2R_mvE0);
	m.addClause(c);
	c = not(E_will_C1R_mvE0) or not(E_will_C2R_mvE0);
	m.addClause(c);

	CNF::Var E_will_CR_accW;
	// neccessarly
	c = not(E_will_CR_accW)  or E_will_C0R_accW      or E_will_C1R_accW      or E_will_C2R_accW;
	m.addClause(c);
	// sufficient
	c = not(E_will_C0R_accW) or E_will_C1R_accW      or E_will_C2R_accW      or E_will_CR_accW;
	m.addClause(c);
	c = E_will_C0R_accW      or not(E_will_C1R_accW) or E_will_C2R_accW      or E_will_CR_accW;
	m.addClause(c);
	c = E_will_C0R_accW      or E_will_C1R_accW      or not(E_will_C2R_accW) or E_will_CR_accW;
	m.addClause(c);
	c = not(E_will_C0R_accW) or not(E_will_C1R_accW);
	m.addClause(c);
	c = not(E_will_C0R_accW) or not(E_will_C2R_accW);
	m.addClause(c);
	c = not(E_will_C1R_accW) or not(E_will_C2R_accW);
	m.addClause(c);
	CNF::Var E_will_CR_mvW1;
	// neccessarly
	c = not(E_will_CR_mvW1)  or E_will_C0R_mvW1      or E_will_C1R_mvW1      or E_will_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(E_will_C0R_mvW1) or E_will_C1R_mvW1      or E_will_C2R_mvW1      or E_will_CR_mvW1;
	m.addClause(c);
	c = E_will_C0R_mvW1      or not(E_will_C1R_mvW1) or E_will_C2R_mvW1      or E_will_CR_mvW1;
	m.addClause(c);
	c = E_will_C0R_mvW1      or E_will_C1R_mvW1      or not(E_will_C2R_mvW1) or E_will_CR_mvW1;
	m.addClause(c);
	c = not(E_will_C0R_mvW1) or not(E_will_C1R_mvW1);
	m.addClause(c);
	c = not(E_will_C0R_mvW1) or not(E_will_C2R_mvW1);
	m.addClause(c);
	c = not(E_will_C1R_mvW1) or not(E_will_C2R_mvW1);
	m.addClause(c);
	CNF::Var E_will_CR_mvW0;
	// neccessarly
	c = not(E_will_CR_mvW0)  or E_will_C0R_mvW0      or E_will_C1R_mvW0      or E_will_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(E_will_C0R_mvW0) or E_will_C1R_mvW0      or E_will_C2R_mvW0      or E_will_CR_mvW0;
	m.addClause(c);
	c = E_will_C0R_mvW0      or not(E_will_C1R_mvW0) or E_will_C2R_mvW0      or E_will_CR_mvW0;
	m.addClause(c);
	c = E_will_C0R_mvW0      or E_will_C1R_mvW0      or not(E_will_C2R_mvW0) or E_will_CR_mvW0;
	m.addClause(c);
	c = not(E_will_C0R_mvW0) or not(E_will_C1R_mvW0);
	m.addClause(c);
	c = not(E_will_C0R_mvW0) or not(E_will_C2R_mvW0);
	m.addClause(c);
	c = not(E_will_C1R_mvW0) or not(E_will_C2R_mvW0);
	m.addClause(c);


	CNF::Var N_will_nobodyhome= var( G.north(v),    t+1,    NdStat::nobodyhome);
	CNF::Var N_will_R_ready   = var( G.north(v),    t+1,    NdStat::R_ready);
	CNF::Var N_will_R_accN    = var( G.north(v),    t+1,    R_Move::accN);
	CNF::Var N_will_R_mvN1    = var( G.north(v),    t+1,    R_Move::mvN1);
	CNF::Var N_will_R_mvN0    = var( G.north(v),    t+1,    R_Move::mvN0);
	CNF::Var N_will_R_accS    = var( G.north(v),    t+1,    R_Move::accS);
	CNF::Var N_will_R_mvS1    = var( G.north(v),    t+1,    R_Move::mvS1);
	CNF::Var N_will_R_mvS0    = var( G.north(v),    t+1,    R_Move::mvS0);
	CNF::Var N_will_C0R_ready = var( G.north(v),    t+1,    NdStat::C0R_ready);
	CNF::Var N_will_C0R_accN  = var( G.north(v),    t+1,    R_Move::w0_accN);
	CNF::Var N_will_C0R_mvN1  = var( G.north(v),    t+1,    R_Move::w0_mvN1);
	CNF::Var N_will_C0R_mvN2  = var( G.north(v),    t+1,    R_Move::w0_mvN2);
	CNF::Var N_will_C0R_mvN3  = var( G.north(v),    t+1,    R_Move::w0_mvN3);
	CNF::Var N_will_C0R_mvN0  = var( G.north(v),    t+1,    R_Move::w0_mvN0);
	CNF::Var N_will_C0R_accS  = var( G.north(v),    t+1,    R_Move::w0_accS);
	CNF::Var N_will_C0R_mvS1  = var( G.north(v),    t+1,    R_Move::w0_mvS1);
	CNF::Var N_will_C0R_mvS2  = var( G.north(v),    t+1,    R_Move::w0_mvS2);
	CNF::Var N_will_C0R_mvS3  = var( G.north(v),    t+1,    R_Move::w0_mvS3);
	CNF::Var N_will_C0R_mvS0  = var( G.north(v),    t+1,    R_Move::w0_mvS0);
	CNF::Var N_will_C1R_ready = var( G.north(v),    t+1,    NdStat::C1R_ready);
	CNF::Var N_will_C1R_accN  = var( G.north(v),    t+1,    R_Move::w1_accN);
	CNF::Var N_will_C1R_mvN1  = var( G.north(v),    t+1,    R_Move::w1_mvN1);
	CNF::Var N_will_C1R_mvN2  = var( G.north(v),    t+1,    R_Move::w1_mvN2);
	CNF::Var N_will_C1R_mvN3  = var( G.north(v),    t+1,    R_Move::w1_mvN3);
	CNF::Var N_will_C1R_mvN0  = var( G.north(v),    t+1,    R_Move::w1_mvN0);
	CNF::Var N_will_C1R_accS  = var( G.north(v),    t+1,    R_Move::w1_accS);
	CNF::Var N_will_C1R_mvS1  = var( G.north(v),    t+1,    R_Move::w1_mvS1);
	CNF::Var N_will_C1R_mvS2  = var( G.north(v),    t+1,    R_Move::w1_mvS2);
	CNF::Var N_will_C1R_mvS3  = var( G.north(v),    t+1,    R_Move::w1_mvS3);
	CNF::Var N_will_C1R_mvS0  = var( G.north(v),    t+1,    R_Move::w1_mvS0);
	CNF::Var N_will_C2R_ready = var( G.north(v),    t+1,    NdStat::C2R_ready);
	CNF::Var N_will_C2R_accN  = var( G.north(v),    t+1,    R_Move::w2_accN);
	CNF::Var N_will_C2R_mvN1  = var( G.north(v),    t+1,    R_Move::w2_mvN1);
	CNF::Var N_will_C2R_mvN2  = var( G.north(v),    t+1,    R_Move::w2_mvN2);
	CNF::Var N_will_C2R_mvN3  = var( G.north(v),    t+1,    R_Move::w2_mvN3);
	CNF::Var N_will_C2R_mvN0  = var( G.north(v),    t+1,    R_Move::w2_mvN0);
	CNF::Var N_will_C2R_accS  = var( G.north(v),    t+1,    R_Move::w2_accS);
	CNF::Var N_will_C2R_mvS1  = var( G.north(v),    t+1,    R_Move::w2_mvS1);
	CNF::Var N_will_C2R_mvS2  = var( G.north(v),    t+1,    R_Move::w2_mvS2);
	CNF::Var N_will_C2R_mvS3  = var( G.north(v),    t+1,    R_Move::w2_mvS3);
	CNF::Var N_will_C2R_mvS0  = var( G.north(v),    t+1,    R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var N_will_CR_accN;
	// neccessarly
	c = not(N_will_CR_accN)  or N_will_C0R_accN      or N_will_C1R_accN      or N_will_C2R_accN;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_accN) or N_will_C1R_accN      or N_will_C2R_accN      or N_will_CR_accN;
	m.addClause(c);
	c = N_will_C0R_accN      or not(N_will_C1R_accN) or N_will_C2R_accN      or N_will_CR_accN;
	m.addClause(c);
	c = N_will_C0R_accN      or N_will_C1R_accN      or not(N_will_C2R_accN) or N_will_CR_accN;
	m.addClause(c);
	c = not(N_will_C0R_accN) or not(N_will_C1R_accN);
	m.addClause(c);
	c = not(N_will_C0R_accN) or not(N_will_C2R_accN);
	m.addClause(c);
	c = not(N_will_C1R_accN) or not(N_will_C2R_accN);
	m.addClause(c);
	CNF::Var N_will_CR_mvN1;
	// neccessarly
	c = not(N_will_CR_mvN1)  or N_will_C0R_mvN1      or N_will_C1R_mvN1      or N_will_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_mvN1) or N_will_C1R_mvN1      or N_will_C2R_mvN1      or N_will_CR_mvN1;
	m.addClause(c);
	c = N_will_C0R_mvN1      or not(N_will_C1R_mvN1) or N_will_C2R_mvN1      or N_will_CR_mvN1;
	m.addClause(c);
	c = N_will_C0R_mvN1      or N_will_C1R_mvN1      or not(N_will_C2R_mvN1) or N_will_CR_mvN1;
	m.addClause(c);
	c = not(N_will_C0R_mvN1) or not(N_will_C1R_mvN1);
	m.addClause(c);
	c = not(N_will_C0R_mvN1) or not(N_will_C2R_mvN1);
	m.addClause(c);
	c = not(N_will_C1R_mvN1) or not(N_will_C2R_mvN1);
	m.addClause(c);
	CNF::Var N_will_CR_mvN2;
	// neccessarly
	c = not(N_will_CR_mvN2)  or N_will_C0R_mvN2      or N_will_C1R_mvN2      or N_will_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_mvN2) or N_will_C1R_mvN2      or N_will_C2R_mvN2      or N_will_CR_mvN2;
	m.addClause(c);
	c = N_will_C0R_mvN2      or not(N_will_C1R_mvN2) or N_will_C2R_mvN2      or N_will_CR_mvN2;
	m.addClause(c);
	c = N_will_C0R_mvN2      or N_will_C1R_mvN2      or not(N_will_C2R_mvN2) or N_will_CR_mvN2;
	m.addClause(c);
	c = not(N_will_C0R_mvN2) or not(N_will_C1R_mvN2);
	m.addClause(c);
	c = not(N_will_C0R_mvN2) or not(N_will_C2R_mvN2);
	m.addClause(c);
	c = not(N_will_C1R_mvN2) or not(N_will_C2R_mvN2);
	m.addClause(c);
	CNF::Var N_will_CR_mvN3;
	// neccessarly
	c = not(N_will_CR_mvN3)  or N_will_C0R_mvN3      or N_will_C1R_mvN3      or N_will_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_mvN3) or N_will_C1R_mvN3      or N_will_C2R_mvN3      or N_will_CR_mvN3;
	m.addClause(c);
	c = N_will_C0R_mvN3      or not(N_will_C1R_mvN3) or N_will_C2R_mvN3      or N_will_CR_mvN3;
	m.addClause(c);
	c = N_will_C0R_mvN3      or N_will_C1R_mvN3      or not(N_will_C2R_mvN3) or N_will_CR_mvN3;
	m.addClause(c);
	c = not(N_will_C0R_mvN3) or not(N_will_C1R_mvN3);
	m.addClause(c);
	c = not(N_will_C0R_mvN3) or not(N_will_C2R_mvN3);
	m.addClause(c);
	c = not(N_will_C1R_mvN3) or not(N_will_C2R_mvN3);
	m.addClause(c);
	CNF::Var N_will_CR_mvN0;
	// neccessarly
	c = not(N_will_CR_mvN0)  or N_will_C0R_mvN0      or N_will_C1R_mvN0      or N_will_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_mvN0) or N_will_C1R_mvN0      or N_will_C2R_mvN0      or N_will_CR_mvN0;
	m.addClause(c);
	c = N_will_C0R_mvN0      or not(N_will_C1R_mvN0) or N_will_C2R_mvN0      or N_will_CR_mvN0;
	m.addClause(c);
	c = N_will_C0R_mvN0      or N_will_C1R_mvN0      or not(N_will_C2R_mvN0) or N_will_CR_mvN0;
	m.addClause(c);
	c = not(N_will_C0R_mvN0) or not(N_will_C1R_mvN0);
	m.addClause(c);
	c = not(N_will_C0R_mvN0) or not(N_will_C2R_mvN0);
	m.addClause(c);
	c = not(N_will_C1R_mvN0) or not(N_will_C2R_mvN0);
	m.addClause(c);

	CNF::Var N_will_CR_accS;
	// neccessarly
	c = not(N_will_CR_accS)  or N_will_C0R_accS      or N_will_C1R_accS      or N_will_C2R_accS;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_accS) or N_will_C1R_accS      or N_will_C2R_accS      or N_will_CR_accS;
	m.addClause(c);
	c = N_will_C0R_accS      or not(N_will_C1R_accS) or N_will_C2R_accS      or N_will_CR_accS;
	m.addClause(c);
	c = N_will_C0R_accS      or N_will_C1R_accS      or not(N_will_C2R_accS) or N_will_CR_accS;
	m.addClause(c);
	c = not(N_will_C0R_accS) or not(N_will_C1R_accS);
	m.addClause(c);
	c = not(N_will_C0R_accS) or not(N_will_C2R_accS);
	m.addClause(c);
	c = not(N_will_C1R_accS) or not(N_will_C2R_accS);
	m.addClause(c);
	CNF::Var N_will_CR_mvS1;
	// neccessarly
	c = not(N_will_CR_mvS1)  or N_will_C0R_mvS1      or N_will_C1R_mvS1      or N_will_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_mvS1) or N_will_C1R_mvS1      or N_will_C2R_mvS1      or N_will_CR_mvS1;
	m.addClause(c);
	c = N_will_C0R_mvS1      or not(N_will_C1R_mvS1) or N_will_C2R_mvS1      or N_will_CR_mvS1;
	m.addClause(c);
	c = N_will_C0R_mvS1      or N_will_C1R_mvS1      or not(N_will_C2R_mvS1) or N_will_CR_mvS1;
	m.addClause(c);
	c = not(N_will_C0R_mvS1) or not(N_will_C1R_mvS1);
	m.addClause(c);
	c = not(N_will_C0R_mvS1) or not(N_will_C2R_mvS1);
	m.addClause(c);
	c = not(N_will_C1R_mvS1) or not(N_will_C2R_mvS1);
	m.addClause(c);
	CNF::Var N_will_CR_mvS2;
	// neccessarly
	c = not(N_will_CR_mvS2)  or N_will_C0R_mvS2      or N_will_C1R_mvS2      or N_will_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_mvS2) or N_will_C1R_mvS2      or N_will_C2R_mvS2      or N_will_CR_mvS2;
	m.addClause(c);
	c = N_will_C0R_mvS2      or not(N_will_C1R_mvS2) or N_will_C2R_mvS2      or N_will_CR_mvS2;
	m.addClause(c);
	c = N_will_C0R_mvS2      or N_will_C1R_mvS2      or not(N_will_C2R_mvS2) or N_will_CR_mvS2;
	m.addClause(c);
	c = not(N_will_C0R_mvS2) or not(N_will_C1R_mvS2);
	m.addClause(c);
	c = not(N_will_C0R_mvS2) or not(N_will_C2R_mvS2);
	m.addClause(c);
	c = not(N_will_C1R_mvS2) or not(N_will_C2R_mvS2);
	m.addClause(c);
	CNF::Var N_will_CR_mvS3;
	// neccessarly
	c = not(N_will_CR_mvS3)  or N_will_C0R_mvS3      or N_will_C1R_mvS3      or N_will_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_mvS3) or N_will_C1R_mvS3      or N_will_C2R_mvS3      or N_will_CR_mvS3;
	m.addClause(c);
	c = N_will_C0R_mvS3      or not(N_will_C1R_mvS3) or N_will_C2R_mvS3      or N_will_CR_mvS3;
	m.addClause(c);
	c = N_will_C0R_mvS3      or N_will_C1R_mvS3      or not(N_will_C2R_mvS3) or N_will_CR_mvS3;
	m.addClause(c);
	c = not(N_will_C0R_mvS3) or not(N_will_C1R_mvS3);
	m.addClause(c);
	c = not(N_will_C0R_mvS3) or not(N_will_C2R_mvS3);
	m.addClause(c);
	c = not(N_will_C1R_mvS3) or not(N_will_C2R_mvS3);
	m.addClause(c);
	CNF::Var N_will_CR_mvS0;
	// neccessarly
	c = not(N_will_CR_mvS0)  or N_will_C0R_mvS0      or N_will_C1R_mvS0      or N_will_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(N_will_C0R_mvS0) or N_will_C1R_mvS0      or N_will_C2R_mvS0      or N_will_CR_mvS0;
	m.addClause(c);
	c = N_will_C0R_mvS0      or not(N_will_C1R_mvS0) or N_will_C2R_mvS0      or N_will_CR_mvS0;
	m.addClause(c);
	c = N_will_C0R_mvS0      or N_will_C1R_mvS0      or not(N_will_C2R_mvS0) or N_will_CR_mvS0;
	m.addClause(c);
	c = not(N_will_C0R_mvS0) or not(N_will_C1R_mvS0);
	m.addClause(c);
	c = not(N_will_C0R_mvS0) or not(N_will_C2R_mvS0);
	m.addClause(c);
	c = not(N_will_C1R_mvS0) or not(N_will_C2R_mvS0);
	m.addClause(c);


	CNF::Var W_will_nobodyhome= var( G.west(v),    t+1,    NdStat::nobodyhome);
	CNF::Var W_will_R_ready   = var( G.west(v),    t+1,    NdStat::R_ready);
	CNF::Var W_will_R_accE    = var( G.west(v),    t+1,    R_Move::accE);
	CNF::Var W_will_R_mvE0    = var( G.west(v),    t+1,    R_Move::mvE0);
	CNF::Var W_will_R_accW    = var( G.west(v),    t+1,    R_Move::accW);
	CNF::Var W_will_R_mvW0    = var( G.west(v),    t+1,    R_Move::mvW0);
	CNF::Var W_will_C0R_ready = var( G.west(v),    t+1,    NdStat::C0R_ready);
	CNF::Var W_will_C0R_accE  = var( G.west(v),    t+1,    R_Move::w0_accE);
	CNF::Var W_will_C0R_mvE1  = var( G.west(v),    t+1,    R_Move::w0_mvE1);
	CNF::Var W_will_C0R_mvE0  = var( G.west(v),    t+1,    R_Move::w0_mvE0);
	CNF::Var W_will_C0R_accW  = var( G.west(v),    t+1,    R_Move::w0_accW);
	CNF::Var W_will_C0R_mvW1  = var( G.west(v),    t+1,    R_Move::w0_mvW1);
	CNF::Var W_will_C0R_mvW0  = var( G.west(v),    t+1,    R_Move::w0_mvW0);
	CNF::Var W_will_C1R_ready = var( G.west(v),    t+1,    NdStat::C1R_ready);
	CNF::Var W_will_C1R_accE  = var( G.west(v),    t+1,    R_Move::w1_accE);
	CNF::Var W_will_C1R_mvE1  = var( G.west(v),    t+1,    R_Move::w1_mvE1);
	CNF::Var W_will_C1R_mvE0  = var( G.west(v),    t+1,    R_Move::w1_mvE0);
	CNF::Var W_will_C1R_accW  = var( G.west(v),    t+1,    R_Move::w1_accW);
	CNF::Var W_will_C1R_mvW1  = var( G.west(v),    t+1,    R_Move::w1_mvW1);
	CNF::Var W_will_C1R_mvW0  = var( G.west(v),    t+1,    R_Move::w1_mvW0);
	CNF::Var W_will_C2R_ready = var( G.west(v),    t+1,    NdStat::C2R_ready);
	CNF::Var W_will_C2R_accE  = var( G.west(v),    t+1,    R_Move::w2_accE);
	CNF::Var W_will_C2R_mvE1  = var( G.west(v),    t+1,    R_Move::w2_mvE1);
	CNF::Var W_will_C2R_mvE0  = var( G.west(v),    t+1,    R_Move::w2_mvE0);
	CNF::Var W_will_C2R_accW  = var( G.west(v),    t+1,    R_Move::w2_accW);
	CNF::Var W_will_C2R_mvW1  = var( G.west(v),    t+1,    R_Move::w2_mvW1);
	CNF::Var W_will_C2R_mvW0  = var( G.west(v),    t+1,    R_Move::w2_mvW0);

        // Abbreviations
	CNF::Var W_will_CR_accE;
	// neccessarly
	c = not(W_will_CR_accE)  or W_will_C0R_accE      or W_will_C1R_accE      or W_will_C2R_accE;
	m.addClause(c);
	// sufficient
	c = not(W_will_C0R_accE) or W_will_C1R_accE      or W_will_C2R_accE      or W_will_CR_accE;
	m.addClause(c);
	c = W_will_C0R_accE      or not(W_will_C1R_accE) or W_will_C2R_accE      or W_will_CR_accE;
	m.addClause(c);
	c = W_will_C0R_accE      or W_will_C1R_accE      or not(W_will_C2R_accE) or W_will_CR_accE;
	m.addClause(c);
	c = not(W_will_C0R_accE) or not(W_will_C1R_accE);
	m.addClause(c);
	c = not(W_will_C0R_accE) or not(W_will_C2R_accE);
	m.addClause(c);
	c = not(W_will_C1R_accE) or not(W_will_C2R_accE);
	m.addClause(c);
	CNF::Var W_will_CR_mvE1;
	// neccessarly
	c = not(W_will_CR_mvE1)  or W_will_C0R_mvE1      or W_will_C1R_mvE1      or W_will_C2R_mvE1;
	m.addClause(c);
	// sufficient
	c = not(W_will_C0R_mvE1) or W_will_C1R_mvE1      or W_will_C2R_mvE1      or W_will_CR_mvE1;
	m.addClause(c);
	c = W_will_C0R_mvE1      or not(W_will_C1R_mvE1) or W_will_C2R_mvE1      or W_will_CR_mvE1;
	m.addClause(c);
	c = W_will_C0R_mvE1      or W_will_C1R_mvE1      or not(W_will_C2R_mvE1) or W_will_CR_mvE1;
	m.addClause(c);
	c = not(W_will_C0R_mvE1) or not(W_will_C1R_mvE1);
	m.addClause(c);
	c = not(W_will_C0R_mvE1) or not(W_will_C2R_mvE1);
	m.addClause(c);
	c = not(W_will_C1R_mvE1) or not(W_will_C2R_mvE1);
	m.addClause(c);
	CNF::Var W_will_CR_mvE0;
	// neccessarly
	c = not(W_will_CR_mvE0)  or W_will_C0R_mvE0      or W_will_C1R_mvE0      or W_will_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(W_will_C0R_mvE0) or W_will_C1R_mvE0      or W_will_C2R_mvE0      or W_will_CR_mvE0;
	m.addClause(c);
	c = W_will_C0R_mvE0      or not(W_will_C1R_mvE0) or W_will_C2R_mvE0      or W_will_CR_mvE0;
	m.addClause(c);
	c = W_will_C0R_mvE0      or W_will_C1R_mvE0      or not(W_will_C2R_mvE0) or W_will_CR_mvE0;
	m.addClause(c);
	c = not(W_will_C0R_mvE0) or not(W_will_C1R_mvE0);
	m.addClause(c);
	c = not(W_will_C0R_mvE0) or not(W_will_C2R_mvE0);
	m.addClause(c);
	c = not(W_will_C1R_mvE0) or not(W_will_C2R_mvE0);
	m.addClause(c);

	CNF::Var W_will_CR_mvW1;
	// neccessarly
	c = not(W_will_CR_mvW1)  or W_will_C0R_mvW1      or W_will_C1R_mvW1      or W_will_C2R_mvW1;
	m.addClause(c);
	// sufficient
	c = not(W_will_C0R_mvW1) or W_will_C1R_mvW1      or W_will_C2R_mvW1      or W_will_CR_mvW1;
	m.addClause(c);
	c = W_will_C0R_mvW1      or not(W_will_C1R_mvW1) or W_will_C2R_mvW1      or W_will_CR_mvW1;
	m.addClause(c);
	c = W_will_C0R_mvW1      or W_will_C1R_mvW1      or not(W_will_C2R_mvW1) or W_will_CR_mvW1;
	m.addClause(c);
	c = not(W_will_C0R_mvW1) or not(W_will_C1R_mvW1);
	m.addClause(c);
	c = not(W_will_C0R_mvW1) or not(W_will_C2R_mvW1);
	m.addClause(c);
	c = not(W_will_C1R_mvW1) or not(W_will_C2R_mvW1);
	m.addClause(c);
	CNF::Var W_will_CR_mvW0;
	// neccessarly
	c = not(W_will_CR_mvW0)  or W_will_C0R_mvW0      or W_will_C1R_mvW0      or W_will_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(W_will_C0R_mvW0) or W_will_C1R_mvW0      or W_will_C2R_mvW0      or W_will_CR_mvW0;
	m.addClause(c);
	c = W_will_C0R_mvW0      or not(W_will_C1R_mvW0) or W_will_C2R_mvW0      or W_will_CR_mvW0;
	m.addClause(c);
	c = W_will_C0R_mvW0      or W_will_C1R_mvW0      or not(W_will_C2R_mvW0) or W_will_CR_mvW0;
	m.addClause(c);
	c = not(W_will_C0R_mvW0) or not(W_will_C1R_mvW0);
	m.addClause(c);
	c = not(W_will_C0R_mvW0) or not(W_will_C2R_mvW0);
	m.addClause(c);
	c = not(W_will_C1R_mvW0) or not(W_will_C2R_mvW0);
	m.addClause(c);


	CNF::Var S_will_nobodyhome= var( G.south(v),    t+1,    NdStat::nobodyhome);
	CNF::Var S_will_R_ready   = var( G.south(v),    t+1,    NdStat::R_ready);
	CNF::Var S_will_R_accN    = var( G.south(v),    t+1,    R_Move::accN);
	CNF::Var S_will_R_mvN1    = var( G.south(v),    t+1,    R_Move::mvN1);
	CNF::Var S_will_R_mvN0    = var( G.south(v),    t+1,    R_Move::mvN0);
	CNF::Var S_will_R_accS    = var( G.south(v),    t+1,    R_Move::accS);
	CNF::Var S_will_R_mvS1    = var( G.south(v),    t+1,    R_Move::mvS1);
	CNF::Var S_will_R_mvS0    = var( G.south(v),    t+1,    R_Move::mvS0);
	CNF::Var S_will_C0R_ready = var( G.south(v),    t+1,    NdStat::C0R_ready);
	CNF::Var S_will_C0R_accN  = var( G.south(v),    t+1,    R_Move::w0_accN);
	CNF::Var S_will_C0R_mvN1  = var( G.south(v),    t+1,    R_Move::w0_mvN1);
	CNF::Var S_will_C0R_mvN2  = var( G.south(v),    t+1,    R_Move::w0_mvN2);
	CNF::Var S_will_C0R_mvN3  = var( G.south(v),    t+1,    R_Move::w0_mvN3);
	CNF::Var S_will_C0R_mvN0  = var( G.south(v),    t+1,    R_Move::w0_mvN0);
	CNF::Var S_will_C0R_accS  = var( G.south(v),    t+1,    R_Move::w0_accS);
	CNF::Var S_will_C0R_mvS1  = var( G.south(v),    t+1,    R_Move::w0_mvS1);
	CNF::Var S_will_C0R_mvS2  = var( G.south(v),    t+1,    R_Move::w0_mvS2);
	CNF::Var S_will_C0R_mvS3  = var( G.south(v),    t+1,    R_Move::w0_mvS3);
	CNF::Var S_will_C0R_mvS0  = var( G.south(v),    t+1,    R_Move::w0_mvS0);
	CNF::Var S_will_C1R_ready = var( G.south(v),    t+1,    NdStat::C1R_ready);
	CNF::Var S_will_C1R_accN  = var( G.south(v),    t+1,    R_Move::w1_accN);
	CNF::Var S_will_C1R_mvN1  = var( G.south(v),    t+1,    R_Move::w1_mvN1);
	CNF::Var S_will_C1R_mvN2  = var( G.south(v),    t+1,    R_Move::w1_mvN2);
	CNF::Var S_will_C1R_mvN3  = var( G.south(v),    t+1,    R_Move::w1_mvN3);
	CNF::Var S_will_C1R_mvN0  = var( G.south(v),    t+1,    R_Move::w1_mvN0);
	CNF::Var S_will_C1R_accS  = var( G.south(v),    t+1,    R_Move::w1_accS);
	CNF::Var S_will_C1R_mvS1  = var( G.south(v),    t+1,    R_Move::w1_mvS1);
	CNF::Var S_will_C1R_mvS2  = var( G.south(v),    t+1,    R_Move::w1_mvS2);
	CNF::Var S_will_C1R_mvS3  = var( G.south(v),    t+1,    R_Move::w1_mvS3);
	CNF::Var S_will_C1R_mvS0  = var( G.south(v),    t+1,    R_Move::w1_mvS0);
	CNF::Var S_will_C2R_ready = var( G.south(v),    t+1,    NdStat::C2R_ready);
	CNF::Var S_will_C2R_accN  = var( G.south(v),    t+1,    R_Move::w2_accN);
	CNF::Var S_will_C2R_mvN1  = var( G.south(v),    t+1,    R_Move::w2_mvN1);
	CNF::Var S_will_C2R_mvN2  = var( G.south(v),    t+1,    R_Move::w2_mvN2);
	CNF::Var S_will_C2R_mvN3  = var( G.south(v),    t+1,    R_Move::w2_mvN3);
	CNF::Var S_will_C2R_mvN0  = var( G.south(v),    t+1,    R_Move::w2_mvN0);
	CNF::Var S_will_C2R_accS  = var( G.south(v),    t+1,    R_Move::w2_accS);
	CNF::Var S_will_C2R_mvS1  = var( G.south(v),    t+1,    R_Move::w2_mvS1);
	CNF::Var S_will_C2R_mvS2  = var( G.south(v),    t+1,    R_Move::w2_mvS2);
	CNF::Var S_will_C2R_mvS3  = var( G.south(v),    t+1,    R_Move::w2_mvS3);
	CNF::Var S_will_C2R_mvS0  = var( G.south(v),    t+1,    R_Move::w2_mvS0);

        // Abbreviations
	CNF::Var S_will_CR_accN;
	// neccessarly
	c = not(S_will_CR_accN)   or S_will_C0R_accN          or S_will_C1R_accN      or S_will_C2R_accN;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_accN)  or S_will_C1R_accN          or S_will_C2R_accN      or S_will_CR_accN;
	m.addClause(c);
	c = S_will_C0R_accN       or not(S_will_C1R_accN)     or S_will_C2R_accN      or S_will_CR_accN;
	m.addClause(c);
	c = S_will_C0R_accN       or S_will_C1R_accN          or not(S_will_C2R_accN) or S_will_CR_accN;
	m.addClause(c);
	c = not(S_will_C0R_accN)  or not(S_will_C1R_accN);
	m.addClause(c);
	c = not(S_will_C0R_accN)  or not(S_will_C2R_accN);
	m.addClause(c);
	c = not(S_will_C1R_accN)  or not(S_will_C2R_accN);
	m.addClause(c);
	CNF::Var S_will_CR_mvN1;
	// neccessarly
	c = not(S_will_CR_mvN1)   or S_will_C0R_mvN1          or S_will_C1R_mvN1      or S_will_C2R_mvN1;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_mvN1)  or S_will_C1R_mvN1          or S_will_C2R_mvN1      or S_will_CR_mvN1;
	m.addClause(c);
	c = S_will_C0R_mvN1       or not(S_will_C1R_mvN1)     or S_will_C2R_mvN1      or S_will_CR_mvN1;
	m.addClause(c);
	c = S_will_C0R_mvN1       or S_will_C1R_mvN1          or not(S_will_C2R_mvN1) or S_will_CR_mvN1;
	m.addClause(c);
	c = not(S_will_C0R_mvN1)  or not(S_will_C1R_mvN1);
	m.addClause(c);
	c = not(S_will_C0R_mvN1)  or not(S_will_C2R_mvN1);
	m.addClause(c);
	c = not(S_will_C1R_mvN1)  or not(S_will_C2R_mvN1);
	m.addClause(c);
	CNF::Var S_will_CR_mvN2;
	// neccessarly
	c = not(S_will_CR_mvN2)   or S_will_C0R_mvN2          or S_will_C1R_mvN2      or S_will_C2R_mvN2;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_mvN2)  or S_will_C1R_mvN2          or S_will_C2R_mvN2      or S_will_CR_mvN2;
	m.addClause(c);
	c = S_will_C0R_mvN2       or not(S_will_C1R_mvN2)     or S_will_C2R_mvN2      or S_will_CR_mvN2;
	m.addClause(c);
	c = S_will_C0R_mvN2       or S_will_C1R_mvN2          or not(S_will_C2R_mvN2) or S_will_CR_mvN2;
	m.addClause(c);
	c = not(S_will_C0R_mvN2)  or not(S_will_C1R_mvN2);
	m.addClause(c);
	c = not(S_will_C0R_mvN2)  or not(S_will_C2R_mvN2);
	m.addClause(c);
	c = not(S_will_C1R_mvN2)  or not(S_will_C2R_mvN2);
	m.addClause(c);
	CNF::Var S_will_CR_mvN3;
	// neccessarly
	c = not(S_will_CR_mvN3)   or S_will_C0R_mvN3          or S_will_C1R_mvN3      or S_will_C2R_mvN3;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_mvN3)  or S_will_C1R_mvN3          or S_will_C2R_mvN3      or S_will_CR_mvN3;
	m.addClause(c);
	c = S_will_C0R_mvN3       or not(S_will_C1R_mvN3)     or S_will_C2R_mvN3      or S_will_CR_mvN3;
	m.addClause(c);
	c = S_will_C0R_mvN3       or S_will_C1R_mvN3          or not(S_will_C2R_mvN3) or S_will_CR_mvN3;
	m.addClause(c);
	c = not(S_will_C0R_mvN3)  or not(S_will_C1R_mvN3);
	m.addClause(c);
	c = not(S_will_C0R_mvN3)  or not(S_will_C2R_mvN3);
	m.addClause(c);
	c = not(S_will_C1R_mvN3)  or not(S_will_C2R_mvN3);
	m.addClause(c);
	CNF::Var S_will_CR_mvN0;
	// neccessarly
	c = not(S_will_CR_mvN0)   or S_will_C0R_mvN0          or S_will_C1R_mvN0      or S_will_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_mvN0)  or S_will_C1R_mvN0          or S_will_C2R_mvN0      or S_will_CR_mvN0;
	m.addClause(c);
	c = S_will_C0R_mvN0       or not(S_will_C1R_mvN0)     or S_will_C2R_mvN0      or S_will_CR_mvN0;
	m.addClause(c);
	c = S_will_C0R_mvN0       or S_will_C1R_mvN0          or not(S_will_C2R_mvN0) or S_will_CR_mvN0;
	m.addClause(c);
	c = not(S_will_C0R_mvN0)  or not(S_will_C1R_mvN0);
	m.addClause(c);
	c = not(S_will_C0R_mvN0)  or not(S_will_C2R_mvN0);
	m.addClause(c);
	c = not(S_will_C1R_mvN0)  or not(S_will_C2R_mvN0);
	m.addClause(c);

	CNF::Var S_will_CR_mvS1;
	// neccessarly
	c = not(S_will_CR_mvS1)   or S_will_C0R_mvS1           or S_will_C1R_mvS1      or S_will_C2R_mvS1;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_mvS1)  or S_will_C1R_mvS1           or S_will_C2R_mvS1      or S_will_CR_mvS1;
	m.addClause(c);
	c = S_will_C0R_mvS1       or not(S_will_C1R_mvS1)      or S_will_C2R_mvS1      or S_will_CR_mvS1;
	m.addClause(c);
	c = S_will_C0R_mvS1       or S_will_C1R_mvS1           or not(S_will_C2R_mvS1) or S_will_CR_mvS1;
	m.addClause(c);
	c = not(S_will_C0R_mvS1)  or not(S_will_C1R_mvS1);
	m.addClause(c);
	c = not(S_will_C0R_mvS1)  or not(S_will_C2R_mvS1);
	m.addClause(c);
	c = not(S_will_C1R_mvS1)  or not(S_will_C2R_mvS1);
	m.addClause(c);
	CNF::Var S_will_CR_mvS2;
	// neccessarly
	c = not(S_will_CR_mvS2)   or S_will_C0R_mvS2           or S_will_C1R_mvS2      or S_will_C2R_mvS2;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_mvS2)  or S_will_C1R_mvS2           or S_will_C2R_mvS2      or S_will_CR_mvS2;
	m.addClause(c);
	c = S_will_C0R_mvS2       or not(S_will_C1R_mvS2)      or S_will_C2R_mvS2      or S_will_CR_mvS2;
	m.addClause(c);
	c = S_will_C0R_mvS2       or S_will_C1R_mvS2           or not(S_will_C2R_mvS2) or S_will_CR_mvS2;
	m.addClause(c);
	c = not(S_will_C0R_mvS2)  or not(S_will_C1R_mvS2);
	m.addClause(c);
	c = not(S_will_C0R_mvS2)  or not(S_will_C2R_mvS2);
	m.addClause(c);
	c = not(S_will_C1R_mvS2)  or not(S_will_C2R_mvS2);
	m.addClause(c);
	CNF::Var S_will_CR_mvS3;
	// neccessarly
	c = not(S_will_CR_mvS3)   or S_will_C0R_mvS3           or S_will_C1R_mvS3      or S_will_C2R_mvS3;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_mvS3)  or S_will_C1R_mvS3           or S_will_C2R_mvS3      or S_will_CR_mvS3;
	m.addClause(c);
	c = S_will_C0R_mvS3       or not(S_will_C1R_mvS3)      or S_will_C2R_mvS3      or S_will_CR_mvS3;
	m.addClause(c);
	c = S_will_C0R_mvS3       or S_will_C1R_mvS3           or not(S_will_C2R_mvS3) or S_will_CR_mvS3;
	m.addClause(c);
	c = not(S_will_C0R_mvS3)  or not(S_will_C1R_mvS3);
	m.addClause(c);
	c = not(S_will_C0R_mvS3)  or not(S_will_C2R_mvS3);
	m.addClause(c);
	c = not(S_will_C1R_mvS3)  or not(S_will_C2R_mvS3);
	m.addClause(c);
	CNF::Var S_will_CR_mvS0;
	// neccessarly
	c = not(S_will_CR_mvS0)   or S_will_C0R_mvS0           or S_will_C1R_mvS0      or S_will_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(S_will_C0R_mvS0)  or S_will_C1R_mvS0           or S_will_C2R_mvS0      or S_will_CR_mvS0;
	m.addClause(c);
	c = S_will_C0R_mvS0       or not(S_will_C1R_mvS0)      or S_will_C2R_mvS0      or S_will_CR_mvS0;
	m.addClause(c);
	c = S_will_C0R_mvS0       or S_will_C1R_mvS0           or not(S_will_C2R_mvS0) or S_will_CR_mvS0;
	m.addClause(c);
	c = not(S_will_C0R_mvS0)  or not(S_will_C1R_mvS0);
	m.addClause(c);
	c = not(S_will_C0R_mvS0)  or not(S_will_C2R_mvS0);
	m.addClause(c);
	c = not(S_will_C1R_mvS0)  or not(S_will_C2R_mvS0);
	m.addClause(c);


	

        // Robot alone
        c = not(Here_will_R_accE) or Here_now_R_ready;
	m.addClause(c);//
	c = not(Here_will_R_mvE0) or W_now_R_accE              or W_now_R_mvE0;
	m.addClause(c);//
	c = not(Here_now_R_mvE0)  or Here_now_R_accE           or E_will_R_ready       or E_will_R_mvE0;
	m.addClause(c);
	c = Here_now_R_mvE0       or not(Here_now_R_accE)      or E_will_R_ready       or E_will_R_mvE0;
	m.addClause(c);
	c = not(Here_now_R_mvE0)  or not(Here_now_R_accE);
	m.addClause(c);//
	c = not(Here_now_R_accE)  or Here_will_nobodyhome      or Here_will_R_mvE0;
	m.addClause(c);//
	c = not(Here_will_R_accE) or E_now_nobodyhome          or E_now_R_accE         or E_now_CR_mvE1;
	m.addClause(c);//
	c = E_will_R_ready        or not(Here_now_R_accE)      or not(E_now_CR_mvE1);
	m.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = E_will_R_ready        or not(Here_now_R_mvE0)      or not(E_now_CR_mvE1);
	m.addClause(c);// Crobot ahead.  Hint: contrapositive

	
	c = not(Here_will_R_accW) or Here_now_R_ready;
	m.addClause(c);//
	c = not(Here_will_R_mvW0) or E_now_R_accW              or E_now_R_mvW0;
	m.addClause(c);//
	c = not(Here_now_R_mvW0)  or Here_now_R_accW           or W_will_R_ready       or W_will_R_mvE0;
	m.addClause(c);
	c = Here_now_R_mvW0       or not(Here_now_R_accW)      or W_will_R_ready       or W_will_R_mvW0;
	m.addClause(c);
	c = not(Here_now_R_mvW0)  or not(Here_now_R_accW);
	m.addClause(c);//
	c = not(Here_now_R_accW)  or Here_will_nobodyhome      or Here_will_R_mvW0;
	m.addClause(c);//
	c = not(Here_will_R_accW) or W_now_nobodyhome          or W_now_R_accW         or W_now_CR_mvW1;
	m.addClause(c);//
	c = W_will_R_ready        or not(Here_now_R_accW)      or not(W_now_CR_mvW1);
	m.addClause(c);// Crobot ahead.  Hint: contrapositive
	c = W_will_R_ready        or not(Here_now_R_mvW0)      or not(W_now_CR_mvW1);
	m.addClause(c);// Crobot ahead.  Hint: contrapositive


	
	c = not(Here_will_R_accN) or Here_now_R_ready;
	m.addClause(c);//
	// neccessarly
	c = not(Here_will_R_mvN1) or Here_now_R_accN           or Here_now_R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_R_accN)  or Here_now_R_mvN0           or Here_will_R_mvN1;
	m.addClause(c);
	c = Here_now_R_accN       or not(Here_now_R_mvN0)      or Here_will_R_mvN1;
	m.addClause(c);
	c = not(Here_now_R_accN)  or not(Here_now_R_mvN0);
	m.addClause(c);//	
	c = not(Here_will_R_mvN0) or S_now_R_mvN1;
	m.addClause(c);//
	c = not(Here_now_R_mvN0)  or Here_now_R_accN           or Here_will_R_mvN1;
	m.addClause(c);
	c = Here_now_R_mvN0       or not(Here_now_R_accN)      or Here_will_R_mvN1;
	m.addClause(c);
	c = not(Here_now_R_mvN0)  or not(Here_now_R_accN);
	m.addClause(c);//
	c = not(Here_now_R_mvN1)  or N_will_R_mvN0             or N_will_R_ready;
	m.addClause(c);//
	c = not(Here_now_R_mvN1)  or Here_will_nobodyhome      or Here_will_R_mvN0;
	m.addClause(c);//
	c = N_will_R_ready        or not(Here_now_R_mvN1)      or not(N_now_CR_mvN3);
	m.addClause(c);// Crobot ahead.  Hint: contrapositive

	
	c = not(Here_will_R_accS) or Here_now_R_ready;
	m.addClause(c);//
	// neccessarly
	c = not(Here_will_R_mvS1) or Here_now_R_accS           or Here_now_R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_R_accS)  or Here_now_R_mvS0           or Here_will_R_mvS1;
	m.addClause(c);
	c = Here_now_R_accS       or not(Here_now_R_mvS0)      or Here_will_R_mvS1;
	m.addClause(c);
	c = not(Here_now_R_accS)  or not(Here_now_R_mvS0);
	m.addClause(c);//!!!!!!!!!!!!!!
	c = not(Here_will_R_mvS0) or N_now_R_mvS1;
	m.addClause(c);//
	c = not(Here_now_R_mvS0)  or Here_now_R_accS           or Here_will_R_mvS1;
	m.addClause(c);
	c = Here_now_R_mvS0       or not(Here_now_R_accS)      or Here_will_R_mvS1;
	m.addClause(c);
	c = not(Here_now_R_mvS0)  or not(Here_now_R_accS);
	m.addClause(c);//
	c = not(Here_now_R_mvS1)  or S_will_R_mvS0             or S_will_R_ready;
	m.addClause(c);//
	c = not(Here_now_R_mvS1)  or Here_will_nobodyhome      or Here_will_R_mvS0;
	m.addClause(c);//
	c = S_will_R_ready        or not(Here_now_R_mvS1)      or not(S_now_CR_mvS3);
	m.addClause(c);// Crobot ahead.  Hint: contrapositive

        // C r o b o t  0
	c = not(Here_will_C0R_accN) or Here_now_C0R_ready;
	m.addClause(c); //
	// neccesarly
	c = not(Here_will_C0R_mvN1) or Here_now_C0R_accN       or Here_now_C0R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accN)  or Here_now_C0R_mvN0       or Here_will_C0R_mvN1;
	m.addClause(c);
	c = Here_now_C0R_accN       or not(Here_now_C0R_mvN0)  or Here_will_C0R_mvN1;
	m.addClause(c);
	c = not(Here_now_C0R_accN)  or not(Here_now_C0R_mvN0);
	m.addClause(c); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C0R_mvN2) or Here_now_C0R_mvN1;
	m.addClause(c);
	c = not(Here_now_C0R_mvN1)  or Here_will_C0R_mvN2;
	m.addClause(c); //
	c = not(Here_will_C0R_mvN3) or Here_now_C0R_mvN2;
	m.addClause(c);
	c = not(Here_now_C0R_mvN2)  or Here_will_C0R_mvN3;
	m.addClause(c); //
	c = not(Here_will_C0R_mvN0) or S_now_C0R_mvN3;
	m.addClause(c); //
	c = not(Here_now_C0R_accN)  or Here_now_C0R_mvN0       or Here_will_C0R_mvN1;
	m.addClause(c);
	c = Here_now_C0R_accN       or not(Here_now_C0R_mvN0)  or Here_will_C0R_mvN1;
	m.addClause(c);
	c = not(Here_now_C0R_accN)  or not(Here_now_C0R_mvN0);
	m.addClause(c); //
	c = not(Here_now_C0R_mvN3)  or N_will_C0R_ready        or N_will_C0R_mvN0;
	m.addClause(c); //
	c = not(Here_now_C0R_mvN3)  or Here_will_nobodyhome    or Here_will_R_mvN0;
	m.addClause(c); //

	
	c = not(Here_will_C0R_accS) or Here_now_C0R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C0R_mvS1) or Here_now_C0R_accS       or Here_now_C0R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accS)  or  Here_now_C0R_mvS0      or Here_will_C0R_mvS1;
	m.addClause(c);
	c = Here_now_C0R_accS       or not(Here_now_C0R_mvS0)  or Here_will_C0R_mvS1;
	m.addClause(c);
	c = not(Here_now_C0R_accS)  or not(Here_now_C0R_mvS0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C0R_mvS2) or Here_now_C0R_mvS1;
	m.addClause(c);
	c = not(Here_now_C0R_mvS1)  or Here_will_C0R_mvS2;
	m.addClause(c); //
	c = not(Here_will_C0R_mvS3) or Here_now_C0R_mvS2;
	m.addClause(c);
	c = not(Here_now_C0R_mvS2)  or Here_will_C0R_mvS3;
	m.addClause(c); //
	c = not(Here_will_C0R_mvS0) or N_now_C0R_mvS3;
	m.addClause(c); //
	c = not(Here_now_C0R_accS)  or Here_now_C0R_mvS0       or Here_will_C0R_mvS1;
	m.addClause(c);
	c = Here_now_C0R_accS       or not(Here_now_C0R_mvS0)  or Here_will_C0R_mvS1;
	m.addClause(c);
	c = not(Here_now_C0R_accS)  or not(Here_now_C0R_mvS0);
	m.addClause(c); //
	c = not(Here_now_C0R_mvS3)  or S_will_C0R_ready        or S_will_C0R_mvS0;
	m.addClause(c); //
	c = not(Here_now_C0R_mvS3)  or Here_will_nobodyhome    or Here_will_R_mvS0;
	m.addClause(c); //


	
	c = not(Here_will_C0R_accE) or Here_now_C0R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C0R_mvE1) or Here_now_C0R_accE       or Here_now_C0R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accE)  or Here_now_C0R_mvE0       or Here_will_C0R_mvE1;
	m.addClause(c);
	c = Here_now_C0R_accE       or not(Here_now_C0R_mvE0)  or Here_will_C0R_mvE1;
	m.addClause(c);
	c = not(Here_now_C0R_accE)  or not(Here_now_C0R_mvE0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C0R_mvE0) or W_now_C0R_mvE1;
	m.addClause(c); //
	c = not(Here_now_C0R_accE)  or Here_now_C0R_mvE0       or Here_will_C0R_mvE1;
	m.addClause(c);
	c = Here_now_C0R_accE       or not(Here_now_C0R_mvE0)  or Here_will_C0R_mvE1;
	m.addClause(c);
	c = not(Here_now_C0R_accE)  or not(Here_now_C0R_mvE0);
	m.addClause(c); //
	c = not(Here_now_C0R_mvE1)  or E_will_C0R_ready        or E_will_C0R_mvE0;
	m.addClause(c); //
	c = not(Here_now_C0R_mvE1)  or Here_will_nobodyhome    or Here_will_R_mvE0;
	m.addClause(c); //


	
	c = not(Here_will_C0R_accW) or Here_now_C0R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C0R_mvW1) or Here_now_C0R_accW       or Here_now_C0R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C0R_accW)  or Here_now_C0R_mvW0       or Here_will_C0R_mvW1;
	m.addClause(c);
	c = Here_now_C0R_accW       or not(Here_now_C0R_mvW0)  or Here_will_C0R_mvW1;
	m.addClause(c);
	c = not(Here_now_C0R_accW)  or not(Here_now_C0R_mvW0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C0R_mvW0) or E_now_C0R_mvW1;
	m.addClause(c); //
	c = not(Here_now_C0R_accW)  or Here_now_C0R_mvW0       or Here_will_C0R_mvW1;
	m.addClause(c);
	c = Here_now_C0R_accW       or not(Here_now_C0R_mvW0)  or Here_will_C0R_mvW1;
	m.addClause(c);
	c = not(Here_now_C0R_accW)  or not(Here_now_C0R_mvW0);
	m.addClause(c); //
	c = not(Here_now_C0R_mvW1)  or W_will_C0R_ready        or W_will_C0R_mvW0;
	m.addClause(c); //
	c = not(Here_now_C0R_mvW1)  or Here_will_nobodyhome    or Here_will_R_mvW0;
	m.addClause(c); //


	
        // C r o b o t  1
	c = not(Here_will_C1R_accN) or Here_now_C1R_ready;
	m.addClause(c); //
	// neccesarly
	c = not(Here_will_C1R_mvN1) or Here_now_C1R_accN       or Here_now_C1R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C1R_accN)  or Here_now_C1R_mvN0       or Here_will_C1R_mvN1;
	m.addClause(c);
	c = Here_now_C1R_accN       or not(Here_now_C1R_mvN0)  or Here_will_C1R_mvN1;
	m.addClause(c);
	c = not(Here_now_C1R_accN)  or not(Here_now_C1R_mvN0);
	m.addClause(c); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C1R_mvN2) or Here_now_C1R_mvN1;
	m.addClause(c);
	c = not(Here_now_C1R_mvN1)  or Here_will_C1R_mvN2;
	m.addClause(c); //
	c = not(Here_will_C1R_mvN3) or Here_now_C1R_mvN2;
	m.addClause(c);
	c = not(Here_now_C1R_mvN2)  or Here_will_C1R_mvN3;
	m.addClause(c); //
	c = not(Here_will_C1R_mvN0) or S_now_C1R_mvN3;
	m.addClause(c); //
	c = not(Here_now_C1R_accN)  or Here_now_C1R_mvN0       or Here_will_C1R_mvN1;
	m.addClause(c);
	c = Here_now_C1R_accN       or not(Here_now_C1R_mvN0)  or Here_will_C1R_mvN1;
	m.addClause(c);
	c = not(Here_now_C1R_accN)  or not(Here_now_C1R_mvN0);
	m.addClause(c); //
	c = not(Here_now_C1R_mvN3)  or N_will_C1R_ready        or N_will_C1R_mvN0;
	m.addClause(c); //
	c = not(Here_now_C1R_mvN3)  or Here_will_nobodyhome    or Here_will_R_mvN0;
	m.addClause(c); //

	
	c = not(Here_will_C1R_accS) or Here_now_C1R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C1R_mvS1) or Here_now_C1R_accS       or Here_now_C1R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C1R_accS)  or  Here_now_C1R_mvS0      or Here_will_C1R_mvS1;
	m.addClause(c);
	c = Here_now_C1R_accS       or not(Here_now_C1R_mvS0)  or Here_will_C1R_mvS1;
	m.addClause(c);
	c = not(Here_now_C1R_accS)  or not(Here_now_C1R_mvS0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C1R_mvS2) or Here_now_C1R_mvS1;
	m.addClause(c);
	c = not(Here_now_C1R_mvS1)  or Here_will_C1R_mvS2;
	m.addClause(c); //
	c = not(Here_will_C1R_mvS3) or Here_now_C1R_mvS2;
	m.addClause(c);
	c = not(Here_now_C1R_mvS2)  or Here_will_C1R_mvS3;
	m.addClause(c); //
	c = not(Here_will_C1R_mvS0) or N_now_C1R_mvS3;
	m.addClause(c); //
	c = not(Here_now_C1R_accS)  or Here_now_C1R_mvS0       or Here_will_C1R_mvS1;
	m.addClause(c);
	c = Here_now_C1R_accS       or not(Here_now_C1R_mvS0)  or Here_will_C1R_mvS1;
	m.addClause(c);
	c = not(Here_now_C1R_accS)  or not(Here_now_C1R_mvS0);
	m.addClause(c); //
	c = not(Here_now_C1R_mvS3)  or S_will_C1R_ready        or S_will_C1R_mvS0;
	m.addClause(c); //
	c = not(Here_now_C1R_mvS3)  or Here_will_nobodyhome    or Here_will_R_mvS0;
	m.addClause(c); //


	
	c = not(Here_will_C1R_accE) or Here_now_C1R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C1R_mvE1) or Here_now_C1R_accE       or Here_now_C1R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C1R_accE)  or Here_now_C1R_mvE0       or Here_will_C1R_mvE1;
	m.addClause(c);
	c = Here_now_C1R_accE       or not(Here_now_C1R_mvE0)  or Here_will_C1R_mvE1;
	m.addClause(c);
	c = not(Here_now_C1R_accE)  or not(Here_now_C1R_mvE0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C1R_mvE0) or W_now_C1R_mvE1;
	m.addClause(c); //
	c = not(Here_now_C1R_accE)  or Here_now_C1R_mvE0       or Here_will_C1R_mvE1;
	m.addClause(c);
	c = Here_now_C1R_accE       or not(Here_now_C1R_mvE0)  or Here_will_C1R_mvE1;
	m.addClause(c);
	c = not(Here_now_C1R_accE)  or not(Here_now_C1R_mvE0);
	m.addClause(c); //
	c = not(Here_now_C1R_mvE1)  or E_will_C1R_ready        or E_will_C1R_mvE0;
	m.addClause(c); //
	c = not(Here_now_C1R_mvE1)  or Here_will_nobodyhome    or Here_will_R_mvE0;
	m.addClause(c); //


	
	c = not(Here_will_C1R_accW) or Here_now_C1R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C1R_mvW1) or Here_now_C1R_accW       or Here_now_C1R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C1R_accW)  or Here_now_C1R_mvW0       or Here_will_C1R_mvW1;
	m.addClause(c);
	c = Here_now_C1R_accW       or not(Here_now_C1R_mvW0)  or Here_will_C1R_mvW1;
	m.addClause(c);
	c = not(Here_now_C1R_accW)  or not(Here_now_C1R_mvW0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C1R_mvW0) or E_now_C1R_mvW1;
	m.addClause(c); //
	c = not(Here_now_C1R_accW)  or Here_now_C1R_mvW0       or Here_will_C1R_mvW1;
	m.addClause(c);
	c = Here_now_C1R_accW       or not(Here_now_C1R_mvW0)  or Here_will_C1R_mvW1;
	m.addClause(c);
	c = not(Here_now_C1R_accW)  or not(Here_now_C1R_mvW0);
	m.addClause(c); //
	c = not(Here_now_C1R_mvW1)  or W_will_C1R_ready        or W_will_C1R_mvW0;
	m.addClause(c); //
	c = not(Here_now_C1R_mvW1)  or Here_will_nobodyhome    or Here_will_R_mvW0;
	m.addClause(c); //




        // C r o b o t  2
	c = not(Here_will_C2R_accN) or Here_now_C2R_ready;
	m.addClause(c); //
	// neccesarly
	c = not(Here_will_C2R_mvN1) or Here_now_C2R_accN       or Here_now_C2R_mvN0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C2R_accN)  or Here_now_C2R_mvN0       or Here_will_C2R_mvN1;
	m.addClause(c);
	c = Here_now_C2R_accN       or not(Here_now_C2R_mvN0)  or Here_will_C2R_mvN1;
	m.addClause(c);
	c = not(Here_now_C2R_accN)  or not(Here_now_C2R_mvN0);
	m.addClause(c); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C2R_mvN2) or Here_now_C2R_mvN1;
	m.addClause(c);
	c = not(Here_now_C2R_mvN1)  or Here_will_C2R_mvN2;
	m.addClause(c); //
	c = not(Here_will_C2R_mvN3) or Here_now_C2R_mvN2;
	m.addClause(c);
	c = not(Here_now_C2R_mvN2)  or Here_will_C2R_mvN3;
	m.addClause(c); //
	c = not(Here_will_C2R_mvN0) or S_now_C2R_mvN3;
	m.addClause(c); //
	c = not(Here_now_C2R_accN)  or Here_now_C2R_mvN0       or Here_will_C2R_mvN1;
	m.addClause(c);
	c = Here_now_C2R_accN       or not(Here_now_C2R_mvN0)  or Here_will_C2R_mvN1;
	m.addClause(c);
	c = not(Here_now_C2R_accN)  or not(Here_now_C2R_mvN0);
	m.addClause(c); //
	c = not(Here_now_C2R_mvN3)  or N_will_C2R_ready        or N_will_C2R_mvN0;
	m.addClause(c); //
	c = not(Here_now_C2R_mvN3)  or Here_will_nobodyhome    or Here_will_R_mvN0;
	m.addClause(c); //

	
	c = not(Here_will_C2R_accS) or Here_now_C2R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C2R_mvS1) or Here_now_C2R_accS       or Here_now_C2R_mvS0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C2R_accS)  or  Here_now_C2R_mvS0      or Here_will_C2R_mvS1;
	m.addClause(c);
	c = Here_now_C2R_accS       or not(Here_now_C2R_mvS0)  or Here_will_C2R_mvS1;
	m.addClause(c);
	c = not(Here_now_C2R_accS)  or not(Here_now_C2R_mvS0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C2R_mvS2) or Here_now_C2R_mvS1;
	m.addClause(c);
	c = not(Here_now_C2R_mvS1)  or Here_will_C2R_mvS2;
	m.addClause(c); //
	c = not(Here_will_C2R_mvS3) or Here_now_C2R_mvS2;
	m.addClause(c);
	c = not(Here_now_C2R_mvS2)  or Here_will_C2R_mvS3;
	m.addClause(c); //
	c = not(Here_will_C2R_mvS0) or N_now_C2R_mvS3;
	m.addClause(c); //
	c = not(Here_now_C2R_accS)  or Here_now_C2R_mvS0       or Here_will_C2R_mvS1;
	m.addClause(c);
	c = Here_now_C2R_accS       or not(Here_now_C2R_mvS0)  or Here_will_C2R_mvS1;
	m.addClause(c);
	c = not(Here_now_C2R_accS)  or not(Here_now_C2R_mvS0);
	m.addClause(c); //
	c = not(Here_now_C2R_mvS3)  or S_will_C2R_ready        or S_will_C2R_mvS0;
	m.addClause(c); //
	c = not(Here_now_C2R_mvS3)  or Here_will_nobodyhome    or Here_will_R_mvS0;
	m.addClause(c); //


	
	c = not(Here_will_C2R_accE) or Here_now_C2R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C2R_mvE1) or Here_now_C2R_accE       or Here_now_C2R_mvE0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C2R_accE)  or Here_now_C2R_mvE0       or Here_will_C2R_mvE1;
	m.addClause(c);
	c = Here_now_C2R_accE       or not(Here_now_C2R_mvE0)  or Here_will_C2R_mvE1;
	m.addClause(c);
	c = not(Here_now_C2R_accE)  or not(Here_now_C2R_mvE0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C2R_mvE0) or W_now_C2R_mvE1;
	m.addClause(c); //
	c = not(Here_now_C2R_accE)  or Here_now_C2R_mvE0       or Here_will_C2R_mvE1;
	m.addClause(c);
	c = Here_now_C2R_accE       or not(Here_now_C2R_mvE0)  or Here_will_C2R_mvE1;
	m.addClause(c);
	c = not(Here_now_C2R_accE)  or not(Here_now_C2R_mvE0);
	m.addClause(c); //
	c = not(Here_now_C2R_mvE1)  or E_will_C2R_ready        or E_will_C2R_mvE0;
	m.addClause(c); //
	c = not(Here_now_C2R_mvE1)  or Here_will_nobodyhome    or Here_will_R_mvE0;
	m.addClause(c); //


	
	c = not(Here_will_C2R_accW) or Here_now_C2R_ready;
	m.addClause(c); //
	// neccessarly
	c = not(Here_will_C2R_mvW1) or Here_now_C2R_accW       or Here_now_C2R_mvW0;
	m.addClause(c);
	// sufficient
	c = not(Here_now_C2R_accW)  or Here_now_C2R_mvW0       or Here_will_C2R_mvW1;
	m.addClause(c);
	c = Here_now_C2R_accW       or not(Here_now_C2R_mvW0)  or Here_will_C2R_mvW1;
	m.addClause(c);
	c = not(Here_now_C2R_accW)  or not(Here_now_C2R_mvW0);
	m.addClause(c); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	c = not(Here_will_C2R_mvW0) or E_now_C2R_mvW1;
	m.addClause(c); //
	c = not(Here_now_C2R_accW)  or Here_now_C2R_mvW0       or Here_will_C2R_mvW1;
	m.addClause(c);
	c = Here_now_C2R_accW       or not(Here_now_C2R_mvW0)  or Here_will_C2R_mvW1;
	m.addClause(c);
	c = not(Here_now_C2R_accW)  or not(Here_now_C2R_mvW0);
	m.addClause(c); //
	c = not(Here_now_C2R_mvW1)  or W_will_C2R_ready        or W_will_C2R_mvW0;
	m.addClause(c); //
	c = not(Here_now_C2R_mvW1)  or Here_will_nobodyhome    or Here_will_R_mvW0;
	m.addClause(c); //
    }


    // L I F T I N G + D R O P P I N G
    { // Lifting process
        CNF::Var Here_now_empty            = var(v,       t,    On_Node::empty);
	CNF::Var Here_now_C0               = var(v,       t,    On_Node::Car0);
	CNF::Var Here_now_C1               = var(v,       t,    On_Node::Car1);
	CNF::Var Here_now_C2               = var(v,       t,    On_Node::Car2);

	CNF::Var Here_now_R_ready          = var(v,       t,    NdStat::R_ready);
	CNF::Var Here_now_R_lift           = var(v,       t,    R_Vertical::lift);
	CNF::Var Here_now_R_lifting1       = var(v,       t,    R_Vertical::l1);
	CNF::Var Here_now_R_lifting2       = var(v,       t,    R_Vertical::l2);
	CNF::Var Here_now_R_lifting3       = var(v,       t,    R_Vertical::l3);
	CNF::Var Here_now_R_lifting4       = var(v,       t,    R_Vertical::l4);

	CNF::Var Here_will_R_lift          = var(v,       t+1,  R_Vertical::lift);
	CNF::Var Here_will_R_lifting1      = var(v,       t+1,  R_Vertical::l1);
	CNF::Var Here_will_R_lifting2      = var(v,       t+1,  R_Vertical::l2);
	CNF::Var Here_will_R_lifting3      = var(v,       t+1,  R_Vertical::l3);
	CNF::Var Here_will_R_lifting4      = var(v,       t+1,  R_Vertical::l4);

	CNF::Var Here_will_C0R_ready       = var(v,       t+1,  NdStat::C0R_ready);
	CNF::Var Here_will_C1R_ready       = var(v,       t+1,  NdStat::C1R_ready);
	CNF::Var Here_will_C2R_ready       = var(v,       t+1,  NdStat::C2R_ready);
        // Abbreviation:
	CNF::Var Here_will_CR_ready;
	// neccessarly
	c = not(Here_will_CR_ready)  or Here_will_C0R_ready        or Here_will_C1R_ready      or Here_will_C2R_ready;
	m.addClause(c);
	// sufficient
	c = not(Here_will_C0R_ready) or Here_will_C1R_ready        or Here_will_C2R_ready      or Here_will_CR_ready;
	m.addClause(c);
	c = Here_will_C0R_ready      or not(Here_will_C1R_ready)   or Here_will_C2R_ready      or Here_will_CR_ready;
	m.addClause(c);
	c = Here_will_C0R_ready      or Here_will_C1R_ready        or not(Here_will_C2R_ready) or Here_will_CR_ready;
	m.addClause(c);
	c = not(Here_will_C0R_ready) or not(Here_will_C1R_ready);
	m.addClause(c);
	c = not(Here_will_C0R_ready) or not(Here_will_C2R_ready);
	m.addClause(c);
	c = not(Here_will_C1R_ready) or not(Here_will_C2R_ready);
	m.addClause(c);


	c = not(Here_will_R_lift)    or Here_now_R_ready;
	m.addClause(c); //

	c = not(Here_now_R_lift)     or Here_will_R_lifting1;
	m.addClause(c);
	c = Here_now_R_lift          or not(Here_will_R_lifting1);
	m.addClause(c); //
	c = not(Here_now_R_lifting1) or Here_will_R_lifting2;
	m.addClause(c);
	c = Here_now_R_lifting1      or not(Here_will_R_lifting2);
	m.addClause(c); //
	c = not(Here_now_R_lifting2) or Here_will_R_lifting3;
	m.addClause(c);
	c = Here_now_R_lifting2      or not(Here_will_R_lifting3);
	m.addClause(c); //
	c = not(Here_now_R_lifting3) or Here_will_R_lifting4;
	m.addClause(c);
	c = Here_now_R_lifting3      or not(Here_will_R_lifting4);
	m.addClause(c); //
	
	c = not(Here_will_CR_ready)  or Here_now_empty             or Here_now_R_lifting4;
	m.addClause(c); //

	c = not(Here_now_R_lifting4) or Here_will_C0R_ready        or Here_now_C1         or Here_now_C2;
	m.addClause(c);  // maybe make these lazy?!?  
	c = not(Here_now_R_lifting4) or Here_now_C0                or OR Here_now_C1      or Here_will_C2R_ready;
	m.addClause(c); //
	c = not(Here_now_R_lifting4) or Here_will_C0R_ready        or Here_now_C1         or Here_will_C2R_ready;
	m.addClause(c); //
	c = not(Here_now_R_lifting4) or Here_will_C0R_ready        or Here_will_C1R_ready or Here_now_C2;
	m.addClause(c); //
	c = not(Here_now_R_lifting4) or Here_now_C0                or Here_will_C1R_ready or Here_now_C2;
	m.addClause(c); //
	c = not(Here_now_R_lifting4) or Here_now_C0                or Here_will_C1R_ready or Here_will_C2R_ready;
	m.addClause(c); //
	c = not(Here_now_R_lifting4) or Here_will_C0R_ready        or Here_will_C1R_ready or Here_will_C2R_ready;
	m.addClause(c); //
    }
    { // Dropping process
        CNF::Var Here_now_C0R_ready        = var(v,       t,    NdStat::C0R_ready);
	CNF::Var Here_now_C1R_ready        = var(v,       t,    NdStat::C1R_ready);
	CNF::Var Here_now_C2R_ready        = var(v,       t,    NdStat::C2R_ready);
	CNF::Var Here_now_R_drop           = var(v,       t,    R_Vertical::drop);

	CNF::Var Here_will_R_drop          = var(v,       t+1,  R_Vertical::drop);
	CNF::Var Here_will_R_ready         = var(v,       t+1,  NdStat::R_ready);

	CNF::Var Here_will_empty           = var(v,       t+1,  On_Node::empty);
	CNF::Var Here_will_C0              = var(v,       t+1,  On_Node::Car0);
	CNF::Var Here_will_C1              = var(v,       t+1,  On_Node::Car1);
	CNF::Var Here_will_C2              = var(v,       t+1,  On_Node::Car2);



	c = not(Here_now_R_drop)     or Here_will_R_ready;
	m.addClause(c); //
	c = not(Here_will_C0R_ready) or Here_will_C1R_ready       or Here_will_C2R_ready      or Here_will_empty                   or Here_will_R_drop;
	m.addClause(c);
	c = Here_will_C0R_ready      or not(Here_will_C1R_ready)  or Here_will_C2R_ready      or Here_will_empty                   or Here_will_R_drop;
	m.addClause(c);
	c = Here_will_C0R_ready      or Here_will_C1R_ready       or not(Here_will_C2R_ready) or Here_will_empty                   or Here_will_R_drop;
	m.addClause(c);
	c = not(Here_will_C0R_ready) or not(Here_will_C1R_ready);
	m.addClause(c);
	c = not(Here_will_C0R_ready) or not(Here_will_C2R_ready);
	m.addClause(c);
        c = not(Here_will_C1R_ready) or not(Here_will_C2R_ready);
	m.addClause(c); //
	

	c = not(Here_now_R_drop)     or Here_will_C0              or Here_will_C1             or Here_will_C2;
	m.addClause(c); // maybe make these lazy?!?
	c = not(Here_now_R_drop)     or Here_now_C0R_ready        or Here_will_C1             or Here_will_C2;
	m.addClause(c); //
	c = not(Here_now_R_drop)     or Here_will_C0              or Here_will_C1             or Here_now_C2R_ready;
	m.addClause(c); //
	c = not(Here_now_R_drop)     or Here_now_C0R_ready        or Here_will_C1             or Here_now_C2R_ready;
	m.addClause(c); //
	c = not(Here_now_R_drop)     or Here_now_C0R_ready        or Here_now_C1R_ready       or Here_will_C2;
	m.addClause(c); //
	c = not(Here_now_R_drop)     or Here_will_C0              or Here_now_C1R_ready       or Here_will_C2;
	m.addClause(c); //
	c = not(Here_now_R_drop)     or Here_will_C0              or Here_now_C1R_ready       or Here_now_C2R_ready;
	m.addClause(c); //
	c = not(Here_now_R_drop)     or Here_now_C0R_ready        or Here_now_C1R_ready       or Here_now_C2R_ready;
	m.addClause(c); //
    }
    {// Lift/drop and  PARKED cars
        CNF::Var Here_now_R_lifting4  = var(v,       t,      R_Vertical::l4);
	CNF::Var Here_now_R_drop      = var(v,       t,      R_Vertical::drop);
	CNF::Var Here_now_empty       = var(v,       t,      On_Node::empty);
	CNF::Var Here_now_C0          = var(v,       t,      On_Node::Car0);
	CNF::Var Here_now_C1          = var(v,       t,      On_Node::Car1);
	CNF::Var Here_now_C2          = var(v,       t,      On_Node::Car2);

	CNF::Var Here_will_R_lifting4 = var(v,       t+1,    R_Vertical::l4);
	CNF::Var Here_will_R_drop     = var(v,       t+1,    R_Vertical::drop);
	CNF::Var Here_will_empty      = var(v,       t+1,    On_Node::empty);
	CNF::Var Here_will_C0         = var(v,       t+1,    On_Node::Car0);
	CNF::Var Here_will_C1         = var(v,       t+1,    On_Node::Car1);
	CNF::Var Here_will_C2         = var(v,       t+1,    On_Node::Car2);


	// neccessarly
	c = not(Here_now_R_lifting4) or Here_now_empty            or Here_will_R_drop         or Here_will_empty;
	m.addClause(c);
	c = Here_now_R_lifting4      or not(Here_now_empty)       or Here_will_R_drop         or Here_will_empty;
	m.addClause(c);
	c = not(Here_now_R_lifting4) or not(Here_now_empty);
	m.addClause(c);
	// sufficient
	c = not(Here_will_R_drop)    or Here_will_empty           or Here_now_R_lifting4      or Here_now_empty;
	m.addClause(c);
	c = Here_will_R_drop         or not(Here_will_empty)      or Here_now_R_lifting4      or Here_now_empty;
	m.addClause(c);
	c = not(Here_will_R_drop)    or not(Here_will_empty);
	m.addClause(c); //


	
	c = not(Here_now_R_drop)     or Here_will_R_drop          or not(Here_will_empty);
	m.addClause(c);
	c = Here_now_R_drop          or not(Here_will_R_drop)     or not(Here_will_empty);
	m.addClause(c);
	c = not(Here_now_R_drop)     or not(Here_will_R_drop);
	m.addClause(c); //
	c = not(Here_will_R_lifting4)or Here_now_R_lifting4       or not(Here_now_empty);
	m.addClause(c);
	c = Here_will_R_lifting4     or not(Here_now_R_lifting4)  or not(Here_now_empty);
	m.addClause(c);
	c = not(Here_will_R_lifting4)or not(Here_now_R_lifting4);
	m.addClause(c); //


	
	c = not(Here_will_R_drop)    or Here_now_empty;
	m.addClause(c); //
	c = not( Here_now_R_lifting4)or Here_will_empty;
	m.addClause(c); //


	
	c = not(Here_will_C0)        or Here_will_C1              or Here_now_C0              or Here_now_C1                       or Here_will_R_drop;
	m.addClause(c);
	c = Here_will_C0             or not(Here_will_C1)         or Here_now_C0              or Here_now_C1                       or Here_will_R_drop;
	m.addClause(c);
	c = not(Here_will_C0)        or not(Here_will_C1);
	m.addClause(c); //
	c = not(Here_will_C0)        or Here_will_C2              or Here_now_C0              or Here_now_C2                       or Here_will_R_drop;
	m.addClause(c);
	c = Here_will_C0             or not(Here_will_C2)         or Here_now_C0              or Here_now_C2                       or Here_will_R_drop;
	m.addClause(c);
	c = not(Here_will_C0)        or not(Here_will_C2);
	m.addClause(c); //
	c = not(Here_will_C1)        or Here_will_C2              or Here_now_C1              or Here_now_C2                       or Here_will_R_drop;
	m.addClause(c);
	c = Here_will_C1             or not(Here_will_C2)         or Here_now_C1              or Here_now_C2                       or Here_will_R_drop;
	m.addClause(c);
	c = not(Here_will_C1)        or not(Here_will_C2);
	m.addClause(c); //


	
	c = not(Here_now_C0)         or Here_now_C1               or Here_will_C0             or Here_will_C1                      or Here_will_R_lifting4;
	m.addClause(c);
	c = Here_now_C0              or not(Here_now_C1)          or Here_will_C0             or Here_will_C1                      or Here_will_R_lifting4;
	m.addClause(c);
	c = not(Here_now_C0)         or not(Here_now_C1);
	m.addClause(c); //
	c = not(Here_now_C0)         or Here_now_C2               or Here_will_C0             or Here_will_C2                      or Here_will_R_lifting4;
	m.addClause(c);
	c = Here_now_C0              or not(Here_now_C2)          or Here_will_C0             or Here_will_C2                      or Here_will_R_lifting4;
	m.addClause(c);
	c = not(Here_now_C0)         or not(Here_now_C2);
	m.addClause(c); //
	c = not(Here_now_C1)         or Here_now_C2               or Here_will_C1             or Here_will_C2                      or Here_will_R_lifting4;
	m.addClause(c);
	c = Here_now_C1              or not(Here_now_C2)          or Here_will_C1             or Here_will_C2                      or Here_will_R_lifting4;
	m.addClause(c);
	c = not(Here_now_C1)         or not(Here_now_C2);
	m.addClause(c); //


	
	c = Here_now_empty           or not(Here_will_empty)      or (Here_now_R_lifting4);
	m.addClause(c); //

    }

} // time_link_constraints()

// ************************************************************************************************************************
// * Callback function
// ************************************************************************************************************************

void GridSpace::Grid_Gurobi_Callback::callback() {
    switch (where) {
    case 0:
        break;
        std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: POLLING  Periodic polling callback"<<std::endl;
    case 1:
        break;
        std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: PRESOLVE Currently performing presolve"<<std::endl;
        std::cout<<"CALLBACK: Elapsed solver runtime (seconds): "<<getDoubleInfo(GRB_CB_RUNTIME)<<std::endl;
        std::cout<<"CALLBACK: The number of columns removed by presolve to this point: "<<getIntInfo(GRB_CB_PRE_COLDEL)<<std::endl;
        std::cout<<"CALLBACK: The number of rows removed by presolve to this point: "<<getIntInfo(GRB_CB_PRE_ROWDEL)<<std::endl;
        std::cout<<"CALLBACK: The number of constraint senses changed by presolve to this point: "<<getIntInfo(GRB_CB_PRE_SENCHG)<<std::endl;
        std::cout<<"CALLBACK: The number of variable bounds changed by presolve to this point: "<<getIntInfo(GRB_CB_PRE_BNDCHG)<<std::endl;
        std::cout<<"CALLBACK: The number of coefficients changed by presolve to this point: "<<getIntInfo(GRB_CB_PRE_COECHG)<<std::endl;
    case 2:
        break;
        std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: SIMPLEX  Currently in simplex"<<std::endl;
        std::cout<<"CALLBACK: Elapsed solver runtime (seconds): "<<getDoubleInfo(GRB_CB_RUNTIME)<<std::endl;
        std::cout<<"CALLBACK: Current simplex iteration count: "<<getDoubleInfo(GRB_CB_SPX_ITRCNT)<<std::endl;
        std::cout<<"CALLBACK: Current simplex objective value: "<<getDoubleInfo(GRB_CB_SPX_OBJVAL)<<std::endl;
        std::cout<<"CALLBACK: Current primal infeasibility: "<<getDoubleInfo(GRB_CB_SPX_PRIMINF)<<std::endl;
        std::cout<<"CALLBACK: Current dual infeasibility: "<<getDoubleInfo(GRB_CB_SPX_DUALINF)<<std::endl;
        std::cout<<"CALLBACK: Is problem current perturbed? "<<getIntInfo(GRB_CB_SPX_ISPERT)<<std::endl;
    case 3:
        break;
        std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: MIP      Currently in MIP"<<std::endl;
        std::cout<<"CALLBACK: Elapsed solver runtime (seconds): "<<getDoubleInfo(GRB_CB_RUNTIME)<<std::endl;
        std::cout<<"CALLBACK: Current best objective: "<<getDoubleInfo(GRB_CB_MIP_OBJBST)<<std::endl;
        std::cout<<"CALLBACK: Current best objective bound: "<<getDoubleInfo(GRB_CB_MIP_OBJBND)<<std::endl;
        std::cout<<"CALLBACK: Current explored node count: "<<getDoubleInfo(GRB_CB_MIP_NODCNT)<<std::endl;
        std::cout<<"CALLBACK: Current count of feasible solutions found: "<<getIntInfo(GRB_CB_MIP_SOLCNT)<<std::endl;
        std::cout<<"CALLBACK: Current count of cutting planes applied: "<<getIntInfo(GRB_CB_MIP_CUTCNT)<<std::endl;
        std::cout<<"CALLBACK: Current unexplored node count: "<<getDoubleInfo(GRB_CB_MIP_NODLFT)<<std::endl;
        std::cout<<"CALLBACK: Current simplex iteration count: "<<getDoubleInfo(GRB_CB_MIP_ITRCNT)<<std::endl;
    case 4:
        if ( my_daddy.my_opts.exit_when_better >= getDoubleInfo(GRB_CB_MIPSOL_OBJ) ) {
            std::cout<<"CALLBACK: found a solution which is at least as good as given parameter. Aborting."<<std::endl;
            abort();
        }
        if ( my_daddy.my_opts.early_exit ) {
            if (my_daddy.my_opts.hardwire) {
                std::cout<<"CALLBACK: found a solution with matching terminal state (hardwired). Aborting."<<std::endl;
                abort();
            } else if (terminal_status_matches()) {
                std::cout<<"CALLBACK: found a solution with matching terminal state (not hardwired). Aborting."<<std::endl;
                abort();
            }
        }
        break;
        std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: MIPSOL   Found a new MIP incumbent"<<std::endl;
        std::cout<<"CALLBACK: Elapsed solver runtime (seconds): "<<getDoubleInfo(GRB_CB_RUNTIME)<<std::endl;
        std::cout<<"CALLBACK: Objective value for new solution: "<<getDoubleInfo(GRB_CB_MIPSOL_OBJ)<<std::endl;
        std::cout<<"CALLBACK: Current best objective: "<<getDoubleInfo(GRB_CB_MIPSOL_OBJBST)<<std::endl;
        std::cout<<"CALLBACK: Current best objective bound: "<<getDoubleInfo(GRB_CB_MIPSOL_OBJBND)<<std::endl;
        std::cout<<"CALLBACK: Current explored node count: "<<getDoubleInfo(GRB_CB_MIPSOL_NODCNT)<<std::endl;
        std::cout<<"CALLBACK: Current count of feasible solutions found: "<<getIntInfo(GRB_CB_MIPSOL_SOLCNT)<<std::endl;
    case 5:
        break;
        std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: MIPNODE  Currently exploring a MIP node"<<std::endl;
        std::cout<<"CALLBACK: Elapsed solver runtime (seconds): "<<getDoubleInfo(GRB_CB_RUNTIME)<<std::endl;
        std::cout<<"CALLBACK: Optimization status of current MIP node: "<<getIntInfo(GRB_CB_MIPNODE_STATUS)<<std::endl;
        std::cout<<"CALLBACK: Optimization status is:";
        switch (getIntInfo(GRB_CB_MIPNODE_STATUS)) {
        case  1: std::cout<<" LOADED          Model is loaded, but no solution information is available.\n";
            break;
        case  2: std::cout<<" OPTIMAL         Model was solved to optimality (subject to tolerances), and an optimal solution is available.\n";
            break;
        case  3: std::cout<<" INFEASIBLE      Model was proven to be infeasible.\n";
            break;
        case  4: std::cout<<" INF_OR_UNBD     Model was proven to be either infeasible or unbounded. To obtain a more definitive conclusion, set the DualReductions parameter to 0 and reoptimize.\n";
            break;
        case  5: std::cout<<" UNBOUNDED       Model was proven to be unbounded. Important note: an unbounded status indicates the presence of an unbounded ray that allows the objective to improve without limit. It says nothing about whether the model has a feasible solution. If you require information on feasibility, you should set the objective to zero and reoptimize.\n";
            break;
        case  6: std::cout<<" CUTOFF          Optimal objective for model was proven to be worse than the value specified in the Cutoff parameter. No solution information is available.\n";
            break;
        case  7: std::cout<<" ITERATION_LIMIT Optimization terminated because the total number of simplex iterations performed exceeded the value specified in the IterationLimit parameter, or because the total number of barrier iterations exceeded the value specified in the BarIterLimit parameter.\n";
            break;
        case  8: std::cout<<" NODE_LIMIT      Optimization terminated because the total number of branch-and-cut nodes explored exceeded the value specified in the NodeLimit parameter.\n";
            break;
        case  9: std::cout<<" TIME_LIMIT      Optimization terminated because the time expended exceeded the value specified in the TimeLimit parameter.\n";
            break;
        case 10: std::cout<<" SOLUTION_LIMIT  Optimization terminated because the number of solutions found reached the value specified in the SolutionLimit parameter.\n";
            break;
        case 11: std::cout<<" INTERRUPTED     Optimization was terminated by the user.\n";
            break;
        case 12: std::cout<<" NUMERIC         Optimization was terminated due to unrecoverable numerical difficulties.\n";
            break;
        case 13: std::cout<<" SUBOPTIMAL      Unable to satisfy optimality tolerances; a sub-optimal solution is available.\n";
            break;
        case 14: std::cout<<" INPROGRESS      An asynchronous optimization call was made, but the associated optimization run is not yet complete.\n";
            break;
        default: std::cout<<" WEIRD           You shoulnd't be here!!!\n";
        }
        std::cout<<"CALLBACK: Current best objective: "<<getDoubleInfo(GRB_CB_MIPNODE_OBJBST)<<std::endl;
        std::cout<<"CALLBACK: Current best objective bound: "<<getDoubleInfo(GRB_CB_MIPNODE_OBJBND)<<std::endl;
        std::cout<<"CALLBACK: Current explored node count: "<<getDoubleInfo(GRB_CB_MIPNODE_NODCNT)<<std::endl;
        std::cout<<"CALLBACK: Current count of feasible solutions found: "<<getIntInfo(GRB_CB_MIPNODE_SOLCNT)<<std::endl;
    case 6:
        break;
        // std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: MESSAGE  Printing a log message"<<std::endl;
        // std::cout<<"CALLBACK: Elapsed solver runtime (seconds): "<<getDoubleInfo(GRB_CB_RUNTIME)<<std::endl;
        // std::cout<<"CALLBACK: The message that is being printed: "<<getStringInfo(GRB_CB_MSG_STRING)<<std::endl;
    case 7:
        break;
        std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: BARRIER  Currently in barrier"<<std::endl;
        std::cout<<"CALLBACK: Elapsed solver runtime (seconds): "<<getDoubleInfo(GRB_CB_RUNTIME)<<std::endl;
        std::cout<<"CALLBACK: Current barrier iteration count: "<<getIntInfo(GRB_CB_BARRIER_ITRCNT)<<std::endl;
        std::cout<<"CALLBACK: Primal objective value for current barrier iterate: "<<getDoubleInfo(GRB_CB_BARRIER_PRIMOBJ)<<std::endl;
        std::cout<<"CALLBACK: Dual objective value for current barrier iterate: "<<getDoubleInfo(GRB_CB_BARRIER_DUALOBJ)<<std::endl;
        std::cout<<"CALLBACK: Primal infeasibility for current barrier iterate: "<<getDoubleInfo(GRB_CB_BARRIER_PRIMINF)<<std::endl;
        std::cout<<"CALLBACK: Dual infeasibility for current barrier iterate: "<<getDoubleInfo(GRB_CB_BARRIER_DUALINF)<<std::endl;
        std::cout<<"CALLBACK: Complementarity violation for current barrier iterate: "<<getDoubleInfo(GRB_CB_BARRIER_COMPL)<<std::endl;
    default:
        std::cout<<"CALLBACK: HEY, GUROBI CALLED BACK: WEIRD   You're not supposed to be here!!!"<<std::endl;
    }
} //^ callback()


bool GridSpace::Grid_Gurobi_Callback::terminal_status_matches()
{
    for (short y=0; y<my_daddy.G.NS_sz(); ++y) {
        for (short x=0; x<my_daddy.G.EW_sz(); ++x) {
            XY xy {x,y};
            if ( my_daddy.G.exists(xy) )  {
                const Full_Stat s = (*my_daddy.p_terminal_state)[xy];

                if ( s.on_node!=On_Node::empty && ( !my_daddy.my_opts.ignore_C0 || s.on_node!=On_Node::Car0 ) ) {
                    for (    On_Node    i=begin_On_Node();    i!=end_On_Node();    ++i) {
                        const double RHS = (s.on_node==i ? 1 : 0 );
                        GRBLinExpr _x = my_daddy.var(xy,my_daddy.t_max,i);
                        if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_terminal_state(): There's something wrong with this On_Node variable (GRBLinExpr has !=1 terms).");
                        GRBVar x = _x.getVar(0);
                        if ( std::fabs(getSolution(x) - RHS) > .1 )   return false;
                    } //^ for stat
                } //^ if whether to ignore C0
                if (!my_daddy.my_opts.ignore_robots) {
                    for (NdStat     i=begin_NdStat();     i!=end_NdStat();     ++i) {
                        const double RHS = (s.ndstat==i ? 1 : 0 );
                        GRBLinExpr _x = my_daddy.var(xy,my_daddy.t_max,i);
                        if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_terminal_state(): There's something wrong with this NdStat variable (GRBLinExpr has !=1 terms).");
                        GRBVar x = _x.getVar(0);
                        if ( std::fabs(getSolution(x) - RHS) > .1 )   return false;
                    } //^ for
                    for (R_Vertical i=begin_R_Vertical(); i!=end_R_Vertical(); ++i) {
                        const double RHS = (s.r_vert==i ? 1 : 0 );
                        GRBLinExpr _x = my_daddy.var(xy,my_daddy.t_max,i);
                        if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_terminal_state(): There's something wrong with this R_Vertical variable (GRBLinExpr has !=1 terms).");
                        GRBVar x = _x.getVar(0);
                        if ( std::fabs(getSolution(x) - RHS) > .1 )   return false;
                    } //^ for
                    for (R_Move       i=begin_R_Move();       i!=end_R_Move();       ++i) {
                        const double RHS = (s.r_mv==i ? 1 : 0 );
                        double val = 0.;
                        const Direction d = get_direction(i);
                        if ( my_daddy.G.move(xy,d)!=nowhere ) {
                            GRBLinExpr _x = my_daddy.var(xy,my_daddy.t_max,i);
                            if (_x.size() != 1) throw std::runtime_error("Grid_Gurobi::set_terminal_state(): There's something wrong with this R_Move variable (GRBLinExpr has !=1 terms).");
                            GRBVar x = _x.getVar(0);
                            val = getSolution(x);
                        }
                        if ( std::fabs(val - RHS) > .1 )   return false;
                    } //^ for
                } // if (do robots)
            } // if exists
        } // for x
    } // for y
    return true;
} //^ terminal_status_matches()



// EOF grid_gurobi.cc
