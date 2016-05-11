// grid_gurobi.hh C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#ifndef __ROBOT__GRID_GUROBI_HH__
#define __ROBOT__GRID_GUROBI_HH__
#include "grid_stat.hh"


class GRBEnv;
class GRBModel;
class GRBLinExpr;
class GRBVar;

namespace GridSpace {

    class Grid_Gurobi_Callback;

    class Grid_Gurobi
    {
        friend class Grid_Gurobi_Callback;
    public:
	Grid_Gurobi(const Grid & _G, unsigned t_max);
	~Grid_Gurobi();

	const Grid & G;

	enum class Ignore_Robots_In_Terminal_State { Yes, No };
	void set_initial_state (const Stat_Vector_t &);                                                                                    // set state at t=0
	void set_terminal_state(const Stat_Vector_t &, Ignore_Robots_In_Terminal_State ignore_robots=Ignore_Robots_In_Terminal_State::No); // set state at t=t_max



	void                          optimize(double heuristics=.75, int n_threads=1, double time_limit=60., int n_sols=0);
        std::vector< Stat_Vector_t >  get_solution() const;

    protected:
	GRBLinExpr var(XY, unsigned t, On_Node    )   const;
	GRBLinExpr var(XY, unsigned t, NdStat     )   const;
	GRBLinExpr var(XY, unsigned t, R_Vertical )   const;
	GRBLinExpr var(XY, unsigned t, R_Move     )   const;

    private:
	const unsigned t_max;

	GRBEnv    * const p_genv;
	GRBModel  * const p_model;
	GRBModel & model;

	bool       initial_state_has_been_set;
	bool       terminal_state_has_been_set;



	typedef std::vector<GRBVar*> nodevariables_t;

	typename Grid::vector_grid<nodevariables_t> onnode_vars;      // delete[] the entries of this to delete all the GRBVar*-arrays in nodevariables_t.
	typename Grid::vector_grid<nodevariables_t> ndstat_vars;
	typename Grid::vector_grid<nodevariables_t> rvertical_vars;
	typename Grid::vector_grid<nodevariables_t> rmv_vars;


	void make_model();                                  // create the model, by calling the following fns:

	void make_vars();                                   // create + add all variables, by calling the following fn:
	void atom_vars(XY, unsigned t);

	void make_constraints     ();                       // create + add all constraints, by calling the following 2 fns:
	void atom_constraints     (XY, unsigned t);   // create + add all constraints involing only time t
	void time_link_constraints(XY, unsigned t);   // create + add each constraint which involes both time t and t+1

    }; // class Grid_Gurobi

} // namespace Gridstat

#endif
// EOF grid_gurobi.hh
