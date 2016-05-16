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

#include <set>
#include <string>

namespace GridSpace {

    class Grid_Gurobi_Callback;

    struct My_Options {
        void set_ignore_robots(bool val) {ignore_robots=val;}
        void set_ignore_C0(bool val) {ignore_C0=val;}
        void set_hardwire(bool val) {hardwire=val;}
        void set_punish_mismatch(bool val) {punish_mismatch=val;}
        void set_early_exit(bool val) {early_exit=val;}
        void set_exit_when_better(double val) {exit_when_better=val;}
    private:
        friend class Grid_Gurobi;
        friend class Grid_Gurobi_Callback;

        My_Options(): ignore_robots(false), ignore_C0(false), hardwire(false), punish_mismatch(false), early_exit{false}, exit_when_better{-1.e+99} {}
        bool ignore_robots;
        bool ignore_C0;
        bool hardwire;
        bool punish_mismatch;
        bool early_exit;
        double exit_when_better;
    };

    class Grid_Gurobi
    {
        friend class Grid_Gurobi_Callback;
    public:
	Grid_Gurobi(const Grid & _G, unsigned t_max);
	~Grid_Gurobi();

	const Grid & G;

	void set_initial_state (const Stat_Vector_t *);  // STORES THE POINTER!! DON'T DELETE VECTOR UNTIL WHEN YOU DELETE *THIS!
	void set_terminal_state(const Stat_Vector_t *);  // STORES THE POINTER!! DON'T DELETE VECTOR UNTIL WHEN YOU DELETE *THIS!

        void                          set_GRBparameter(std::string, double);
        void                          set_GRBparameter(std::string, int);
        static std::set<std::string>  list_GRBparameters();

        My_Options & options() { return my_opts; }


	void                          optimize();
        std::vector< Stat_Vector_t >  get_solution() const;

    protected:
	GRBLinExpr var(XY, unsigned t, On_Node    )   const;
	GRBLinExpr var(XY, unsigned t, NdStat     )   const;
	GRBLinExpr var(XY, unsigned t, R_Vertical )   const;
	GRBLinExpr var(XY, unsigned t, R_Move     )   const;

        My_Options my_opts;

    private:
	const unsigned t_max;

	GRBEnv    * const p_genv;
	GRBModel  * const p_model;
	GRBModel & model;

        const Stat_Vector_t *p_initial_state;
        const Stat_Vector_t *p_terminal_state;

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
