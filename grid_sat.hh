// grid_sat.hh C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#ifndef __ROBOT__GRID_SAT_HH__
#define __ROBOT__GRID_SAT_HH__
#include "grid_stat.hh"


#include "CNF.hh"

#include <set>
#include <string>

namespace GridSpace {

    struct My_Options {
        void set_ignore_robots(bool val) {ignore_robots=val;}
        void set_ignore_C0(bool val) {ignore_C0=val;}
    private:
        friend class Grid_Sat;

        My_Options(): ignore_robots(false), ignore_C0(false) {}
        bool ignore_robots;
        bool ignore_C0;
    };

    class Grid_Sat
    {
    public:
	Grid_Sat(const Grid & _G, unsigned t_max);
	~Grid_Sat();

	const Grid & G;

	void set_initial_state (const Stat_Vector_t *);  // STORES THE POINTER!! DON'T DELETE VECTOR UNTIL WHEN YOU DELETE *THIS!
	void set_terminal_state(const Stat_Vector_t *);  // STORES THE POINTER!! DON'T DELETE VECTOR UNTIL WHEN YOU DELETE *THIS!

        My_Options & options() { return my_opts; }


	void                          optimize();
        std::vector< Stat_Vector_t >  get_solution() const;

    protected:
	CNF::Var var(XY, unsigned t, On_Node    )   const;      std::string var(XY, unsigned t, On_Node    )   const;
	CNF::Var var(XY, unsigned t, NdStat     )   const;      std::string var(XY, unsigned t, NdStat     )   const;
	CNF::Var var(XY, unsigned t, R_Vertical )   const;      std::string var(XY, unsigned t, R_Vertical )   const;
	CNF::Var var(XY, unsigned t, R_Move     )   const;      std::string var(XY, unsigned t, R_Move     )   const;

        My_Options my_opts;

      int has_solution;
     
    private:
	const unsigned t_max;

        CNF::Model  * const p_model;
        CNF::Model & model;

        const Stat_Vector_t *p_initial_state;
        const Stat_Vector_t *p_terminal_state;

	typedef std::vector<CNF::Var*> nodevariables_t;

	typename Grid::vector_grid<nodevariables_t> onnode_vars;      // delete[] the entries of this to delete all the NF::Var-arrays in nodevariables_t.
	typename Grid::vector_grid<nodevariables_t> ndstat_vars;
	typename Grid::vector_grid<nodevariables_t> rvertical_vars;
	typename Grid::vector_grid<nodevariables_t> rmv_vars;


	void make_model();                                  // create the model, by calling the following fns:

	void make_vars();                                   // create + add all variables, by calling the following fn:
	void atom_vars(XY, unsigned t);

	void make_constraints     ();                       // create + add all constraints, by calling the following 2 fns:
	void atom_constraints     (XY, unsigned t);   // create + add all constraints involing only time t
	void time_link_constraints(XY, unsigned t);   // create + add each constraint which involes both time t and t+1

    }; // class Grid_Sat

} // namespace Gridstat

#endif
// EOF grid_sat.hh
