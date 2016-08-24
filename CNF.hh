// CNF.hh C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#ifndef __ROBOT__CNF_HH__
#define __ROBOT__CNF_HH__

#include <string>
#include <forward_list>
#include <iostream>

namespace CNF {

    // ****************************************************************************************************
    // Variables
    class Var
    {
        friend class Model;
        friend Var operator not(Var);
        int i;
    public:
        struct One {};
        Var():    i(0) {}
        Var(One): i(1) {}
    };

    Var One{ Var::One{} };
    Var Zero = not(One);

    inline
    Var operator not(Var v)                                    { Var not_v; not_v.i=-v.i; return not_v; }


    // ****************************************************************************************************
    // Clauses

    struct Clause
    {
        friend class Model;
        friend Clause operator or (Var,Var);
        Clause & operator = (Var v)                            { c.clear(); c.push_front(v); return *this; }

        Clause & operator or (Var v)                           { c.push_front(v); return *this; }
    private:
        std::forward_list< Var > c;
    };

    Clause operator or (Var u, Var v)                          { Clause c; c.c.push_front(u); c.c.push_front(v); return c; }

    // ****************************************************************************************************
    // Formulas

    class Model
    {
        int         n_vars;
        unsigned    n_clauses;
        std::string f;
    public:
        Model(): n_vars(1), n_clauses(0) {}

        // Variables:
        Var addVar()                                           { Var v; v.i = ++n_vars;  return v; }

        // Clauses
        void addClause(const Clause & c);

        // Output
        void dump(std::ostream & out)                          { out<<"p "<<n_vars<<' '<<n_clauses<<'\n'<<One.i<<" 0\n"<<f; }
    };

} //^ namespace


// using namespace CNF;
// Model m;
// Var x1 = m.addVar();
// Var x2 = m.addVar();
// Var x3 = m.addVar();
// Var x4 = m.addVar();
// Var x7 = m.addVar();
//
// Clause c;
// c = x1 or not(x2) or x3;
// m.addClause(c);
// c = x3 or x4 or not(x7) or Zero;
// m.addClause(c);
// m.dump(std::out);

#endif
// ^CNF.hh EOF