#include "CNF.hh"

#include <stdexcept>

void CNF::Model::addClause(const Clause & c)
{
    for (auto v=c.c.begin(); v!=c.c.end(); ++v)  {
        if (abs(v->i) < 1  ||  abs(v->i) > n_vars)  throw std::runtime_error("CNF::Model::addClause(): clause contains non-existing variable.");
        f += std::to_string(v->i) + " ";
    }
    f += " 0 \n";
}


void CNF::Model::read_DIMACS(std::istream & in) {
    the_value.resize(n_vars,(char)-1);

    int  i;
    int  abs_i;
    while (true) {
        if (in.peek()=='v')  in.get();
        in >> i;
        abs_i = std::abs(i);
        if (abs_i < 1 || abs_i > n_vars) throw std::string("CNF::Model::read()_DIMACS: Syntax error: Variable index out of range");
        the_value[abs_i] = (bool)( abs_i==i );
    } //^ while
    the_value[0] = 1;

    for (int i=0; i<n_vars; ++i)  if (the_value[i]<0 || the_value[i]>1)  throw std::string("CNF::Model::read()_DIMACS: Something went wrong, maybe there's a missing variable.");
} //^ read_DIMACS()
