#include "CNF.hh"

#include <stdexcept>

void CNF::Model::addClause(const Clause & c)
{
    for (auto v=c.c.begin(); v!=c.c.end(); ++v)  {
        if (abs(v->i) < 1  ||  abs(v->i) > n_vars)  throw std::runtime_error("CNF::addClause(): clause contains non-existing variable.");
        f += std::to_string(v->i) + " ";
    }
    f += " 0 \n";
}
