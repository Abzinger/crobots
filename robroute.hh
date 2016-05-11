// robroute.hh C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#ifndef __ROBROUTE_HH__
#define __ROBROUTE_HH__

#include "grid.hh"
#include "grid_stat.hh"

#include <iostream>
#include <vector>
#include <string>

namespace GridSpace {

    void write_robroute(std::ostream                                  & out,
                        const std::vector< GridSpace::Stat_Vector_t > & );

    GridSpace::Grid * read_robroute(std::istream                            & in,
                                    std::vector< GridSpace::Stat_Vector_t > * p_vec,
                                    std::string                             * p_comments);
    // Allocates memory for the grid which is returned; caller takes ownership of that memory (i.e., has to free it)!
    // the vector is first clear()ed then resize()d.

} // namespace Grid


#endif //^ def
// ^robroute.hh EOF
