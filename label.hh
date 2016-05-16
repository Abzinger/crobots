// label.hh C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#ifndef __ROBOT__LABEL_HH__
#define __ROBOT__LABEL_HH__

#include "grid.hh"
#include "grid_stat.hh"

#include <stdexcept>
#include <vector>

//
// Robots are labeled {1,...,R}.  Robot pseudo-label 0 means "no Robot"
//

namespace GridSpace {

    struct Label
    {
        const Grid & G;

        Label(const Grid & _G, std::vector< GridSpace::Stat_Vector_t > && _stat): G(_G),stat(_stat) {label_it();}


    protected:
        std::vector< GridSpace::Stat_Vector_t > stat;

        unsigned R; // numo robots

        const XY & location_of_robot(unsigned t, unsigned short r)   const { return locrob[t*R+r-1]; } // "-1" is needed because robot indices are in {1,...,R}
        XY &       location_of_robot(unsigned t, unsigned short r)         { return locrob[t*R+r-1]; } 

    private:
        typedef Grid::vector_grid< unsigned short >     Robvec_t;
        typedef std::vector< Robvec_t >                 RobTimeVec_t;
        RobTimeVec_t                                    rob;

        std::vector< XY >                               locrob;

        void label_it();
    };

} // namespace Grid


#endif
    // ^label.hh EOF
