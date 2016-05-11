// example_grids.hh C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#ifndef __ROBOT__EXAMPLE_GRIDS_HH__
#define __ROBOT__EXAMPLE_GRIDS_HH__

#include "grid.hh"

namespace GridSpace {

    struct Marsi3A_ExampleGrid: public Grid {
        Marsi3A_ExampleGrid();
    };

    struct Marsi3B_ExampleGrid: public Grid {
        Marsi3B_ExampleGrid();
    };

    struct Marsi3C_ExampleGrid: public Grid {
        Marsi3C_ExampleGrid();
    };

    struct Marsi3D_ExampleGrid: public Grid {
        Marsi3D_ExampleGrid();
    };

    struct Marsi3E_ExampleGrid: public Grid {
        Marsi3E_ExampleGrid();
    };
} //^ namespace

#endif
// EOF grid.hh
