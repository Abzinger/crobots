// grid_properties.hh C++11
// Part of the robots project
// Author: Dirk Oliver Theis
#ifndef __ROBOT__GRID_PROPERTIES_HH__
#define __ROBOT__GRID_PROPERTIES_HH__

#include "grid.hh"

namespace GridSpace {

    bool grid_connected( const Grid & G );

    bool grid_fullsize( const Grid & G ); // outermost rows/columns are in use


} //^ namespace

#endif
// ^grid_properties.hh EOF
