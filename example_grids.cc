// example_grids.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis

#include "example_grids.hh"


GridSpace::Marsi3A_ExampleGrid::Marsi3A_ExampleGrid():
    Grid{4,5}
{
    data[idx(0,3)] = Node_type({                                 Direction::east});
    data[idx(1,3)] = Node_type({Direction::south,Direction::west,Direction::east});
    data[idx(2,3)] = Node_type({Direction::south,Direction::west,Direction::east});
    data[idx(3,3)] = Node_type({Direction::south,Direction::west,Direction::east});
    data[idx(4,3)] = Node_type({                 Direction::west});


    data[idx(0,2)] = Node_type({                                                  Direction::east});
    data[idx(1,2)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(2,2)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(3,2)] = Node_type({Direction::north,Direction::south,Direction::west});
    data[idx(4,2)] = Node_type({});

    data[idx(0,1)] = Node_type({                                                  Direction::east});
    data[idx(1,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(2,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(3,1)] = Node_type({Direction::north,Direction::south,Direction::west});
    data[idx(4,1)] = Node_type({});

    data[idx(0,0)] = Node_type({                                 Direction::east});
    data[idx(1,0)] = Node_type({Direction::north,Direction::west,Direction::east});
    data[idx(2,0)] = Node_type({Direction::north,Direction::west,Direction::east});
    data[idx(3,0)] = Node_type({Direction::north,Direction::west});
    data[idx(4,0)] = Node_type({});

    check_Grid_consistency(*this, "Marsi3A-constructor");
} // Marsi3A_ExampleGrid ---constructor

GridSpace::Marsi3B_ExampleGrid::Marsi3B_ExampleGrid(): Grid{4,4}
{
    data[idx(0,0)] = Node_type({                 Direction::east});
    data[idx(1,0)] = Node_type({Direction::north,Direction::west,Direction::east});
    data[idx(2,0)] = Node_type({Direction::north,Direction::west,Direction::east});
    data[idx(3,0)] = Node_type({Direction::north,                Direction::west});

    data[idx(0,1)] = Node_type({                                  Direction::east});
    data[idx(1,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(2,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(3,1)] = Node_type({Direction::north,Direction::south,                Direction::west});

    data[idx(0,2)] = Node_type({                                  Direction::east});
    data[idx(1,2)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(2,2)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(3,2)] = Node_type({Direction::north,Direction::south,                Direction::west});

    data[idx(0,3)] = Node_type({                 Direction::east});
    data[idx(1,3)] = Node_type({Direction::south,Direction::west,Direction::east});
    data[idx(2,3)] = Node_type({Direction::south,Direction::west,Direction::east});
    data[idx(3,3)] = Node_type({Direction::south,                Direction::west});

    check_Grid_consistency(*this, "Marsi3B-constructor");
} // Marsi3B_ExampleGrid ---constructor

GridSpace::Marsi3C_ExampleGrid::Marsi3C_ExampleGrid(): Grid{4,11}
{
    data[idx( 0,3)] = Node_type({                 Direction::south,Direction::east                });
    data[idx( 1,3)] = Node_type({                                  Direction::west,Direction::east});
    data[idx( 2,3)] = Node_type({                 Direction::south,                Direction::west});
    data[idx( 3,3)] = Node_type({});
    data[idx( 4,3)] = Node_type({});
    data[idx( 5,3)] = Node_type({});
    data[idx( 6,3)] = Node_type({});
    data[idx( 7,3)] = Node_type({});
    data[idx( 8,3)] = Node_type({});
    data[idx( 9,3)] = Node_type({});
    data[idx(10,3)] = Node_type({});

    data[idx( 0,2)] = Node_type({Direction::north,Direction::south,Direction::east                });
    data[idx( 1,2)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx( 2,2)] = Node_type({Direction::north,Direction::south,                Direction::west});
    data[idx( 3,2)] = Node_type({                 Direction::south,Direction::east                });
    data[idx( 4,2)] = Node_type({                 Direction::south,                Direction::west});
    data[idx( 5,2)] = Node_type({                 Direction::south,Direction::east                });
    data[idx( 6,2)] = Node_type({                 Direction::south,                Direction::west});
    data[idx( 7,2)] = Node_type({                 Direction::south,Direction::east                });
    data[idx( 8,2)] = Node_type({                 Direction::south,                Direction::west});
    data[idx( 9,2)] = Node_type({                 Direction::south,Direction::east                });
    data[idx(10,2)] = Node_type({                 Direction::south,                Direction::west});

    data[idx( 0,1)] = Node_type({Direction::north,Direction::south,Direction::east                });
    data[idx( 1,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx( 2,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx( 3,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx( 4,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx( 5,1)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx( 6,1)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx( 7,1)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx( 8,1)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx( 9,1)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx(10,1)] = Node_type({Direction::north,                                 Direction::west});

    data[idx( 0,0)] = Node_type({Direction::north,                 Direction::east                });
    data[idx( 1,0)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx( 2,0)] = Node_type({Direction::north,                                 Direction::west});
    data[idx( 3,0)] = Node_type({Direction::north,                 Direction::east                });
    data[idx( 4,0)] = Node_type({Direction::north,                                 Direction::west});
    data[idx( 5,0)] = Node_type({});
    data[idx( 6,0)] = Node_type({});
    data[idx( 7,0)] = Node_type({});
    data[idx( 8,0)] = Node_type({});
    data[idx( 9,0)] = Node_type({});
    data[idx(10,0)] = Node_type({});

    check_Grid_consistency(*this, "Marsi3C-constructor");
} // Marsi3C_ExampleGrid ---constructor

GridSpace::Marsi3D_ExampleGrid::Marsi3D_ExampleGrid(): Grid{10,7}
{
    data[idx(0,9)] = Node_type({});
    data[idx(1,9)] = Node_type({});
    data[idx(2,9)] = Node_type({});
    data[idx(3,9)] = Node_type({});
    data[idx(4,9)] = Node_type({                 Direction::south,                Direction::east});
    data[idx(5,9)] = Node_type({                                  Direction::west,Direction::east});
    data[idx(6,9)] = Node_type({                 Direction::south,Direction::west                });

    data[idx(0,8)] = Node_type({});
    data[idx(1,8)] = Node_type({});
    data[idx(2,8)] = Node_type({});
    data[idx(3,8)] = Node_type({});
    data[idx(4,8)] = Node_type({Direction::north,Direction::south                                });
    data[idx(5,8)] = Node_type({});
    data[idx(6,8)] = Node_type({Direction::north,Direction::south                                });

    data[idx(0,7)] = Node_type({});
    data[idx(1,7)] = Node_type({});
    data[idx(2,7)] = Node_type({});
    data[idx(3,7)] = Node_type({});
    data[idx(4,7)] = Node_type({Direction::north,Direction::south,                Direction::east});
    data[idx(5,7)] = Node_type({                                  Direction::west,Direction::east});
    data[idx(6,7)] = Node_type({Direction::north,Direction::south,Direction::west                });

    data[idx(0,6)] = Node_type({});
    data[idx(1,6)] = Node_type({});
    data[idx(2,6)] = Node_type({});
    data[idx(3,6)] = Node_type({});
    data[idx(4,6)] = Node_type({Direction::north,Direction::south,                Direction::east});
    data[idx(5,6)] = Node_type({                                  Direction::west,Direction::east});
    data[idx(6,6)] = Node_type({Direction::north,Direction::south,Direction::west                });

    data[idx(0,5)] = Node_type({});
    data[idx(1,5)] = Node_type({});
    data[idx(2,5)] = Node_type({});
    data[idx(3,5)] = Node_type({});
    data[idx(4,5)] = Node_type({Direction::north,Direction::south                                });
    data[idx(5,5)] = Node_type({});
    data[idx(6,5)] = Node_type({Direction::north,Direction::south                                });

    data[idx(0,4)] = Node_type({});
    data[idx(1,4)] = Node_type({});
    data[idx(2,4)] = Node_type({});
    data[idx(3,4)] = Node_type({});
    data[idx(4,4)] = Node_type({Direction::north,                                 Direction::east});
    data[idx(5,4)] = Node_type({                                  Direction::west,Direction::east});
    data[idx(6,4)] = Node_type({Direction::north,Direction::south,Direction::west                });

    data[idx(0,3)] = Node_type({                                                  Direction::east});
    data[idx(1,3)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx(2,3)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx(3,3)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx(4,3)] = Node_type({                                  Direction::west,Direction::east});
    data[idx(5,3)] = Node_type({                                  Direction::west,Direction::east});
    data[idx(6,3)] = Node_type({Direction::north,                 Direction::west                });

    data[idx(0,2)] = Node_type({                                                  Direction::east});
    data[idx(1,2)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(2,2)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(3,2)] = Node_type({Direction::north,Direction::south,Direction::west                });
    data[idx(4,2)] = Node_type({});
    data[idx(5,2)] = Node_type({});
    data[idx(6,2)] = Node_type({});

    data[idx(0,1)] = Node_type({                                                  Direction::east});
    data[idx(1,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(2,1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx(3,1)] = Node_type({Direction::north,Direction::south,Direction::west                });
    data[idx(4,1)] = Node_type({});
    data[idx(5,1)] = Node_type({});
    data[idx(6,1)] = Node_type({});

    data[idx(0,0)] = Node_type({                                                  Direction::east});
    data[idx(1,0)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx(2,0)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx(3,0)] = Node_type({Direction::north,                 Direction::west                });
    data[idx(4,0)] = Node_type({});
    data[idx(5,0)] = Node_type({});
    data[idx(6,0)] = Node_type({});

    check_Grid_consistency(*this, "Marsi3D-constructor");
}  // Marsi3D_ExampleGrid ---constructor

GridSpace::Marsi3E_ExampleGrid::Marsi3E_ExampleGrid(): Grid{10,13}
{
    data[idx( 0, 9)] = Node_type({});
    data[idx( 1, 9)] = Node_type({                 Direction::south,                Direction::east});
    data[idx( 2, 9)] = Node_type({                                  Direction::west,Direction::east});
    data[idx( 3, 9)] = Node_type({                 Direction::south,Direction::west                });
    data[idx( 4, 9)] = Node_type({});
    data[idx( 5, 9)] = Node_type({});
    data[idx( 6, 9)] = Node_type({});
    data[idx( 7, 9)] = Node_type({});
    data[idx( 8, 9)] = Node_type({});
    data[idx( 9, 9)] = Node_type({});
    data[idx(10, 9)] = Node_type({});
    data[idx(11, 9)] = Node_type({});
    data[idx(12, 9)] = Node_type({});

    data[idx( 0, 8)] = Node_type({});
    data[idx( 1, 8)] = Node_type({Direction::north,Direction::south                                });
    data[idx( 2, 8)] = Node_type({});
    data[idx( 3, 8)] = Node_type({Direction::north,Direction::south                                });
    data[idx( 4, 8)] = Node_type({});
    data[idx( 5, 8)] = Node_type({});
    data[idx( 6, 8)] = Node_type({});
    data[idx( 7, 8)] = Node_type({});
    data[idx( 8, 8)] = Node_type({});
    data[idx( 9, 8)] = Node_type({});
    data[idx(10, 8)] = Node_type({});
    data[idx(11, 8)] = Node_type({});
    data[idx(12, 8)] = Node_type({});

    data[idx( 0, 7)] = Node_type({});
    data[idx( 1, 7)] = Node_type({Direction::north,Direction::south,                Direction::east});
    data[idx( 2, 7)] = Node_type({                                  Direction::west,Direction::east});
    data[idx( 3, 7)] = Node_type({Direction::north,Direction::south,Direction::west                });
    data[idx( 4, 7)] = Node_type({});
    data[idx( 5, 7)] = Node_type({});
    data[idx( 6, 7)] = Node_type({});
    data[idx( 7, 7)] = Node_type({});
    data[idx( 8, 7)] = Node_type({});
    data[idx( 9, 7)] = Node_type({});
    data[idx(10, 7)] = Node_type({});
    data[idx(11, 7)] = Node_type({});
    data[idx(12, 7)] = Node_type({});

    data[idx( 0, 6)] = Node_type({});
    data[idx( 1, 6)] = Node_type({Direction::north,Direction::south,                Direction::east});
    data[idx( 2, 6)] = Node_type({                                  Direction::west,Direction::east});
    data[idx( 3, 6)] = Node_type({Direction::north,Direction::south,Direction::west                });
    data[idx( 4, 6)] = Node_type({});
    data[idx( 5, 6)] = Node_type({});
    data[idx( 6, 6)] = Node_type({});
    data[idx( 7, 6)] = Node_type({});
    data[idx( 8, 6)] = Node_type({});
    data[idx( 9, 6)] = Node_type({});
    data[idx(10, 6)] = Node_type({});
    data[idx(11, 6)] = Node_type({});
    data[idx(12, 6)] = Node_type({});

    data[idx( 0, 5)] = Node_type({});
    data[idx( 1, 5)] = Node_type({Direction::north,Direction::south                                });
    data[idx( 2, 5)] = Node_type({});
    data[idx( 3, 5)] = Node_type({Direction::north,Direction::south                                });
    data[idx( 4, 5)] = Node_type({});
    data[idx( 5, 5)] = Node_type({});
    data[idx( 6, 5)] = Node_type({});
    data[idx( 7, 5)] = Node_type({});
    data[idx( 8, 5)] = Node_type({});
    data[idx( 9, 5)] = Node_type({});
    data[idx(10, 5)] = Node_type({});
    data[idx(11, 5)] = Node_type({});
    data[idx(12, 5)] = Node_type({});

    data[idx( 0, 4)] = Node_type({});
    data[idx( 1, 4)] = Node_type({Direction::north,Direction::south,                Direction::east});
    data[idx( 2, 4)] = Node_type({                                  Direction::west,Direction::east});
    data[idx( 3, 4)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx( 4, 4)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx( 5, 4)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx( 6, 4)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx( 7, 4)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx( 8, 4)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx( 9, 4)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx(10, 4)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx(11, 4)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx(12, 4)] = Node_type({                 Direction::south,Direction::west                });

    data[idx( 0, 3)] = Node_type({});
    data[idx( 1, 3)] = Node_type({Direction::north,Direction::south,                Direction::east});
    data[idx( 2, 3)] = Node_type({                 Direction::south,Direction::west,Direction::east});
    data[idx( 3, 3)] = Node_type({                                  Direction::west                });
    data[idx( 4, 3)] = Node_type({Direction::north,                                 Direction::east});
    data[idx( 5, 3)] = Node_type({Direction::north,                 Direction::west                });
    data[idx( 6, 3)] = Node_type({Direction::north,                                                });
    data[idx( 7, 3)] = Node_type({Direction::north,                                 Direction::east});
    data[idx( 8, 3)] = Node_type({Direction::north,                 Direction::west                });
    data[idx( 9, 3)] = Node_type({Direction::north,                                 Direction::east});
    data[idx(10, 3)] = Node_type({Direction::north,                 Direction::west                });
    data[idx(11, 3)] = Node_type({Direction::north,                                 Direction::east});
    data[idx(12, 3)] = Node_type({Direction::north,                 Direction::west                });

    data[idx( 0, 2)] = Node_type({                 Direction::south,                Direction::east});
    data[idx( 1, 2)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx( 2, 2)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx( 3, 2)] = Node_type({                                  Direction::west                });
    data[idx( 4, 2)] = Node_type({});
    data[idx( 5, 2)] = Node_type({});
    data[idx( 6, 2)] = Node_type({});
    data[idx( 7, 2)] = Node_type({});
    data[idx( 8, 2)] = Node_type({});
    data[idx( 9, 2)] = Node_type({});
    data[idx(10, 2)] = Node_type({});
    data[idx(11, 2)] = Node_type({});
    data[idx(12, 2)] = Node_type({});

    data[idx( 0, 1)] = Node_type({Direction::north,Direction::south,                Direction::east});
    data[idx( 1, 1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx( 2, 1)] = Node_type({Direction::north,Direction::south,Direction::west,Direction::east});
    data[idx( 3, 1)] = Node_type({                                  Direction::west                });
    data[idx( 4, 1)] = Node_type({});
    data[idx( 5, 1)] = Node_type({});
    data[idx( 6, 1)] = Node_type({});
    data[idx( 7, 1)] = Node_type({});
    data[idx( 8, 1)] = Node_type({});
    data[idx( 9, 1)] = Node_type({});
    data[idx(10, 1)] = Node_type({});
    data[idx(11, 1)] = Node_type({});
    data[idx(12, 1)] = Node_type({});

    data[idx( 0, 0)] = Node_type({Direction::north,                                 Direction::east});
    data[idx( 1, 0)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx( 2, 0)] = Node_type({Direction::north,                 Direction::west,Direction::east});
    data[idx( 3, 0)] = Node_type({                                  Direction::west                });
    data[idx( 4, 0)] = Node_type({});
    data[idx( 5, 0)] = Node_type({});
    data[idx( 6, 0)] = Node_type({});
    data[idx( 7, 0)] = Node_type({});
    data[idx( 8, 0)] = Node_type({});
    data[idx( 9, 0)] = Node_type({});
    data[idx(10, 0)] = Node_type({});
    data[idx(11, 0)] = Node_type({});
    data[idx(12, 0)] = Node_type({});

    check_Grid_consistency(*this, "Marsi3E-constructor");
}  // Marsi3E_ExampleGrid ---constructor
