// label.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis

#include "label.hh"

void GridSpace::Label::label_it()
{
    if (stat.size() < 1)  throw std::string("GridSpace::Label::label_it(): state vector time not long enough");
    rob.resize( stat.size(), G ); // let's hope this initializes everything to 0  :)

    // find robots at time 0:
    {
        XY xy;
        for (xy.y=0; xy.y<G.NS; ++xy.y) {
            for (xy.x=0; xy.x<G.EW; ++xy.x) {
                if (G.exists(xy)) {
                    if (stat[t][xy].ndstat != nobodyhome) {
                        ++R;
                        location_of_robot(0,R) = xy;
                        rob[t][xy] = R;
                    } else {
                        rob[t][xy] = 0;
                    }
                } //^ if xy exists
            } //^ for x
        } //^ for y
    }

    for (unsigned t=1; t<stat.size(); ++t) {
        for (unsigned r=1; r<=R; ++r) {
            const XY xy = location_of_robot(t-1,r);
            FullStat last_stat = stat[t-1][xy];
            switch (last_stat.ndstat) {
	    case R_ready:
            case C0R_ready:
            case C1R_ready:
            case C2R_ready:
            case R_vertical:      location_of_robot(t,r) = xy; rob[t][xy] = r;    break;

            case R_moving:
            case C0R_moving:
            case C1R_moving:
            case C2R_moving:
                if (get_ETA(last_stat.r_mv) > 1) {
                    location_of_robot(t,r) = xy;
                    rob[t][xy] = r;
                }
                else { // arrival is now!
                    const Direction d = get_direction(last_stat.r_mv);
                    const XY uv = G.move(xy, d);
                    if (uv==nowhere) throw std::string("GridSpace::Label::label_it(): movement into non-existing slot");
                    location_of_robot(t,r) = uv;
                    rob[t][uv] = r;
                }

                break;
            default:
                throw std::string("GridSpace::Label::label_it(): weirdo NdStat");
            } //^ switch
        } //^ for r
    } //^ for t
} //^ label_it()

bool GridSpace::Label::try_deletion(unsigned r, unsigned t_0, unsigned t_1)
{
    if (t_1 <= t_0) throw std::string("GridSpace::Label::try_deletion(): t_1 must be > t_0");

    // locations must be the same:
    if (location_of_robot(t_0,r) != location_of_robot(t_1,r))  return false;
    // operations must be the same:
    {
        Full_Stat stat_0 = stat[t_0][location_of_robot(t_0,r)];
        Full_Stat stat_1 = stat[t_1][location_of_robot(t_1,r)];
        if (stat_0.ndstat != stat_1.ndstat)  return false;
        switch (stat_0.ndstat) {
        case NdStat::R_vertical:
            if (stat_0.r_vert != stat_1.r_vert) return false;
            break;

        case NdStat::R_moving:
        case NdStat::C0R_moving:
        case NdStat::C1R_moving:
        case NdStat::C2R_moving:
            if (stat_0.r_mv != stat_1.r_mv) return false;
            break;

        case NdStat::R_ready:
        case NdStat::C0R_ready:
        case NdStat::C1R_ready:
        case NdStat::C2R_ready:
            break;

        default:
            throw std::string("GridStat::Label::try_deletion(): something is messed up with the stats.");
        } //^ switch
    } //^

    // check collision with other robots
    for (unsigned dt=0; dt<stat.size()-t_1; ++dt) {
        const unsigned t1        = t_1+dt;
        const XY       xy1       = location_of_robot(t1);
        const FullStat st1       = stat[t1][xy1];
        const unsigned t0        = t_0+dt;
        const FullStat st_t0_xy1 = stat[t0][xy1];

        const unsigned other_r = rob[t0][xy1];
        // no collision with other robot
        if (other_r && other_r != r)   return false;
        // same thing being lifted/dropped:
        if (st1.ndstat==R_Vertical && st1.on_node!=st_t0_xy1.on_node)   return false;

        

} //^ try_deletion()

void GridSpace::Label::shortcut()
{
    for (unsigned r=1; r<=R; ++r) {
        for (unsigned t=0; t<stat.size(); ++t) {
            const XY xy      = location_of_robot(t,r);
            FullStat xy_stat = stat[t][xy];
            
        } //^ for t
    } //^ for r
} //^ shortcut()
