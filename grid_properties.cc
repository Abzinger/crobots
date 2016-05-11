// grid_properties.cc C++11
// Part of the robots project
// Author: Dirk Oliver Theis

#include "grid_properties.hh"

#include "DOTs_Code/UnionFind.hh"

//######################################################################################################################################################
// grid_connected()
//######################################################################################################################################################
bool GridSpace::grid_connected( const Grid & G )
{
    typedef XY                             Base_t;
    typedef Grid::vector_grid< XY       >  Parent_t;
    typedef Grid::vector_grid< unsigned >  Size_t;

    typedef DOTs_Code::UnionFind< Base_t, Parent_t, Size_t > UF_t;

    Parent_t parray{G};
    Size_t   sizarray{G};

    UF_t uf (&parray,&sizarray);

    // make singletons
    for(short y=0; y<G.NS; ++y)  for(short x=0; x<G.EW; ++x)  if(G.exists(XY{x,y}))    uf.make_set( XY{x,y} );

    unsigned n_merges = 0;
    for (short y=0; y<G.NS; ++y)  {
        for (short x=0; x<G.EW; ++x) {
            const XY xy {x,y};
            if (G.exists(XY{x,y})) {
                { // east:
                    const XY uv = G.east(xy);
                    if (uv != nowhere)  uf.do_union(xy,uv, &n_merges);
                }
                { // north:
                    const XY uv = G.north(xy);
                    if (uv != nowhere)  uf.do_union(xy,uv, &n_merges);
                }
                { // west:
                    const XY uv = G.west(xy);
                    if (uv != nowhere)  uf.do_union(xy,uv, &n_merges);
                }
                { // south:
                    const XY uv = G.south(xy);
                    if (uv != nowhere)  uf.do_union(xy,uv, &n_merges);
                }
            } //^ if exists
        } //^ for x
    } //^ for y

    if ( n_merges >= G.numo_slots()  &&  n_merges > 0) throw std::string("GridSpace::grid_connected(): more merges than slots!! :-|");

    return (  n_merges+1 == G.numo_slots()  );
} //^ grid_connected()

//######################################################################################################################################################
// grid_fullsize()
//######################################################################################################################################################
bool GridSpace::grid_fullsize( const Grid & G )
{
    {
        short y;
        for (y=0; y<G.NS && !G.exists(XY{0,y}); ++y);
        if (y==G.NS) return false;
    }
    {
        short y;
        for (y=0; y<G.NS && !G.exists(XY{(short)(G.EW-1),y}); ++y);
        if (y==G.NS) return false;
    }
    {
        short x;
        for (x=0; x<G.EW && !G.exists(XY{x,0}); ++x);
        if (x==G.EW) return false;
    }
    {
        short x;
        for (x=0; x<G.EW && !G.exists(XY{x,(short)(G.NS-1)}); ++x);
        if (x==G.EW) return false;
    }
    return true;
} //^ grid_fullsize()


// ^grid_properties.cc EOF
