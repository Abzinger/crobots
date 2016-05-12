// DOTs_Code/UnionFind.hh
// DOT's Code
// Those were the times: from DOT's PhD thesis software

#ifndef __DOTs_Code__UnionFind_hh__
#define __DOTs_Code__UnionFind_hh__

namespace DOTs_Code {

    template<class BASE_t, class PARENTARRAY_t, class SIZEARRAY_t>
    // BASE_t         is a type with assignment.
    // PARENTARRAY_t  allows operator[]: T --> T
    // SIZEARRAY_t    allows operator[]: T --> N, where N subset ZZ.
    struct UnionFind
    {
        UnionFind (PARENTARRAY_t  * p_parent , SIZEARRAY_t  * p_size): p(*p_parent), sz(*p_size) {}
        // PRECONDITIONS
        // parent:  must be allocated so that parent[t] is valid for all t in T which will be passed as parameter to any of the member functions.
        // size:    (same)


        inline    void    make_set (BASE_t e);

        inline    BASE_t  find     (BASE_t e)  const;

        inline    BASE_t  do_union (BASE_t e, BASE_t f);                      // returns the new root

        template<class COUNTER_t>
        inline    BASE_t  do_union (BASE_t e, BASE_t f, COUNTER_t *p_count);  // returns the new root
                                                                              // calls ++(*p_count) if two sets were actuallyt merged

    protected:
        PARENTARRAY_t    & p;
        SIZEARRAY_t & sz;
    }; //^ UnionFind

} //^ namespace

template<class BASE_t, class PARENTARRAY_t, class SIZEARRAY_t>
void DOTs_Code::UnionFind<BASE_t,PARENTARRAY_t,SIZEARRAY_t>::make_set(BASE_t e)
{
    p[e] = e;
    sz[e] = 1;
} //^ make_set()


template<class BASE_t, class PARENTARRAY_t, class SIZEARRAY_t>
BASE_t DOTs_Code::UnionFind<BASE_t,PARENTARRAY_t,SIZEARRAY_t>::find(BASE_t x) const
{
    BASE_t y{x};
    while (p[y] != y)    y = p[y];
    while (p[x] != y )   { BASE_t t{x}; x = p[x]; p[t] = y; }
    return y;
} //^ find()


template<class BASE_t, class PARENTARRAY_t, class SIZEARRAY_t>
BASE_t DOTs_Code::UnionFind<BASE_t,PARENTARRAY_t,SIZEARRAY_t>::do_union(BASE_t e, BASE_t f)
{
    if (p[e] != e) e = find(e);
    if (p[f] != f) f = find(f);
    if (e==f) return e;

    if (sz[e] <= sz[f]) {
        sz[f] = sz[e] + sz[f];
        return p[e] = f;
    } else {
        sz[e] = sz[e] + sz[f];
        return p[f] = e;
    }
} //^ do_union()


template<class BASE_t, class PARENTARRAY_t, class SIZEARRAY_t>
template<class COUNTER_t>
BASE_t DOTs_Code::UnionFind<BASE_t,PARENTARRAY_t,SIZEARRAY_t>::do_union(BASE_t e, BASE_t f, COUNTER_t *p_count)
{
    if (p[e] != e) e = find(e);
    if (p[f] != f) f = find(f);
    if (e==f)  return e;
    else       ++(*p_count);

    if (sz[e] <= sz[f]) {
        sz[f] = sz[e] + sz[f];
        return p[e] = f;
    } else {
        sz[e] = sz[e] + sz[f];
        return p[f] = e;
    }
} //^ do_union()


#endif // ^ndef

// ^DOTs_Code/UnionFind.hh EOF
