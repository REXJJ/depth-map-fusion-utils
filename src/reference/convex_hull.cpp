#include <boost/geometry.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <iostream>

using namespace std;
namespace bg = boost::geometry;

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::tuple<float, float> point;
typedef bg::model::ring<point> ring;

void dump(point const& p) { cout << get<0>(p) << " " << get<1>(p) << ","; }

void call_API(point const* arr, size_t n) {
    cout << "hull (API):";
    for_each(arr, arr+n, dump);
    cout << "\n";
}

int main()
{
    ring poly {
        { 0.0, 0.0 },
        //{ 0.5, 0.5 }, // mid point, which should not be part of the hull
        { 1.0, 0.0 },
        { 0.5, 0.0 },
        { 1.0, 1.0 },
        { 2.0, 2.0 },
        { 1.0, 0.0 },
    };

    cout << "raw:       " << bg::wkt(poly) << "\n";
    // bg::correct(poly);
    // cout << "corrected: " << bg::wkt(poly) << "\n";

    ring hull;
    bg::convex_hull(poly, hull);

    cout << "hull:      " << bg::wkt(hull) << "\n";

    point const* hull_array_ptr = &hull.front();

    call_API(hull_array_ptr, hull.size());
}
