from shapely.geometry import Polygon, LineString, Point
from geopy.distance import geodesic
from shapely.ops import nearest_points

from Launch_Parameters import beach_plygon
from search_leg import Search_leg
import Launch_Parameters


def calc_intersec(beach_polygon, leg):

    beach = Polygon(beach_polygon)
    start_point = Point(leg.start_pos)
    end_point = Point(leg.end_pos)

    if beach.contains(start_point) == False and beach.contains(end_point) == False:
        # both starting and end posistion of leg is outside of beach polygon and can therefore
        # we presume that the leg does not cross the beach. Edge cases may circumvent above logic,
        # #but that is beyond scope of this project
        return leg

    elif beach.contains(start_point) == True and beach.contains(end_point) == False:
        int_sec_index = 1
        leg.is_active = True
        leg.intersect_dir = 1

    elif beach.contains(start_point) == False and beach.contains(end_point) == True:
        int_sec_index = 0
        leg.is_active = True
        leg.intersect_dir = -1

    else:
        leg.is_active = False
        return leg


    leg_points = (start_point,end_point)
    line = LineString(leg_points)
    # Use Shapely to check for intersection
    inter = beach.intersection(line)

    if inter.is_empty:
        pass

    else:

        if inter.geom_type in ("LineString", "MultiLineString"):
            # The line overlaps with polygon edge(s)
            # You can sample endpoints or points along it, for example:
            for g in getattr(inter, "geoms", [inter]):
                coords = list(g.coords)

        if int_sec_index == 0:
            leg.intersect_point = coords[0]
        elif int_sec_index == 1:
            leg.intersect_point = coords[-1]

    return leg

def calc_intersect_from_pos(pos_1, pos_2, polygon):
    beach = Polygon(polygon)

    # Example line segment (two points)
    line_points = [pos_1, pos_2]
    line = LineString(line_points)

    # Intersection
    inter = beach.intersection(line)

    if inter.is_empty:
        return None
    else:

        if inter.geom_type in ("LineString", "MultiLineString"):
            # The line overlaps with polygon edge(s)
            # You can sample endpoints or points along it, for example:
            for g in getattr(inter, "geoms", [inter]):
                coords = list(g.coords)

            return coords[0]

def calc_point_in_poly(beach_polygon, pos):
    beach = Polygon(beach_polygon)
    point = Point(pos)

    if beach.contains(point):
        return True
    else:
        return False

def calc_dist_to_poly(beach_plygon, pos):
    beach = Polygon(beach_plygon)
    p = Point(pos)

    p_on_poly, _  = nearest_points(beach, p)

    geo_dist = geodesic(([pos[1], pos[0]]),(p_on_poly.y, p_on_poly.x))

    return geo_dist