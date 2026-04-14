class Search_leg:
    start_pos = (0,0)
    end_pos = (0,0)
    intersect_point = None
    is_active = False

    # If intersect_dir = 1, we are heading towards the end_position
    # If = -1, we are heading towards the start_position
    # if = 0, default value + has no intersection point with beach polygon
    intersect_dir = 0
