def normalize_heading(heading):
    if heading > 180.0:
        return normalize_heading(heading - 360.0)
    elif heading < -180.0:
        return normalize_heading(heading + 360.0)
    return heading