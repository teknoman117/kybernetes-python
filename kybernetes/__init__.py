# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

def normalize_heading(heading):
    if heading > 180.0:
        return normalize_heading(heading - 360.0)
    elif heading < -180.0:
        return normalize_heading(heading + 360.0)
    return heading