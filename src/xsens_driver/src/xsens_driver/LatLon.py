# Functions for Latitude, Longitude calculations

import math

###def trunc(f,n):
#### REF http://stackoverflow.com/questions/783897/truncating-floats-in-python
###    slen = len('%.*f' % (n, f))
###    return float(str(f)[:slen])
###    
###def dd_to_dms(unit_dd):
#### REF http://en.wikipedia.org/wiki/Decimal_degrees
###    unit_d = trunc(unit_dd,0)
###    unit_m = trunc(math.fabs(unit_dd)*60,0)%60
###    unit_s = (math.fabs(unit_dd)*3600)%60
###    return [unit_d, unit_m, unit_s]

def distanceTwoLatLong(lat1,lat2,lon1,lon2): #returns distance between two locations in lat/long in meters.
# Code from http://www.movable-type.co.uk/scripts/latlong.html
    R = 6371000 #Radius of the earth in m
    dLat = math.radians(lat2-lat1)
    dLon = math.radians(lon2-lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    a = math.sin(dLat/2)*math.sin(dLat/2)+math.sin(dLon/2)*math.sin(dLon/2)*math.cos(lat1)*math.cos(lat2)
    c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a)) 
    d = R*c

    return d

def bearingTwoLatLong(lat1,lat2,lon1,lon2): #returns bearing between two locations in lat/long in degrees.
# Code from http://www.movable-type.co.uk/scripts/latlong.html: 0 degree is the heading towards north
    dLat = math.radians(lat2-lat1)
    dLon = math.radians(lon2-lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon);
    brng = math.atan2(y, x)
    brng= math.degrees(brng)

    return brng
  	
def RangeBearingToXY(d,brng): # convert
    brng = math.radians(brng)
    p1_x = d*math.sin(brng)
    p1_y = d*math.cos(brng)
    
    return [p1_x, p1_y]

def main_input(LatLon_dd, LatLon_ori):
    lat_dd = LatLon_dd[0]
    lon_dd = LatLon_dd[1]
    lat_ori = LatLon_ori[0]
    lon_ori = LatLon_ori[1]
    rnge = distanceTwoLatLong(lat_ori,lat_dd,lon_ori,lon_dd)
    brng = bearingTwoLatLong(lat_ori,lat_dd,lon_ori,lon_dd)
    [x, y] = RangeBearingToXY(rnge,brng)

    return [x, y]
