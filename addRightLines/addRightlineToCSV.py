#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 10 00:26:11 2021

@author: minghao
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


df_original = pd.read_csv('pd-ring-road-2022-12-03.csv', skiprows=(0,1))
df_right_lines = pd.read_csv('right_line_results.csv')
df_right_lines.columns = ['Bus_stop_line_east', 'Bus_stop_line_north']
right_lines = np.array(df_right_lines)


### assign the none values with the nearest right line
for i in range(len(df_original)):
    i_line = df_original.loc[i]
    if np.isnan(i_line['Bus_stop_line_east']):
        # find the nearest point in detected results
        i_car_xy = np.array([i_line['East_P'], i_line['North_P']])
        i_dist = np.sum(np.square(right_lines-i_car_xy), axis=1)
        i_argmin = i_dist.argmin()
        df_original.at[i,'Bus_stop_line_east'] = right_lines[i_argmin,0]
        df_original.at[i,'Bus_stop_line_north'] = right_lines[i_argmin,1]
        
    
## Interpolate from 506 to 556, from 648 to 668, 1152 to 1178, 1446 to 1483
df_original.loc[506:557, ['Bus_stop_line_east', 'Bus_stop_line_north']] = None
df_original.loc[648:669, ['Bus_stop_line_east', 'Bus_stop_line_north']] = None
df_original.loc[1152:1179, ['Bus_stop_line_east', 'Bus_stop_line_north']] = None
df_original.loc[1446:1483, ['Bus_stop_line_east', 'Bus_stop_line_north']] = None

df_original['Bus_stop_line_east'] = df_original['Bus_stop_line_east'].interpolate()
df_original['Bus_stop_line_north'] = df_original['Bus_stop_line_north'].interpolate()


## save df based on the desired format
test_df = pd.concat([pd.DataFrame([pd.Series(),pd.Series()]), df_original], ignore_index=True)
test_df.iloc[1,:] = test_df.columns
test_df.columns = [None]*len(test_df.columns)
test_df.to_csv('pd-ring-road-2022-12-03-with-right-lines.csv', index=False)
        
        
plt.scatter(df_original['Right_curb_east'], df_original['Right_curb_north'])
plt.scatter(df_original['Bus_stop_line_east'], df_original['Bus_stop_line_north'])

plt.xlabel("E")
plt.ylabel('N')
plt.axis('equal')


from gmplot import GoogleMapPlotter

import pyproj

centerline_E = np.array(df_original['Center_line_east'])
centerlin_N = np.array(df_original['Center_line_north'])

leftcurb_E = np.array( df_original['Bus_stop_line_east'] )
leftcurb_N = np.array( df_original['Bus_stop_line_north'] )
rightcurb_E = np.array( df_original['Right_curb_east'] )
rightcurb_N = np.array( df_original['Right_curb_north'] )


ref_easting = 537132
ref_northing = 4813391

## UTM grid zone
# import geodesy.utm
# z, b = geodesy.utm.gridZone(43.4722836, -80.5410776)
z, b = 17, 'T'
utm_proj = pyproj.Proj(proj='utm', zone=z, datum='WGS84')

def project_from_LUTM_to_LatLong(localE, localN):
    lon, lat = utm_proj(ref_easting+localE, ref_northing+localN, inverse=True)
    return lon, lat

centerline_lon, centerline_lat = project_from_LUTM_to_LatLong(centerline_E, centerlin_N)
leftcurb_lon, leftcurb_lat = project_from_LUTM_to_LatLong(leftcurb_E, leftcurb_N)
rightcurb_lon, rightcurb_lat = project_from_LUTM_to_LatLong(rightcurb_E, rightcurb_N)
class CustomGoogleMapPlotter(GoogleMapPlotter):
    def __init__(self, center_lat, center_lng, zoom, apikey='',
                  map_type='satellite'):
        super().__init__(center_lat, center_lng, zoom, apikey)

        self.map_type = map_type
        assert(self.map_type in ['roadmap', 'satellite', 'hybrid', 'terrain'])

    def write_map(self,  f):
        f.write('\t\tvar centerlatlng = new google.maps.LatLng(%f, %f);\n' %
                (self.center[0], self.center[1]))
        f.write('\t\tvar myOptions = {\n')
        f.write('\t\t\tzoom: %d,\n' % (self.zoom))
        f.write('\t\t\tcenter: centerlatlng,\n')

        # This is the only line we change
        f.write('\t\t\tmapTypeId: \'{}\'\n'.format(self.map_type))


        f.write('\t\t};\n')
        f.write(
            '\t\tvar map = new google.maps.Map(document.getElementById("map_canvas"), myOptions);\n')
        f.write('\n')
        
gmap = CustomGoogleMapPlotter(43.4722836, -80.5410776, 16, map_type='satellite')
# gmap.plot(lats, lons, 'cornflowerblue', edge_width=10)
to_draw_inds = np.linspace(0, len(centerline_lon)-1, 1000, dtype=np.int32)
gmap.scatter(centerline_lat[to_draw_inds], centerline_lon[to_draw_inds], 'yellow', edge_width=6)
gmap.scatter(leftcurb_lat[to_draw_inds], leftcurb_lon[to_draw_inds], 'red', edge_width=6)
gmap.scatter(rightcurb_lat[to_draw_inds], rightcurb_lon[to_draw_inds], 'blue', edge_width=6)
gmap.draw("mymap.html")




'''
### Interpolate code
original_csv_path = '/home/minghao/Documents/UWaterloo/LaneDetection/oct4_rr_latest.csv'
df = pd.read_csv(original_csv_path, skiprows=(0,1))

# df = df[1778:1792]
df.loc[df['Center_line_east'] == 0,['Center_line_east', 'Center_line_north']] = None

plt.scatter(df['Center_line_east'], df['Center_line_north'])
plt.scatter(df['Right_curb_east'], df['Right_curb_north'])
plt.scatter(df['Left_curb_east'], df['Left_curb_north'])
plt.scatter(df['East_P'], df['North_P'])

plt.xlabel("E")
plt.ylabel('N')
plt.axis('equal')


# interpolate df
# First step: calculate lateral distance between center marker and ego vehicle.
# Second step: intepolater the lateral distance
# Third step: change the lateral distance back to LUTM


## calculate ego vhicle heading

diff_E = np.array(df['East_P'].diff())
diff_E[0] = diff_E[-1]
diff_N = np.array(df['North_P'].diff())
diff_N[0] = diff_N[-1]

heading = np.rad2deg(np.arctan2(diff_N, diff_E))
inds = np.zeros_like(heading, dtype = np.bool)
inds[800:] = True
inds = inds & (heading > -80)
heading[inds] -= 360

ego_E = np.array(df['East_P'])
ego_N = np.array(df['North_P'])
center_E = np.array( df['Center_line_east'] )
center_N = np.array( df['Center_line_north'])

cos_heading = np.cos( np.deg2rad(heading) )
sin_heading = np.sin( np.deg2rad(heading) )

dist_E = center_E - ego_E
dist_N = center_N - ego_N
lateral_distance = dist_N * cos_heading - dist_E * sin_heading

lateral_dis_df = pd.DataFrame(lateral_distance)
lateral_dis_df = lateral_dis_df.interpolate()
lateral_dis_df.plot()
interpolated_lateral_dis = np.array(lateral_dis_df)[:,0]


displace_E = -interpolated_lateral_dis * sin_heading
displace_N = interpolated_lateral_dis * cos_heading
interpolated_E = ego_E + displace_E
interpolated_N = ego_N + displace_N

inter_EN = np.vstack([interpolated_E, interpolated_N]).T

interpolated_E[1428:1460] = None
interpolated_E[2825:2864] = None
interpolated_N[1428:1460] = None
interpolated_N[2825:2864] = None

inter_EN_df = pd.DataFrame( {'E':interpolated_E, 'N':interpolated_N} )
inter_EN_df = inter_EN_df.interpolate()

# plt.figure(4)
# plt.scatter(inter_EN_df['E'], inter_EN_df['N'])

df['Center_line_east_new'] = inter_EN_df['E']
df['Center_line_north_new'] = inter_EN_df['N']

plt.figure(3)
plt.scatter(df['Center_line_east'], df['Center_line_north'])
plt.scatter(interpolated_E, interpolated_N)

plt.xlabel("E")
plt.ylabel('N')
plt.axis('equal')

df.to_csv('/home/minghao/Documents/UWaterloo/LaneDetection/oct4_rr_latest_new.csv')
'''





# original_csv_path = '/home/minghao/Documents/UWaterloo/LaneDetection/RingroadWithCenterline.csv'
# df = pd.read_csv(original_csv_path, skiprows=(0,1))

# # df = df[:50]
# # df = df[1778:1792]
# df.loc[df['Bus_east'] == 0,['Center_line_east_raw', 'Center_line_north_raw', 'Bus_east', 'Bus_north', 'Bus_heading']] = None
# df.loc[df['Center_line_east_raw'] == 0,['Center_line_east_raw', 'Center_line_north_raw', 'Bus_east', 'Bus_north', 'Bus_heading']] = None

# plt.scatter(df['Center_line_east'], df['Center_line_north'])
# plt.scatter(df['Center_line_east_raw'], df['Center_line_north_raw'])
# plt.scatter(df['Right_curb_east'], df['Right_curb_north'])
# plt.scatter(df['Left_curb_east'], df['Left_curb_north'])
# plt.scatter(df['Bus_east'], df['Bus_north'])

# plt.xlabel("E")
# plt.ylabel('N')
# plt.axis('equal')

# # assert 0
# ## calculate ego vhicle heading


# heading = np.rad2deg(df['Bus_heading'])
# heading_diff = heading.diff()
# heading[heading_diff.abs()>10] = None
# inds = np.zeros_like(heading, dtype = np.bool)
# inds[350:] = True
# inds = inds & (heading > -156)
# heading[inds] -= 360

# plt.figure(2)
# heading.plot()

# heading = np.array(heading) + 270

# ego_E = np.array(df['Bus_east'])
# ego_N = np.array(df['Bus_north'])
# center_E = np.array( df['Center_line_east_raw'] )
# center_N = np.array( df['Center_line_north_raw'])


# cos_heading = np.cos( np.deg2rad(heading) )
# sin_heading = np.sin( np.deg2rad(heading) )

# dist_E = center_E - ego_E
# dist_N = center_N - ego_N
# lateral_distance = dist_N * cos_heading - dist_E * sin_heading

# assert 0

# lateral_dis_df = pd.DataFrame(lateral_distance)
# lateral_dis_df = lateral_dis_df.interpolate()
# lateral_dis_df.plot()
# interpolated_lateral_dis = np.array(lateral_dis_df)[:,0]


# displace_E = -interpolated_lateral_dis * sin_heading
# displace_N = interpolated_lateral_dis * cos_heading
# interpolated_E = ego_E + displace_E
# interpolated_N = ego_N + displace_N

# inter_EN = np.vstack([interpolated_E, interpolated_N]).T

# interpolated_E[1428:1460] = None
# interpolated_E[2825:2864] = None
# interpolated_N[1428:1460] = None
# interpolated_N[2825:2864] = None

# inter_EN_df = pd.DataFrame( {'E':interpolated_E, 'N':interpolated_N} )
# inter_EN_df = inter_EN_df.interpolate()

# # plt.figure(4)
# # plt.scatter(inter_EN_df['E'], inter_EN_df['N'])

# df['Center_line_east_new'] = inter_EN_df['E']
# df['Center_line_north_new'] = inter_EN_df['N']

# plt.figure(3)
# plt.scatter(df['Center_line_east'], df['Center_line_north'])
# plt.scatter(interpolated_E, interpolated_N)

# plt.xlabel("E")
# plt.ylabel('N')
# plt.axis('equal')

# df.to_csv('/home/minghao/Documents/UWaterloo/LaneDetection/oct4_rr_latest_new.csv')



'''
filtered_csv_path = '/home/minghao/Documents/UWaterloo/LaneDetection/oct4_rr_latest_new.csv'
df = pd.read_csv(filtered_csv_path, skiprows=(0,1))

plt.scatter(df['Center_line_east'], df['Center_line_north'])
plt.scatter(df['Right_curb_east'], df['Right_curb_north'])
plt.scatter(df['Left_curb_east'], df['Left_curb_north'])
plt.scatter(df['East_P'], df['North_P'])

plt.xlabel("E")
plt.ylabel('N')
plt.axis('equal')
'''


'''
### Check result using Google map
from gmplot import GoogleMapPlotter
import geodesy.utm
import pyproj


filtered_csv_path = '/home/minghao/Documents/UWaterloo/LaneDetection/RingroadWithCenterline.csv'
df = pd.read_csv(filtered_csv_path, skiprows=(0,1))
centerline_E = np.array(df['Center_line_east'])
centerlin_N = np.array(df['Center_line_north'])

leftcurb_E = np.array( df['Left_curb_east'] )
leftcurb_N = np.array( df['Left_curb_north'] )
rightcurb_E = np.array( df['Right_curb_east'] )
rightcurb_N = np.array( df['Right_curb_north'] )


ref_easting = 537132
ref_northing = 4813391

## UTM grid zone
z, b = geodesy.utm.gridZone(43.4722836, -80.5410776)
utm_proj = pyproj.Proj(proj='utm', zone=z, datum='WGS84')

def project_from_LUTM_to_LatLong(localE, localN):
    lon, lat = utm_proj(ref_easting+localE, ref_northing+localN, inverse=True)
    return lon, lat

centerline_lon, centerline_lat = project_from_LUTM_to_LatLong(centerline_E, centerlin_N)
leftcurb_lon, leftcurb_lat = project_from_LUTM_to_LatLong(leftcurb_E, leftcurb_N)
rightcurb_lon, rightcurb_lat = project_from_LUTM_to_LatLong(rightcurb_E, rightcurb_N)
class CustomGoogleMapPlotter(GoogleMapPlotter):
    def __init__(self, center_lat, center_lng, zoom, apikey='',
                 map_type='satellite'):
        super().__init__(center_lat, center_lng, zoom, apikey)

        self.map_type = map_type
        assert(self.map_type in ['roadmap', 'satellite', 'hybrid', 'terrain'])

    def write_map(self,  f):
        f.write('\t\tvar centerlatlng = new google.maps.LatLng(%f, %f);\n' %
                (self.center[0], self.center[1]))
        f.write('\t\tvar myOptions = {\n')
        f.write('\t\t\tzoom: %d,\n' % (self.zoom))
        f.write('\t\t\tcenter: centerlatlng,\n')

        # This is the only line we change
        f.write('\t\t\tmapTypeId: \'{}\'\n'.format(self.map_type))


        f.write('\t\t};\n')
        f.write(
            '\t\tvar map = new google.maps.Map(document.getElementById("map_canvas"), myOptions);\n')
        f.write('\n')
        
gmap = CustomGoogleMapPlotter(43.4722836, -80.5410776, 16, map_type='satellite')
# gmap.plot(lats, lons, 'cornflowerblue', edge_width=10)
to_draw_inds = np.linspace(0, len(centerline_lon)-1, 3075, dtype=np.int32)
gmap.scatter(centerline_lat[to_draw_inds], centerline_lon[to_draw_inds], 'yellow', edge_width=6)
gmap.scatter(leftcurb_lat[to_draw_inds], leftcurb_lon[to_draw_inds], 'red', edge_width=6)
gmap.scatter(rightcurb_lat[to_draw_inds], rightcurb_lon[to_draw_inds], 'blue', edge_width=6)
gmap.draw("mymap.html")

'''



