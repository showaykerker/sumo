#!/usr/bin/env python3
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2010-2021 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    gtfs_osm2pt.py
# @author  Giuliana Armellini
# @date    2021-02-18

"""
Import public transport from GTFS (schedules) and OSM (routes) data
"""

import os
import sys
import zipfile
import subprocess
import datetime
import math
from argparse import ArgumentParser
import pandas as pd
pd.options.mode.chained_assignment = None  # default='warn'

sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import sumolib
from sumolib.xml import parse_fast_nested as xmlParser


def initOptions():
    argParser = ArgumentParser()
    argParser.add_argument("-n", "--network", help="SUMO network file", required=True)
    argParser.add_argument("--osm-routes", help="osm routes file", required=True)
    argParser.add_argument("--gtfs", help="define gtfs zip file to load (mandatory)", required=True)
    argParser.add_argument("--region", help="define the region to filter gtfs data, format: W,S,E,N", required=True)
    argParser.add_argument("--date", help="define the day to import, format: 'YYYYMMDD'")
    argParser.add_argument("--pt-types", help="filter pt-types to import (bus, tram, train, subway and/or ferry). format: 'bus,tram'", default="bus,tram,train,subway,ferry", required=False)
    argParser.add_argument("--repair", help="repair osm routes", action='store_true')
    argParser.add_argument("--debug", action='store_true')
    argParser.add_argument("--bus-stop-length", default=13, type=float, help="length for a bus stop")
    argParser.add_argument("--train-stop-length", default=110, type=float, help="length for a train stop")
    argParser.add_argument("--tram-stop-length", default=60, type=float, help="length for a tram stop")
    argParser.add_argument("--duration", default=10, type=int, help="minimum time to wait on a stop")
    argParser.add_argument("--min-stops", default=3, type=int, help="minimum number of stops a public transport line must have to be imported")

    options = argParser.parse_args()
    options.pt_types = options.pt_types.split(",")
    options.region = [float(boundary) for boundary in options.region.split(",")]

    return options


def get_line_dir(line_orig, line_dest):
    # get direction of the route

    lat_dif = float(line_dest[1]) - float(line_orig[1])
    lon_dif = float(line_dest[0]) - float(line_orig[0])

    if lon_dif == 0:  # avoid dividing by 0
        line_dir = 90
    else:
        line_dir = math.degrees(math.atan(abs(lat_dif/lon_dif)))

    if lat_dif >= 0 and lon_dif >= 0:  # 1° quadrant
        line_dir = 90 - line_dir
    elif lat_dif < 0 and lon_dif > 0:  # 2° quadrant
        line_dir = 90 + line_dir
    elif lat_dif <= 0 and lon_dif <= 0:  # 3° quadrant
        line_dir = 90 - line_dir + 180
    else:  # 4° quadrant
        line_dir = 270 + line_dir

    return line_dir


def repair_routes(options, net, sumo_vClass):
    # use duarouter to repair the given osm routes
    osm_routes = {}
    # write dua input file
    with open("dua_input.xml", 'w+', encoding="utf8") as dua_file:
        dua_file.write("<routes>\n")
        for key, value in sumo_vClass.items():
            dua_file.write('\t<vType id="%s" vClass="%s"/>\n' % (key, value))

        sumo_edges = [sumo_edge.getID() for sumo_edge in net.getEdges()]
        for ptline, ptline_route in xmlParser(options.osm_routes, "ptLine", ("id", "name", "line", "type"), "route", ("edges")):
            if ptline.type not in options.pt_types:
                continue

            route_edges = ptline_route.edges.split(" ")
            # search ptLine origin
            index = 0
            line_orig = route_edges[index]
            while line_orig not in sumo_edges and index+1 < len(route_edges):
                # search for first route edge included in the sumo network
                index += 1
                line_orig = route_edges[index]
            if line_orig not in sumo_edges:
                # if no edge found, discard ptLine
                continue
            # adapt osm route to sumo network
            route_edges = route_edges[index:]

            # search ptLine destination
            index = -1
            line_dest = route_edges[index]
            while line_dest not in sumo_edges and index-1 < -len(route_edges):
                # search for last route edge included in the sumo network
                index += -1
                line_orig = route_edges[index]
            if line_dest not in sumo_edges:
                # if no edges found, discard ptLine
                continue
            # adapt osm route to sumo network
            route_edges = route_edges[: index-1]

            # consider only edges in sumo network
            route_edges = [edge for edge in route_edges if edge in sumo_edges]
            if not route_edges:
                # if no edges found, discard ptLine
                continue

            # transform ptLine origin and destination to geo coordinates
            x, y = net.getEdge(line_orig).getFromNode().getCoord()
            line_orig = net.convertXY2LonLat(x, y)
            x, y = net.getEdge(line_dest).getFromNode().getCoord()
            line_dest = net.convertXY2LonLat(x, y)

            # find ptLine direction
            line_dir = get_line_dir(line_orig, line_dest)

            osm_routes[ptline.id] = (ptline.attr_name, ptline.line, ptline.type, line_dir)
            dua_file.write("""\t<trip id="%s" type="%s" depart="0" via="%s"/>\n""" % (ptline.id, ptline.type, (" ").join(route_edges)))
        dua_file.write("</routes>\n")

    # run duarouter
    subprocess.call([sumolib.checkBinary('duarouter'), '-n', options.network,
                    '--route-files', 'dua_input.xml', '--repair', '-o',
                     'dua_output.xml', '--error-log', 'invalid_osm_routes.txt',
                     '--ignore-errors'])

    # parse repaired routes
    n_routes = len(osm_routes)

    for ptline, ptline_route in xmlParser("dua_output.xml", "vehicle", ("id"), "route", ("edges")):
        if len(ptline_route.edges) > 2:
            osm_routes[ptline.id] += (ptline_route.edges, )

    # remove dua files
    os.remove("dua_input.xml")
    os.remove("dua_output.xml")
    os.remove("dua_output.alt.xml")

    # remove invalid routes from dict
    [osm_routes.pop(line) for line in list(osm_routes) if len(osm_routes[line]) < 5]

    if n_routes != len(osm_routes):
        print("Not all given routes have been imported, see 'invalid_osm_routes.txt' for more information")

    return osm_routes


def main(options):

    # ----------------------- Import SUMO net ---------------------------------

    print("Import net")
    net = sumolib.net.readNet(options.network)

    # ----------------------- gtfs, osm and sumo modes ------------------------
    sumo_vClass = {
        'bus': 'bus',
        'train': 'rail',
        'tram': 'tram',
        'subway': 'rail_urban',
        'ferry': 'ship'
        }

    gtfs_modes = {
        # https://developers.google.com/transit/gtfs/reference/#routestxt
        '0':  'tram',
        '1':  'subway',
        '2':  'train',
        '3':  'bus',
        '4':  'ferry',
        # '5':  'cableTram',
        # '6':  'aerialLift',
        # '7':  'funicular',
        # https://developers.google.com/transit/gtfs/reference/extended-route-types
        '100':  'train',        # DB
        '109':  'train',  # S-Bahn
        '400':  'subway',      # U-Bahn
        '1000': 'ferry',        # Faehre
        # additional modes used in Hamburg
        '402':  'subway',      # U-Bahn
        '1200': 'ferry',        # Faehre
        # modes used by hafas
        's': 'train',
        'RE': 'train',
        'RB': 'train',
        'IXB': 'train',        # tbd
        'ICE': 'train',
        'IC': 'train',
        'IRX': 'train',        # tbd
        'EC': 'train',
        'NJ': 'train',        # tbd
        'RHI': 'train',        # tbd
        'DPN': 'train',        # tbd
        'SCH': 'train',        # tbd
        'Bsv': 'train',        # tbd
        'KAT': 'train',        # tbd
        'AIR': 'train',        # tbd
        'DPS': 'train',        # tbd
        'lt': 'train',  # tbd
        'BUS': 'bus',        # tbd
        'Str': 'tram',        # tbd
        'DPF': 'train',        # tbd
        }
    # https://developers.google.com/transit/gtfs/reference/extended-route-types
    for i in range(700, 717):
        gtfs_modes[str(i)] = 'bus'
    for i in range(900, 907):
        gtfs_modes[str(i)] = 'tram'

    # -----------------------  Import route-paths from OSM --------------------

    if options.repair:
        print("Import and repair osm routes")
        osm_routes = repair_routes(options, net, sumo_vClass)
    else:
        print("Import osm routes")
        osm_routes = {}
        for ptline, ptline_route in xmlParser(options.osm_routes, "ptLine", ("id", "name", "line", "type"), "route", ("edges")):
            if ptline.type not in options.pt_types:
                continue
            if len(ptline_route.edges) > 2:
                line_orig = ptline_route.edges.split(" ")[0]
                x, y = net.getEdge(line_orig).getFromNode().getCoord()
                line_orig = net.convertXY2LonLat(x, y)

                line_dest = ptline_route.edges.split(" ")[-1]
                x, y = net.getEdge(line_dest).getFromNode().getCoord()
                line_dest = net.convertXY2LonLat(x, y)

                line_dir = get_line_dir(line_orig, line_dest)

                osm_routes[ptline.id] = (ptline.attr_name, ptline.line, ptline.type, line_dir, ptline_route.edges)

    # -----------------------  Import GTFS data -------------------------------
    print("Import gtfs data")

    gtfsZip = zipfile.ZipFile(options.gtfs)
    routes = pd.read_csv(gtfsZip.open('routes.txt'), dtype=str)
    stops = pd.read_csv(gtfsZip.open('stops.txt'), dtype=str)
    stop_times = pd.read_csv(gtfsZip.open('stop_times.txt'), dtype=str)
    trips = pd.read_csv(gtfsZip.open('trips.txt'), dtype=str)
    shapes = pd.read_csv(gtfsZip.open('shapes.txt'), dtype=str)
    calendar_dates = pd.read_csv(gtfsZip.open('calendar_dates.txt'), dtype=str)
    calendar = pd.read_csv(gtfsZip.open('calendar.txt'), dtype=str)

    # change col types
    stops['stop_lat'] = stops['stop_lat'].astype(float)
    stops['stop_lon'] = stops['stop_lon'].astype(float)
    shapes['shape_pt_lat'] = shapes['shape_pt_lat'].astype(float)
    shapes['shape_pt_lon'] = shapes['shape_pt_lon'].astype(float)
    shapes['shape_pt_sequence'] = shapes['shape_pt_sequence'].astype(float)
    stop_times['stop_sequence'] = stop_times['stop_sequence'].astype(float)

    # filter trips for a representative date
    # from gtfs2fcd.py
    weekday = 'monday tuesday wednesday thursday friday saturday sunday'.split()[datetime.datetime.strptime(options.date, "%Y%m%d").weekday()]
    removed = calendar_dates[(calendar_dates.date == options.date) & (calendar_dates.exception_type == '2')]
    services = calendar[(calendar.start_date <= options.date) & (calendar.end_date >= options.date) &
                        (calendar[weekday] == '1') & (~calendar.service_id.isin(removed.service_id))]
    added = calendar_dates[(calendar_dates.date == options.date) & (calendar_dates.exception_type == '1')]
    gtfs_data = trips[trips.service_id.isin(services.service_id) | trips.service_id.isin(added.service_id)]

    # merge gtfs data from stop_times / trips / routes / stops
    gtfs_data = pd.merge(pd.merge(pd.merge(gtfs_data, stop_times, on='trip_id'), stops, on='stop_id'), routes, on='route_id')

    # filter given pt types
    filter_gtfs_modes = [key for key, value in gtfs_modes.items() if value in options.pt_types]
    gtfs_data = gtfs_data[gtfs_data['route_type'].isin(filter_gtfs_modes)]

    # Filter relevant information
    gtfs_data = gtfs_data[['route_id', 'shape_id', 'trip_id', 'stop_id', 'route_short_name', 'route_type', 'trip_headsign', 'direction_id', 'stop_name', 'stop_lat', 'stop_lon', 'stop_sequence', 'arrival_time', 'departure_time']]

    # replace characters
    gtfs_data['stop_name'] = gtfs_data['stop_name'].str.replace('[/|\'\";,!<>&*?\t\n\r]', ' ')
    gtfs_data['trip_headsign'] = gtfs_data['trip_headsign'].str.replace('[/|\'\";,!<>&*?\t\n\r]', ' ')

    # filter data inside SUMO net by stop location and shape
    gtfs_data = gtfs_data[(options.region[1] <= gtfs_data['stop_lat']) & (gtfs_data['stop_lat'] <= options.region[3]) & (options.region[0] <= gtfs_data['stop_lon']) & (gtfs_data['stop_lon'] <= options.region[2])]
    shapes = shapes[(options.region[1] <= shapes['shape_pt_lat']) & (shapes['shape_pt_lat'] <= options.region[3]) & (options.region[0] <= shapes['shape_pt_lon']) & (shapes['shape_pt_lon'] <= options.region[2])]

    # times to sec to enable sorting
    trip_list = gtfs_data[gtfs_data["stop_sequence"] == 0]
    trip_list['departure'] = pd.to_timedelta(trip_list['arrival_time'])

    # add column for unambiguous stop_id and sumo edge
    gtfs_data["stop_item_id"] = None
    gtfs_data["edge_id"] = None

    # search main and secondary shapes for each pt line (route and direction)
    filter_stops = gtfs_data.groupby(['route_id', 'direction_id', 'shape_id']).agg({'stop_sequence': 'max'}).reset_index()
    group_shapes = filter_stops.groupby(['route_id', 'direction_id']).shape_id.aggregate(lambda x: set(x)).reset_index()
    filter_stops = filter_stops.loc[filter_stops.groupby(['route_id', 'direction_id'])['stop_sequence'].idxmax()][['route_id', 'shape_id', 'direction_id']]
    filter_stops = pd.merge(filter_stops, group_shapes, on=['route_id', 'direction_id'])

    # create dict with shape and main shape
    shapes_dict = {}
    for row in filter_stops.itertuples():
        for sec_shape in row.shape_id_y:
            shapes_dict[sec_shape] = row.shape_id_x

    # create data frame with main shape for stop location
    filter_stops = gtfs_data[gtfs_data['shape_id'].isin(filter_stops.shape_id_x)]
    filter_stops = filter_stops[['route_id', 'shape_id', 'stop_id', 'route_short_name', 'route_type', 'trip_headsign', 'direction_id', 'stop_name', 'stop_lat', 'stop_lon']].drop_duplicates()

    # -----------------------  Define Stops and Routes ------------------------
    print("Map stops and routes")

    map_routes = {}
    map_stops = {}
    radius = 200  # gtfs stops are grouped (no in exact geo position), so a large radius is needed

    missing_stops = []
    missing_lines = []

    for row in filter_stops.itertuples():
        # check if route already discarded
        if row.shape_id in missing_lines:
            continue

        # check if gtfs route already mapped to osm route
        if not map_routes.get(row.shape_id, False):
            # if route not mapped, find the osm route for shape id
            pt_line = row.route_short_name
            pt_type = gtfs_modes[row.route_type]

            # get shape definition and define pt direction
            aux_shapes = shapes[shapes['shape_id'] == row.shape_id]
            pt_orig = aux_shapes[aux_shapes.shape_pt_sequence == aux_shapes.shape_pt_sequence.min()]
            pt_dest = aux_shapes[aux_shapes.shape_pt_sequence == aux_shapes.shape_pt_sequence.max()]
            line_dir = get_line_dir((pt_orig.shape_pt_lon, pt_orig.shape_pt_lat), (pt_dest.shape_pt_lon, pt_dest.shape_pt_lat))

            # get osm lines with same route name and pt type
            osm_lines = [key for key, value in osm_routes.items() if value[1] == pt_line and value[2] == pt_type]
            if len(osm_lines) > 1:
                # get the direction for the found routes and take the route with lower difference
                aux_dif = [abs(line_dir-osm_routes[key][3]) for key in osm_lines]
                osm_id = osm_lines[aux_dif.index(min(aux_dif))]

                # add mapped osm route to dict
                map_routes[row.shape_id] = (osm_id, osm_routes[osm_id][4].split(" "))
            else:
                # no osm route found, do not map stops of route
                missing_lines.append((pt_line, row.trip_headsign))
                continue

        # check if stop already mapped
        stop_mapped = [key for key in map_stops.keys() if key.split("_")[0] == row.stop_id]
        stop_item_id = 0  # for pt stops with different stop points

        # set stop's type, class and length
        pt_type = gtfs_modes[row.route_type]
        pt_class = sumo_vClass[pt_type]
        if pt_class == "bus":
            stop_length = options.bus_stop_length
        elif pt_class == "tram":
            stop_length = options.tram_stop_length
        else:
            stop_length = options.train_stop_length

        if stop_mapped:
            # get maximum item for stop
            stop_item_id = [int(stop.split("_")[1]) for stop in stop_mapped]
            stop_item_id = max(stop_item_id) + 1

            # check if the stop is already define
            for key in stop_mapped:
                # for item of mapped stop
                stop_edge = map_stops[key][1].split("_")[0]
                if stop_edge in map_routes[row.shape_id][1]:
                    # if edge in route, the stops are the same
                    # add the shape id to the stop
                    map_stops[key][5].append(row.shape_id)
                    # add to data frame
                    shape_list = [key for key, value in shapes_dict.items() if value == row.shape_id]
                    gtfs_data.loc[(gtfs_data["stop_id"] == row.stop_id) & (gtfs_data["shape_id"].isin(shape_list)), "stop_item_id"] = key
                    gtfs_data.loc[(gtfs_data["stop_id"] == row.stop_id) & (gtfs_data["shape_id"].isin(shape_list)), "edge_id"] = stop_edge

                    stop_mapped = True
                    break
                else:
                    # check if the wrong edge was adopted
                    # get edges near stop location
                    x, y = net.convertLonLat2XY(row.stop_lon, row.stop_lat)
                    edges = net.getNeighboringEdges(x, y, radius, includeJunctions=False)
                    edges.sort(key=lambda x: x[1])  # sort by distance

                    # interseccion between route edges of all shapes in stop
                    edge_inter = set(map_routes[row.shape_id][1])
                    for shape_item in map_stops[key][5]:  # shapes id of stop
                        edge_inter = set(edge_inter) & set(map_routes[shape_item][1])

                    # find edge
                    new_edge = [edge[0] for edge in edges if edge[0].getID() in edge_inter and edge[0].getLength() >= stop_length*1.20]  # filter length
                    if not new_edge:
                        new_edge = [edge[0] for edge in edges if edge[0].getID() in edge_inter]
                    if not new_edge:
                        continue  # stops are not same

                    # if the edge is in all routes
                    for lane in new_edge[0].getLanes():
                        # update the lane id, start and end
                        if lane.allows(pt_class):
                            lane_id = lane.getID()
                            pos = int(lane.getClosestLanePosAndDist((x, y))[0])
                            start = max(0, pos-stop_length)
                            end = min(start+stop_length, lane.getLength())
                            map_stops[key][1:4] = [lane_id, start, end]
                            break
                    # update edge in data frame
                    gtfs_data.loc[gtfs_data["stop_item_id"] == key, "edge_id"] = new_edge[0].getID()
                    # add to data frame
                    shape_list = [key for key, value in shapes_dict.items() if value == row.shape_id]
                    gtfs_data.loc[(gtfs_data["stop_id"] == row.stop_id) & (gtfs_data["shape_id"].isin(shape_list)), "stop_item_id"] = key
                    gtfs_data.loc[(gtfs_data["stop_id"] == row.stop_id) & (gtfs_data["shape_id"].isin(shape_list)), "edge_id"] = new_edge[0].getID()

                    stop_mapped = True
                    break

            if stop_mapped is not True:
                stop_mapped = None  # if stop not the same, search stop

        # if stop not mapped
        if not stop_mapped:
            # get edges near stop location
            x, y = net.convertLonLat2XY(row.stop_lon, row.stop_lat)
            edges = net.getNeighboringEdges(x, y, radius, includeJunctions=False)
            edges = [edge for edge in edges if edge[0].getLength() >= stop_length*1.20]  # filter length
            edges.sort(key=lambda x: x[1])  # sort by distance

            for edge in edges:
                if not edge[0].getID() in map_routes[row.shape_id][1]:
                    # if edge not in pt line route
                    continue

                for lane in edge[0].getLanes():
                    if not lane.allows(pt_class):
                        continue
                    lane_id = lane.getID()
                    pos = lane.getClosestLanePosAndDist((x, y))[0]
                    start = max(0, pos-stop_length)
                    end = min(start+stop_length, lane.getLength())
                    stop_item_id = "%s_%s" % (row.stop_id, stop_item_id)
                    map_stops[stop_item_id] = [row.stop_name, lane_id, start, end, pt_type, [row.shape_id]]
                    # add data to data frame
                    shape_list = [key for key, value in shapes_dict.items() if value == row.shape_id]
                    gtfs_data.loc[(gtfs_data["stop_id"] == row.stop_id) & (gtfs_data["shape_id"].isin(shape_list)), "stop_item_id"] = stop_item_id
                    gtfs_data.loc[(gtfs_data["stop_id"] == row.stop_id) & (gtfs_data["shape_id"].isin(shape_list)), "edge_id"] = edge[0].getID()

                    stop_mapped = True
                    break
                break

        # if stop not mapped, add to missing stops
        if not stop_mapped:
            missing_stops.append((row.stop_id, row.stop_name, row.route_short_name))

    # -----------------------   Write Stops Output ----------------------------

    print("Generates stops output")

    stop_output = "gtfs_stops.add.xml"
    with open(stop_output, 'w', encoding="utf8") as output_file:
        sumolib.xml.writeHeader(output_file, stop_output, "additional")
        for key, value in map_stops.items():
            if value[4] == "bus":
                output_file.write('    <busStop id="%s" lane="%s" startPos="%s" endPos="%s" name="%s" friendlyPos="true"/>\n' %
                                  (key, value[1], value[2], value[3], value[0]))
            else:
                # from gtfs2pt.py
                output_file.write('    <trainStop id="%s" lane="%s" startPos="%s" endPos="%s" name="%s" friendlyPos="true">\n' %
                                  (key, value[1], value[2], value[3], value[0]))

                ap = sumolib.geomhelper.positionAtShapeOffset(net.getLane(value[1]).getShape(), value[2])
                numAccess = 0
                for accessEdge, _ in sorted(net.getNeighboringEdges(*ap, r=100), key=lambda i: i[1]):
                    if accessEdge.getID() != key.split("_")[0] and accessEdge.allows("pedestrian"):
                        _, accessPos, accessDist = accessEdge.getClosestLanePosDist(ap)
                        output_file.write(('        <access friendlyPos="true" ' +
                                          'lane="%s_0" pos="%s" length="%s"/>\n') %
                                          (accessEdge.getID(), accessPos, 1.5 * accessDist))
                        numAccess += 1
                        if numAccess == 5:
                            break
                output_file.write('    </trainStop>\n')
        output_file.write('</additional>\n')

    print("Generates routes output")

    sequence_errors = []
    route_output = "gtfs_ptline.rou.xml"

    with open(route_output, 'w', encoding="utf8") as output_file:
        sumolib.xml.writeHeader(output_file, route_output, "routes")
        for key, value in sumo_vClass.items():
            output_file.write('    <vType id="%s" vClass="%s"/>\n' % (key, value))

        for row in trip_list.sort_values("departure").itertuples():

            main_shape = shapes_dict.get(row.shape_id, None)
            if not map_routes.get(main_shape, None):
                # if route not mapped
                continue

            pt_type = gtfs_modes[row.route_type]
            edges_list = map_routes[main_shape][1]
            stop_list = gtfs_data[gtfs_data["trip_id"] == row.trip_id].sort_values("stop_sequence")
            stop_index = [edges_list.index(stop.edge_id) for stop in stop_list.itertuples() if stop.edge_id in edges_list]

            if len(set(stop_index)) < options.min_stops:
                # Not enough stops mapped
                continue

            output_file.write('    <vehicle id="%s_%s" line="%s_%s" depart="%s" departEdge="%s" arrivalEdge="%s" type="%s"><!--%s-->\n'
                              % (row.route_short_name, row.trip_id, row.route_id, row.direction_id, row.arrival_time, min(stop_index), max(stop_index), pt_type, row.trip_headsign))
            output_file.write('        <route edges="%s"/>\n' % (" ".join(edges_list)))

            check_seq = -1
            for stop in stop_list.itertuples():
                if not stop.stop_item_id:
                    # if stop not mapped
                    continue
                stop_index = edges_list.index(stop.edge_id)
                if stop_index > check_seq:
                    check_seq = stop_index
                    output_file.write('        <stop busStop="%s" arrival="%s" duration="%s" until="%s"/><!--%s-->\n' %
                                      (stop.stop_item_id, stop.arrival_time, options.duration, stop.departure_time, stop.stop_name))
                elif stop_index < check_seq:
                    # stop not downstream
                    sequence_errors.append((stop.stop_item_id, row.route_short_name, row.trip_headsign, stop.trip_id))

            output_file.write('    </vehicle>\n')
        output_file.write('</routes>\n')

    # -----------------------   Save missing data ------------------
    if any([missing_stops, missing_lines, sequence_errors]):
        print("Not all given gtfs elements have been mapped, see 'gtfs_missing.xml' for more information")
        with open("gtfs_missing.xml", 'w', encoding="utf8") as output_file:
            output_file.write('<missingElements>\n')
            for stop in set(missing_stops):
                output_file.write('    <stop id="%s" name="%s" ptLine="%s"/>\n' % stop)
            for line in set(missing_lines):
                output_file.write('    <ptLine id="%s" trip_headsign="%s"/>\n' % line)
            for stop in set(sequence_errors):
                output_file.write('    <stopSequence stop_id="%s" ptLine="%s" trip_headsign="%s" trip_id="%s"/>\n' % stop)
            output_file.write('</missingElements>\n')


if __name__ == "__main__":
    main(initOptions())
