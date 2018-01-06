import time
import numpy as np
import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
import requests
import osmnx as ox
from shapely.geometry import Point, Polygon, LineString, MultiLineString



def get_graph(coords, distance, network_type):
    
    G = ox.graph_from_point(center_point=coords, distance=distance, distance_type='bbox', network_type=network_type,
                            simplify=True, truncate_by_edge=True, retain_all=True)
    
    return G



def calculate_planar_intersections(G, bbox):
    
    # get an undirected graph to do line intersection counts (so we don't have identical line segments in each direction)
    G_undir = ox.get_undirected(G)
    gdf_edges = ox.graph_to_gdfs(G_undir, nodes=False)
    
    # trim the lines to the bounding box
    lines_in_bbox = gdf_edges.intersection(bbox)
    lines_in_bbox = lines_in_bbox[~lines_in_bbox.is_empty]
    
    # find all points where lines intersect
    all_intersect_points = []
    lines = lines_in_bbox
    for label, line in lines.iteritems():
        intersect_points = lines.drop(label).intersection(line) #intersect with everything but itself
        mask = intersect_points.map(lambda x: isinstance(x, Point))
        all_intersect_points.extend(intersect_points[mask].tolist())
        
    # de-dupe the points of intersection
    all_intersect_points = gpd.GeoSeries(all_intersect_points)
    planar_intersections = gpd.GeoSeries(list(all_intersect_points.unary_union))
    
    return planar_intersections



def calculate_nonplanar_intersections(G, bbox):
    
    # identify every true street intersection in the graph by retaining all the nodes
    # that have more than one street emanating from them. this will include nodes that
    # may have intersecting streets that continue outside of the graph.
    streets_per_node = G.graph['streets_per_node']
    node_ids = set(G.nodes())
    npi_nodes = [node for node, count in streets_per_node.items() if (count > 1) and (node in node_ids)]
    
    # retain only intersections within the bounding box
    npi_points = [Point((data['x'], data['y'])) for node, data in G.nodes(data=True) if node in npi_nodes]
    nonplanar_intersections = gpd.GeoSeries(npi_points).intersection(bbox)
    
    # return all the non-empty (ie, non-null) points
    return nonplanar_intersections[~nonplanar_intersections.is_empty]



def calculate_cleaned_intersections(nonplanar_intersections, original_crs, tolerance=10):
    
    try:
        gdf = gpd.GeoDataFrame(geometry=nonplanar_intersections)
        gdf.crs = original_crs
        gdf_proj = ox.project_gdf(gdf)

        buffered_nodes = gdf_proj.buffer(tolerance).unary_union
        if isinstance(buffered_nodes, Polygon):
            # if only a single node results, make it iterable so we can turn it into a GeoSeries
            buffered_nodes = [buffered_nodes]

        # get the centroids of the merged intersection polygons
        unified_intersections = gpd.GeoSeries(list(buffered_nodes))
        cleaned_intersections = unified_intersections.centroid
        cleaned_intersections.crs = gdf_proj.crs
        cleaned_intersections = cleaned_intersections.to_crs(original_crs)
    
    except Exception as e:
        cleaned_intersections = gpd.GeoSeries([])
        print('calculate_cleaned_intersections error', e)
    
    return cleaned_intersections



def calculate_edge_length_ratios(G, planar_intersections, buffer_size=0.001): #buff size 1mm
    
    edges = ox.graph_to_gdfs(G, nodes=False, fill_edge_geometry=True)
    points = gpd.GeoDataFrame(geometry=planar_intersections)
    points.crs = edges.crs
    
    edges = ox.project_gdf(edges)
    points = ox.project_gdf(points).unary_union.buffer(buffer_size)
    
    segments = []
    for e in edges['geometry']:

        diff = e.difference(points)

        if type(diff) == LineString:
            segments.append(diff)
        elif type(diff) == MultiLineString:
            for ls in diff:
                segments.append(ls)
        elif diff.is_empty:
        	pass
        else:
            print('error: ', type(diff))
            
    mean_planar_segment_length = gpd.GeoSeries(segments).length.mean()
    mean_edge_length = edges['geometry'].length.mean()
    edge_length_ratio = mean_planar_segment_length / mean_edge_length
    
    return mean_edge_length, mean_planar_segment_length, edge_length_ratio



