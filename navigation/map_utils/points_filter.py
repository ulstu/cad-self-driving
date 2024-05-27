import sys
import re
from geopy.distance import geodesic

def read_coordinates(file_name):
    coordinates = []
    with open(file_name, 'r') as file:
        for line in file:
            match = re.match(r'\[(\d+\.\d+),(\d+\.\d+)\]', line.strip())
            if match:
                lat, lon = match.groups()
                coordinates.append((float(lat), float(lon)))
    return coordinates

def write_coordinates(file_name, coordinates):
    with open(file_name, 'w') as file:
        for coord in coordinates:
            file.write(f'[{coord[0]},{coord[1]}],\n')

def filter_coordinates(coordinates, min_distance):
    if not coordinates:
        return []

    filtered_coords = [coordinates[0]]
    for coord in coordinates[1:]:
        if geodesic(filtered_coords[-1], coord).meters >= min_distance:
            filtered_coords.append(coord)

    return filtered_coords

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python points_filter.py <input_file> <output_file> <distance>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    distance = float(sys.argv[3])

    coordinates = read_coordinates(input_file)
    filtered_coordinates = filter_coordinates(coordinates, distance)
    write_coordinates(output_file, filtered_coordinates)
    print(f"Filtered {len(filtered_coordinates)} points from {len(coordinates)} total points.")
