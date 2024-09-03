from stl import mesh
import numpy as np

def extract_dimensions_from_stl(stl_file):
    # Load the STL file
    stl_mesh = mesh.Mesh.from_file(stl_file)

    # Get all the points in the mesh
    points = stl_mesh.vectors.reshape(-1, 3)

    # Calculate the bounds
    min_coords = np.min(points, axis=0)
    max_coords = np.max(points, axis=0)

    # Calculate dimensions
    dimensions = max_coords - min_coords
    length, width, height = dimensions

    return length, width, height

# Example usage:
stl_file = 'foot_L_Link.STL'
length, width, height = extract_dimensions_from_stl(stl_file)
print(f"Length: {length}, Width: {width}, Height: {height}")
