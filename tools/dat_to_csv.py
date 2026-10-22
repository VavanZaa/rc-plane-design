import csv
import sys
import os

def convert_dat_to_csv(input_file):
    if not input_file.lower().endswith('.dat'):
        print(f"Skipping '{input_file}': Not a .dat file")
        return

    output_file = os.path.splitext(input_file)[0] + '.csv'
    
    with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
        
    data_points = []
    dimension = None
    
    for line_num, line in enumerate(lines):
        # Skip empty lines
        parts = line.strip().split()
        if not parts:
            continue
            
        # Try to parse the row as floats
        try:
            coords = [float(p) for p in parts]
            
            # Skip lines that are just point counts in some airfoil formats (e.g. Lednicer "30. 30.")
            if len(coords) == 2 and line_num < 5:
                if all(c.is_integer() for c in coords) and all(c > 5 for c in coords):
                    continue
            
            if dimension is None:
                if len(coords) == 2:
                    dimension = '2D'
                    print(f"[{os.path.basename(input_file)}] Detected 2D data ({len(coords)} columns).")
                elif len(coords) == 3:
                    dimension = '3D'
                    print(f"[{os.path.basename(input_file)}] Detected 3D data ({len(coords)} columns).")
                else:
                    dimension = f'{len(coords)}D'
                    print(f"[{os.path.basename(input_file)}] Detected {dimension} data ({len(coords)} columns).")
                    
            if dimension == '2D' and len(coords) == 2:
                coords.append(0.0)

            data_points.append(coords)
            
        except ValueError:
            # Header or text line, just skip
            continue
            
    if not data_points:
        print(f"No valid coordinate data found in '{input_file}'.")
        return
        
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(data_points)
        
    print(f"Successfully converted to '{output_file}' with {len(data_points)} points.")

if __name__ == "__main__":
    # If file arguments are passed via command line (or drag and drop)
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            convert_dat_to_csv(arg)
    else:
        # Otherwise, process all .dat files in the same directory as the script
        directory = os.path.dirname(os.path.abspath(__file__))
        dat_files = [f for f in os.listdir(directory) if f.lower().endswith('.dat')]
        
        if not dat_files:
            print("No .dat files found in the directory.")
        else:
            print(f"Found {len(dat_files)} .dat files to process.")
            for f in dat_files:
                convert_dat_to_csv(os.path.join(directory, f))
