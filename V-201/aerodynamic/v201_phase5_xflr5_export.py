import os
import csv
import xml.etree.ElementTree as ET
from xml.dom import minidom

def export_xflr5():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    
    csv_path = os.path.join(project_dir, "optimization_results", "v201_phase4_wing_stations.csv")
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        return

    stations = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            stations.append({
                "y_mm": float(row["y_mm"]),
                "chord_mm": float(row["chord_mm"]),
                "twist_deg": float(row["twist_deg"]),
                "panel_dihedral_deg": float(row["panel_dihedral_deg"]),
                "leading_edge_x_mm": float(row["leading_edge_x_mm"]),
                "leading_edge_z_mm": float(row["leading_edge_z_mm"]),
            })

    # Create XML structure
    explane = ET.Element("explane", version="1.0")
    plane = ET.SubElement(explane, "Plane")
    ET.SubElement(plane, "Name").text = "V201_Phase4"
    
    wing = ET.SubElement(plane, "wing")
    ET.SubElement(wing, "Name").text = "Main Wing"
    ET.SubElement(wing, "Type").text = "MAINWING"
    ET.SubElement(wing, "Position").text = "0,0,0"
    ET.SubElement(wing, "Tilt_angle").text = "0"
    ET.SubElement(wing, "Symetric").text = "true"
    ET.SubElement(wing, "isFin").text = "false"
    ET.SubElement(wing, "isDoubleFin").text = "false"
    ET.SubElement(wing, "isSymFin").text = "false"
    
    sections = ET.SubElement(wing, "Sections")
    
    root_le_x = stations[0]["leading_edge_x_mm"]
    
    for i, st in enumerate(stations):
        sec = ET.SubElement(sections, "Section")
        
        y_m = st["y_mm"] / 1000.0
        chord_m = st["chord_mm"] / 1000.0
        # XFLR5 expects offset relative to the wing's position (0,0,0)
        x_offset_m = (st["leading_edge_x_mm"] - root_le_x) / 1000.0
        
        # Dihedral in XFLR5 is the angle of the panel starting at this section
        dihedral_deg = st["panel_dihedral_deg"]
        
        twist_deg = st["twist_deg"]
        
        ET.SubElement(sec, "y_position").text = f"{y_m:.6f}"
        ET.SubElement(sec, "Chord").text = f"{chord_m:.6f}"
        ET.SubElement(sec, "xOffset").text = f"{x_offset_m:.6f}"
        ET.SubElement(sec, "Dihedral").text = f"{dihedral_deg:.6f}"
        ET.SubElement(sec, "Twist").text = f"{twist_deg:.6f}"
        
        ET.SubElement(sec, "x_number_of_panels").text = "13"
        ET.SubElement(sec, "x_panel_distribution").text = "COSINE"
        ET.SubElement(sec, "y_number_of_panels").text = "13"
        ET.SubElement(sec, "y_panel_distribution").text = "SINE"
        
        y_str = f"{int(st['y_mm']):04d}"
        foil_filename = f"v201_phase4_station_{i:02d}_y{y_str}.dat"
        foil_path = os.path.join(project_dir, "optimization_results", "airfoils", foil_filename)
        
        # XFLR5 uses the first line of the dat file as the airfoil name
        with open(foil_path, "r", encoding="utf-8") as f:
            foil_name = f.readline().strip()
        
        ET.SubElement(sec, "Left_Side_FoilName").text = foil_name
        ET.SubElement(sec, "Right_Side_FoilName").text = foil_name

    # Write out the XML
    xml_str = ET.tostring(explane, encoding="utf-8")
    parsed_xml = minidom.parseString(xml_str)
    pretty_xml = parsed_xml.toprettyxml(indent="  ")
    
    out_dir = os.path.join(project_dir, "optimization_results", "xflr5_export")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "v201_phase4_wing.xml")
    
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(pretty_xml)
        
    print(f"Exported XFLR5 wing XML to {out_path}")
    print("To use in XFLR5:")
    print("1. File -> Direct Foil Design -> Open... and select all .dat files in optimization_results/airfoils/")
    print("2. File -> Wing and Plane Design -> Plane -> Import -> select the generated XML file")

if __name__ == "__main__":
    export_xflr5()
