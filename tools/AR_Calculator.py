import argparse
import sys

def calculate_aspect_ratio_area(wingspan, area):
    """Calculate Aspect Ratio using wingspan and wing area."""
    return (wingspan ** 2) / area

def calculate_aspect_ratio_chord(wingspan, average_chord):
    """Calculate Aspect Ratio using wingspan and average chord."""
    return wingspan / average_chord

def main():
    print("=== Wing Aspect Ratio Calculator ===")
    print("Choose calculation method:")
    print("1. Using Wingspan and Wing Area (AR = b^2 / S)")
    print("2. Using Wingspan and Average Chord (AR = b / c)")
    
    choice = input("Enter choice (1 or 2): ").strip()
    
    try:
        if choice == '1':
            wingspan = float(input("Enter Wingspan: "))
            area = float(input("Enter Wing Area (in same base units squared): "))
            if area <= 0:
                print("Error: Area must be greater than 0.")
                return
            ar = calculate_aspect_ratio_area(wingspan, area)
            print(f"\nAspect Ratio (AR): {ar:.2f}")
            
        elif choice == '2':
            wingspan = float(input("Enter Wingspan: "))
            chord = float(input("Enter Average Chord (in same units as wingspan): "))
            if chord <= 0:
                print("Error: Chord must be greater than 0.")
                return
            ar = calculate_aspect_ratio_chord(wingspan, chord)
            print(f"\nAspect Ratio (AR): {ar:.2f}")
            
        else:
            print("Invalid choice. Please run the script again.")
            
    except ValueError:
        print("Invalid input. Please enter numerical values.")

if __name__ == "__main__":
    main()
