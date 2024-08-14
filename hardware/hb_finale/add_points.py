import argparse
import yaml

def make_points(file_name, list_of_contours):
    # read yaml file
    with open("all_contours.yaml", 'r') as file:
        data = yaml.safe_load(file)
    
    # Write in a file named file_name
    with open(file_name, 'w') as file:
        file.write("\n\n")
        for contour_id in list_of_contours:
            contour_id = int(contour_id)
            if contour_id<0:
                contour_id = abs(contour_id)
                contour = data[contour_id]["contours"]
                contour = contour[::-1]
            else:
                contour = data[contour_id]["contours"]
            for point in contour:
                file.write(f"{point}\n")
            file.write("\n\n\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Add points from one contours to another file.")
    parser.add_argument("to_file", type=str, help="The file to write points to.")
    parser.add_argument("-contours", type=str, help="List of comma seperated points representing contour points with sign")
    args = parser.parse_args()
    contours = args.contours.split(",")
    make_points(args.to_file, contours)